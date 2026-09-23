// The surface. A timer wakes a worker, the worker copies the volume in chunks and
// marches cubes over the copy holding no lock, and what comes out is cleaned,
// capped, and published as a Marker.

#include "pimesh_mapping/nodes/mesh_node.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <sys/resource.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace pimesh_mapping
{
namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & text)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = text;
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describe_double(
  const std::string & text, double low, double high)
{
  auto descriptor = describe(text);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = low;
  range.to_value = high;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describe_int(
  const std::string & text, int low, int high)
{
  auto descriptor = describe(text);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = low;
  range.to_value = high;
  descriptor.integer_range.push_back(range);
  return descriptor;
}

double ms_since(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

std::string timestamped_name()
{
  const std::time_t now = std::time(nullptr);
  std::tm parts {};
  localtime_r(&now, &parts);
  char buffer[64];
  std::strftime(buffer, sizeof(buffer), "mesh_%Y%m%d_%H%M%S.ply", &parts);
  return buffer;
}

}  // namespace

MeshNode::MeshNode(const rclcpp::NodeOptions & options)
: Node("mesh_node", options)
{
  const std::string mesh_topic = declare_parameter(
    "mesh_topic", std::string("/world/mesh"),
    describe("TRIANGLE_LIST Marker of the reconstructed surface, capped and latched."));
  world_frame_ = declare_parameter(
    "world_frame", std::string("map"),
    describe("The frame the volume lives in. Must match fusion_node's."));
  volume_key_ = declare_parameter(
    "volume_key", std::string("world"),
    describe(
      "Which process-local volume to mesh. fusion_node fills the same key; a "
      "mismatch is two volumes and a mesh that is silently always empty."));
  marker_ns_ = declare_parameter(
    "marker_ns", std::string("world_mesh"),
    describe("Marker namespace, so a viewer can toggle this surface on its own."));

  const double remesh_period_s = declare_parameter(
    "remesh_period_s", 10.0,
    describe_double(
      "How often to extract a surface. Human rate, not frame rate: accumulation "
      "happens at 16 Hz and this is what a person can follow.", 0.5, 600.0));

  mesh_min_weight_ = static_cast<float>(declare_parameter(
      "mesh_min_weight", 3.0,
      describe_double(
        "Observations a voxel needs before it is meshed. **Not the same question "
        "as the volume's own threshold**, which is the noise floor for "
        "ray-casting: a voxel seen three times is real enough to stop a ray and "
        "thin enough to be one of the shingles a sweep lays down. Raising this "
        "is the honest lever on a mesh that is mostly layers of the same wall.",
        1.0, 1000.0)));
  min_component_triangles_ = static_cast<std::size_t>(declare_parameter(
      "min_component_triangles", 30,
      describe_int(
        "Connected components smaller than this are debris and are dropped. "
        "Their outer boundaries masquerade as holes; capping them would turn "
        "noise flakes into blobs.", 0, 100000)));
  max_hole_radius_m_ = declare_parameter(
    "max_hole_radius_m", 0.25,
    describe_double(
      "Interior boundary loops narrower than this are fanned shut. Each "
      "component's *largest* loop is its frontier and is never closed whatever "
      "its size — unseen space is not invented here.", 0.0, 10.0));
  max_triangles_ = static_cast<std::size_t>(declare_parameter(
      "max_triangles", 120000,
      describe_int(
        "Cap on the published Marker, reached by quadric decimation and never by "
        "subsampling. The saved PLY is full detail: a Marker is rebuilt and "
        "re-serialised on every publish, a file is written once.", 1000, 5000000)));
  snapshot_chunk_ = static_cast<std::size_t>(declare_parameter(
      "snapshot_chunk_blocks", 2048,
      describe_int(
        "Blocks copied per lock acquisition. 2048 blocks is ~12 MB and a few "
        "milliseconds; one lock over a 1.3 GB map would hold the integrator out "
        "for over a hundred, which is the dip gates/mesh.sh asserts is absent.",
        1, 1000000)));
  worker_nice_ = static_cast<int>(declare_parameter(
      "worker_nice", 10,
      describe_int(
        "Nice level for the extraction thread. **The mesher must lose to the "
        "pipeline, and at equal priority it does not.** Measured on bags/desk1: "
        "while an extraction ran, depth_node's rate fell from 17.8 to 14.6 Hz "
        "with its per-frame cost unchanged at 55.9 ms — so it was not doing more "
        "work, it was not being scheduled, and fusion inherited a 404 ms gap "
        "between integrations. Marching cubes and quadric decimation are seconds "
        "of pure CPU with no deadline; every stage above them has one.",
        0, 19)));
  save_dir_ = declare_parameter(
    "save_dir", std::string(""),
    describe(
      "Where a SaveMesh request with an empty path writes. Empty means the "
      "working directory the container was started in."));

  volume_ = VolumeRegistry::get(volume_key_);

  // --- QoS ------------------------------------------------------------------
  //
  // **Latched (transient local) on the mesh**, which is not the usual choice in
  // this workspace and is right here. Everywhere else the rule is KEEP_LAST(1)
  // volatile — freshest frame, no backlog — because the data is a stream and a
  // late subscriber wants the next one. This is not a stream: it changes every
  // ten seconds, and an RViz started between two extractions would otherwise show
  // an empty 3D view for up to ten seconds and look exactly like a dead topic.
  rclcpp::QoS mesh_qos(rclcpp::KeepLast(1));
  mesh_qos.reliable().transient_local();

  mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>(mesh_topic, mesh_qos);
  stats_pub_ = create_publisher<pimesh_msgs::msg::MeshStats>("/world/mesh_stats", mesh_qos);
  pipeline_pub_ = create_publisher<pimesh_msgs::msg::PipelineStats>("/pipeline/stats", 10);

  save_service_ = create_service<pimesh_msgs::srv::SaveMesh>(
    "/world/save_mesh",
    [this](
      const std::shared_ptr<pimesh_msgs::srv::SaveMesh::Request> request,
      std::shared_ptr<pimesh_msgs::srv::SaveMesh::Response> response) {
      on_save(request, response);
    });
  reset_service_ = create_service<pimesh_msgs::srv::ResetMap>(
    "/world/reset_map",
    [this](
      const std::shared_ptr<pimesh_msgs::srv::ResetMap::Request> request,
      std::shared_ptr<pimesh_msgs::srv::ResetMap::Response> response) {
      on_reset(request, response);
    });

  worker_ = std::thread([this] {work();});

  // The timer *asks* for a re-mesh; it does not do one. A timer callback that
  // takes four seconds blocks this node's executor and every callback it owns,
  // including the two services above — so a `save_mesh` call would hang for the
  // duration of an extraction it had nothing to do with.
  timer_ = create_wall_timer(
    std::chrono::duration<double>(remesh_period_s), [this] {request_remesh();});

  RCLCPP_INFO(
    get_logger(),
    "mesh_node up: volume '%s' -> %s every %.1f s, weight >= %.0f, components >= %zu, "
    "holes < %.2f m, cap %zu triangles",
    volume_key_.c_str(), mesh_topic.c_str(), remesh_period_s,
    static_cast<double>(mesh_min_weight_), min_component_triangles_,
    max_hole_radius_m_, max_triangles_);
}

MeshNode::~MeshNode()
{
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    stopping_ = true;
  }
  wake_.notify_all();
  if (worker_.joinable()) {worker_.join();}
}

void MeshNode::request_remesh()
{
  {
    std::lock_guard<std::mutex> lock(wake_mutex_);
    // Set rather than counted: if an extraction overran its period there is
    // nothing to be gained by queueing a second one behind it. Newest wins, the
    // same rule the mailbox implements one stage up.
    wanted_ = true;
  }
  wake_.notify_one();
}

void MeshNode::work()
{
  // Linux nice is **per thread**, which is what makes this the right tool: the
  // node's executor thread, its services and its timer all keep normal priority
  // and only the extraction is pushed down. Raising nice never needs privileges,
  // so a failure here is a curiosity rather than a fault — reported, not fatal.
  if (worker_nice_ != 0 && setpriority(PRIO_PROCESS, 0, worker_nice_) != 0) {
    RCLCPP_WARN(
      get_logger(), "could not nice the extraction thread to %d — it will compete "
      "with the pipeline for CPU", worker_nice_);
  }

  while (true) {
    {
      std::unique_lock<std::mutex> lock(wake_mutex_);
      wake_.wait(lock, [this] {return wanted_ || stopping_;});
      if (stopping_) {return;}
      wanted_ = false;
    }
    remesh();
  }
}

void MeshNode::remesh()
{
  if (!volume_->configured()) {
    // Not an error at startup — this node may have loaded before fusion_node — but
    // it is an error if it never resolves, and the message names the key so a
    // mismatch is diagnosable rather than looking like an empty room.
    if (!warned_no_volume_.exchange(true)) {
      RCLCPP_WARN(
        get_logger(),
        "no configured volume under key '%s' (registered: %s) — waiting. "
        "mesh_node needs a fusion_node in the *same process*: the volume is "
        "hundreds of megabytes and is shared by pointer, not published. See "
        "pimesh_mapping/shared_volume.hpp.",
        volume_key_.c_str(), VolumeRegistry::keys().c_str());
    }
    return;
  }
  warned_no_volume_.store(false);

  const auto started = std::chrono::steady_clock::now();

  // The short lock, repeated. Chunked so the integrator interleaves with it —
  // see SharedVolume::snapshot for the measurement that makes this necessary.
  // Filtered by the meshing threshold: blocks with nothing in them the mesher
  // would believe are not copied. Same surface, a fraction of the memory — see
  // SharedVolume::snapshot for the stall that made it necessary.
  const SharedVolume::Snapshot snapshot = volume_->snapshot(snapshot_chunk_, mesh_min_weight_);
  const double copy_ms = ms_since(started);
  if (snapshot.blocks.empty()) {return;}

  // Counted on the copy, off the integration path. It is the second half of the
  // MeshStats pair whose *gap* is the weight threshold doing its job — a volume
  // where allocated and meshed are equal is one that is meshing its own noise.
  std::size_t voxels_meshed = 0;
  const float weight = std::max(mesh_min_weight_, snapshot.min_weight);
  // A histogram beside the count, logged once per extraction. `mesh_min_weight`
  // is the one lever on a mesh that is mostly layers of the same wall, and
  // choosing it by eye against a picture is how a number gets tuned to make an
  // image look right. This is the evidence to choose it from.
  std::size_t buckets[5] = {0, 0, 0, 0, 0};
  const float edges[5] = {1.0F, 4.0F, 8.0F, 16.0F, 32.0F};
  for (const auto & entry : snapshot.blocks) {
    for (int i = 0; i < TsdfVolume::kBlockVoxels; ++i) {
      const float w = entry.second.voxels[i].weight;
      if (w >= weight) {++voxels_meshed;}
      for (int b = 0; b < 5; ++b) {
        if (w >= edges[b]) {++buckets[b];}
      }
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "weights blocks=%zu copy=%.0fms >=1:%zu >=4:%zu >=8:%zu >=16:%zu >=32:%zu",
    snapshot.blocks.size(), copy_ms, buckets[0], buckets[1], buckets[2], buckets[3],
    buckets[4]);

  const auto march_started = std::chrono::steady_clock::now();
  Mesh mesh = march_cubes(snapshot, mesh_min_weight_);
  const double march_ms = ms_since(march_started);
  const std::size_t raw_triangles = mesh.triangle_count();
  // Logged before the cleanup rather than only at the end, because the stages
  // after this one are the expensive ones and a re-mesh that does not finish
  // otherwise leaves no trace of how big the problem was.
  RCLCPP_INFO(
    get_logger(), "marched %zu triangles from %zu vertices in %.0f ms",
    raw_triangles, mesh.vertex_count(), march_ms);

  // **A ceiling on what decimation is asked to do.** Reducing a few times the cap
  // is what quadric decimation is for; reducing twenty times it is a signal that
  // the *map* is wrong, not the mesh — a shingled volume produces a surface that
  // is mostly layers of the same wall, and grinding it down to 120 000 triangles
  // takes minutes and produces an average of the layers. The lever is
  // mesh_min_weight, and this says so rather than appearing to hang.
  if (raw_triangles > max_triangles_ * 40) {
    RCLCPP_WARN(
      get_logger(),
      "%zu triangles is more than 20x the %zu cap — decimating that is minutes of "
      "work and the result is an average of however many layers of wall are in "
      "there. Raise mesh_min_weight (the histogram above is the evidence for a "
      "value) rather than waiting. Publishing nothing this round.",
      raw_triangles, max_triangles_);
    return;
  }

  const auto clean_started = std::chrono::steady_clock::now();
  const std::size_t pruned = prune_small_components(mesh, min_component_triangles_);
  const std::size_t loops_before = boundary_loops(mesh).size();
  const std::size_t filled = fill_interior_holes(mesh, max_hole_radius_m_);
  const std::size_t loops_after = boundary_loops(mesh).size();
  const double clean_ms = ms_since(clean_started);

  // The full-detail surface, kept for /world/save_mesh. Copied *before*
  // decimation, which is the whole distinction between the file and the Marker.
  {
    std::lock_guard<std::mutex> lock(mesh_mutex_);
    full_mesh_ = mesh;
  }

  const auto decimate_started = std::chrono::steady_clock::now();
  const std::size_t collapses = decimate(mesh, max_triangles_);
  const double decimate_ms = ms_since(decimate_started);

  const double total_ms = ms_since(started);
  last_duration_ms_.store(total_ms);
  remeshes_.fetch_add(1, std::memory_order_relaxed);

  const auto stamp = now();
  publish_marker(mesh, stamp);
  publish_stats(mesh, snapshot, voxels_meshed, total_ms, stamp);

  RCLCPP_INFO(
    get_logger(),
    "mesh blocks=%zu voxels_meshed=%zu raw_tri=%zu pruned=%zu filled=%zu/%zu "
    "loops=%zu->%zu collapses=%zu tri=%zu vert=%zu "
    "copy=%.0f march=%.0f clean=%.0f decimate=%.0f total=%.0f ms",
    snapshot.blocks.size(), voxels_meshed, raw_triangles, pruned, filled, loops_before,
    loops_before, loops_after, collapses, mesh.triangle_count(), mesh.vertex_count(),
    copy_ms, march_ms, clean_ms, decimate_ms, total_ms);
}

void MeshNode::publish_marker(const Mesh & mesh, const rclcpp::Time & stamp)
{
  auto marker = std::make_unique<visualization_msgs::msg::Marker>();
  marker->header.frame_id = world_frame_;
  marker->header.stamp = stamp;
  marker->ns = marker_ns_;
  marker->id = 0;
  marker->type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->pose.orientation.w = 1.0;
  marker->scale.x = 1.0;
  marker->scale.y = 1.0;
  marker->scale.z = 1.0;
  // **The base colour's alpha has to be non-zero for per-vertex colours to apply
  // at all.** RViz multiplies the two, so a Marker with `color.a = 0` and a
  // perfectly good `colors` array renders as nothing whatsoever — an empty 3D
  // view that looks exactly like a topic nobody is publishing.
  marker->color.r = 1.0;
  marker->color.g = 1.0;
  marker->color.b = 1.0;
  marker->color.a = 1.0;

  // TRIANGLE_LIST carries no index buffer: every triangle contributes its three
  // vertices verbatim. This flattening is the format, and it is the last thing
  // that happens — everything before it needed the connectivity the indices are.
  marker->points.resize(mesh.triangles.size() * 3);
  marker->colors.resize(mesh.triangles.size() * 3);
  std::size_t out = 0;
  for (const auto & tri : mesh.triangles) {
    for (int i = 0; i < 3; ++i) {
      const auto & v = mesh.vertices[tri[i]];
      const auto & c = mesh.colours[tri[i]];
      marker->points[out].x = v[0];
      marker->points[out].y = v[1];
      marker->points[out].z = v[2];
      marker->colors[out].r = std::min(1.0F, std::max(0.0F, c[0]));
      marker->colors[out].g = std::min(1.0F, std::max(0.0F, c[1]));
      marker->colors[out].b = std::min(1.0F, std::max(0.0F, c[2]));
      marker->colors[out].a = 1.0F;
      ++out;
    }
  }
  mesh_pub_->publish(std::move(marker));
}

void MeshNode::publish_stats(
  const Mesh & mesh, const SharedVolume::Snapshot & snapshot,
  std::size_t voxels_meshed, double duration_ms, const rclcpp::Time & stamp)
{
  // **Read out of the message before it is published, not after.** `publish`
  // takes the unique_ptr by value and moves from it, which leaves this one null —
  // so a later `stats->triangles` is a null dereference, and it is one the
  // compiler is perfectly happy with. It cost an afternoon here: the container
  // segfaulted immediately after a re-mesh that had done every hard thing
  // correctly, and the whole cleanup replayed offline on the very same mesh
  // without a murmur, because the offline harness never published anything. The
  // stage that crashes is not always the stage that is wrong, and a use-after-move
  // is invisible to every test that does not run the real transport.
  const auto triangle_count = static_cast<std::uint32_t>(mesh.triangle_count());
  // The *volume's* count, carried on the snapshot, not the filtered copy's: the
  // gap between allocated and meshed is what MeshStats is for, and measuring the
  // first on a copy that was already filtered by the second would close it by
  // construction.
  const auto voxels_allocated = snapshot.voxels_allocated;

  auto stats = std::make_unique<pimesh_msgs::msg::MeshStats>();
  stats->header.stamp = stamp;
  stats->header.frame_id = world_frame_;
  stats->vertices = static_cast<std::uint32_t>(mesh.vertex_count());
  stats->triangles = triangle_count;
  stats->voxels_allocated = voxels_allocated;
  stats->voxels_meshed = voxels_meshed;
  stats->voxel_size_m = snapshot.voxel_size_m;
  stats->mesh_duration_ms = static_cast<float>(duration_ms);
  stats->frames_integrated = snapshot.frames_integrated;

  // The extent of the *meshed surface*, not of the allocated volume: the two
  // differ by whatever the weight threshold excluded, and a viewer told the
  // second would draw a bounding box around noise.
  if (!mesh.vertices.empty()) {
    cv::Vec3f low = mesh.vertices.front();
    cv::Vec3f high = mesh.vertices.front();
    for (const auto & v : mesh.vertices) {
      for (int i = 0; i < 3; ++i) {
        low[i] = std::min(low[i], v[i]);
        high[i] = std::max(high[i], v[i]);
      }
    }
    stats->volume_min.x = low[0];
    stats->volume_min.y = low[1];
    stats->volume_min.z = low[2];
    stats->volume_max.x = high[0];
    stats->volume_max.y = high[1];
    stats->volume_max.z = high[2];
  }
  stats_pub_->publish(std::move(stats));

  auto pipeline = std::make_unique<pimesh_msgs::msg::PipelineStats>();
  pipeline->header.stamp = stamp;
  pipeline->header.frame_id = world_frame_;
  pipeline->stage = "mesh";
  const auto count = remeshes_.load(std::memory_order_relaxed);
  pipeline->rate_hz = duration_ms > 0.0 ? static_cast<float>(1000.0 / duration_ms) : 0.0F;
  pipeline->latency_ms = static_cast<float>(duration_ms);
  pipeline->latency_p95_ms = static_cast<float>(duration_ms);
  pipeline->frames_in = snapshot.frames_integrated;
  pipeline->frames_out = count;
  char detail[256];
  std::snprintf(
    detail, sizeof(detail), "%u triangles, %zu blocks, %.1f%% of voxels meshed",
    triangle_count, snapshot.blocks.size(),
    voxels_allocated ? 100.0 * static_cast<double>(voxels_meshed) /
    static_cast<double>(voxels_allocated) : 0.0);
  pipeline->detail = detail;
  pipeline_pub_->publish(std::move(pipeline));
}

void MeshNode::on_save(
  const std::shared_ptr<pimesh_msgs::srv::SaveMesh::Request> request,
  std::shared_ptr<pimesh_msgs::srv::SaveMesh::Response> response)
{
  Mesh mesh;
  {
    std::lock_guard<std::mutex> lock(mesh_mutex_);
    // A copy under the lock rather than writing the file under it: a PLY of a
    // million triangles is tens of megabytes and the worker would be blocked out
    // of its next extraction for the duration of a disk write.
    mesh = full_mesh_;
  }
  if (mesh.empty()) {
    response->success = false;
    response->message =
      "nothing has been meshed yet — either no frames have been integrated or the "
      "first extraction has not run";
    return;
  }

  std::string path = request->path;
  if (path.empty()) {
    path = save_dir_.empty() ? timestamped_name() : (save_dir_ + "/" + timestamped_name());
  }

  std::string error;
  if (!write_ply(mesh, path, error)) {
    response->success = false;
    response->message = error;
    RCLCPP_WARN(get_logger(), "save_mesh failed: %s", error.c_str());
    return;
  }

  response->success = true;
  response->path = path;
  response->triangles = static_cast<std::uint32_t>(mesh.triangle_count());
  response->message = "full detail, before the Marker's decimation";
  RCLCPP_INFO(
    get_logger(), "saved %zu triangles to %s", mesh.triangle_count(), path.c_str());
}

void MeshNode::on_reset(
  const std::shared_ptr<pimesh_msgs::srv::ResetMap::Request>,
  std::shared_ptr<pimesh_msgs::srv::ResetMap::Response> response)
{
  if (!volume_->configured()) {
    response->success = false;
    response->message = "no configured volume under key '" + volume_key_ + "'";
    return;
  }
  const std::size_t blocks = volume_->reset();
  {
    std::lock_guard<std::mutex> lock(mesh_mutex_);
    full_mesh_.clear();
  }
  // An empty Marker, published straight away rather than at the next tick. A
  // latched topic still holding the old surface after a reset is a viewer showing
  // a room that has been thrown away, and nothing says so.
  publish_marker(Mesh(), now());
  response->success = true;
  response->message = "cleared " + std::to_string(blocks) + " blocks";
  RCLCPP_INFO(get_logger(), "map reset: %zu blocks cleared", blocks);
}

}  // namespace pimesh_mapping

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_mapping::MeshNode)
