// One tab that shows the pipeline, and a rule that it must never slow it down.

#include "pimesh_dashboard/dashboard_node.hpp"
#include "pimesh_dashboard/json.hpp"
#include "pimesh_dashboard/mesh_payload.hpp"

#include <algorithm>
#include <cstdio>
#include <sstream>
#include <utility>

// get_package_share_path, not get_package_share_directory: the latter is
// deprecated on Lyrical and both exist on Jazzy, so this is the spelling that is
// current at both ends — the same choice pimesh_camera made for the same reason.
#include "ament_index_cpp/get_package_share_path.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace pimesh_dashboard
{
namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & text)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = text;
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describe_int(
  const std::string & text, std::int64_t low, std::int64_t high)
{
  auto descriptor = describe(text);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = low;
  range.to_value = high;
  descriptor.integer_range.push_back(range);
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

}  // namespace

DashboardNode::DashboardNode(const rclcpp::NodeOptions & options)
: Node("dashboard_node", options)
{
  WebServer::Config config;
  config.port = static_cast<int>(
    declare_parameter(
      "port", 8080,
      describe_int("TCP port for the page and its WebSocket.", 1, 65535)));
  const std::string root = declare_parameter(
    "web_root", std::string(),
    describe(
      "Directory the page is served from. Empty means the package's own "
      "share/web, which is where the vendored assets are installed — there is no "
      "CDN here on purpose: the LAN may have no internet and a monitoring tool "
      "that breaks when DNS does is not one."));
  config.root = root.empty() ?
    (ament_index_cpp::get_package_share_path("pimesh_dashboard") / "web").string() : root;
  config.send_limit_bytes = static_cast<std::size_t>(
    declare_parameter(
      "send_limit_bytes", 8 * 1024 * 1024,
      describe_int(
        "Per-client queued bytes past which frames are dropped and counted. This "
        "is the pacing rule in one number: a backgrounded browser stops reading, "
        "and without a ceiling the queue grows until the pipeline slows down.",
        64 * 1024, 512 * 1024 * 1024)));
  config.max_clients = static_cast<std::size_t>(
    declare_parameter("max_clients", 8, describe_int("Connected viewers allowed.", 1, 128)));

  const double stats_rate_hz = declare_parameter(
    "stats_rate_hz", 10.0,
    describe_double("How often the panel and the pose are sent.", 0.5, 60.0));
  rgb_period_s_ = 1.0 / declare_parameter(
    "rgb_rate_hz", 10.0, describe_double("Cap on the camera strip.", 0.5, 60.0));
  depth_period_s_ = 1.0 / declare_parameter(
    "depth_rate_hz", 5.0, describe_double("Cap on the depth strip.", 0.5, 60.0));
  stale_after_s_ = declare_parameter(
    "stale_after_s", 2.0,
    describe_double(
      "Seconds without a message before a row is marked STALE. Measured on "
      "receipt, never on header.stamp: the capture row is stamped on the Pi and "
      "read here, and that difference is NTP's business.", 0.2, 60.0));
  trail_limit_ = static_cast<std::size_t>(
    declare_parameter(
      "trail_points", 600,
      describe_int("Poses kept for the trajectory tail the page draws.", 2, 20000)));

  const std::string stats_topic = declare_parameter(
    "stats_topic", std::string("/pipeline/stats"),
    describe("Where every stage reports what it measured about itself."));
  const std::string rgb_topic = declare_parameter(
    "rgb_topic", std::string("/keypoints/image/compressed"),
    describe(
      "The annotated keypoint frame — one image, not two, since the corners are "
      "drawn on the RGB by the node that found them."));
  const std::string depth_topic = declare_parameter(
    "depth_topic", std::string("/depth/image/compressed"),
    describe(
      "The colour-mapped depth, as JPEG. **Colour-mapped in depth_node, not "
      "here**: a browser cannot map a 3.7 MB float image, and a fixed "
      "[0, max_range] scale is what makes a colour mean a distance across frames "
      "rather than within one."));
  const std::string odom_topic = declare_parameter(
    "odom_topic", std::string("/odom"), describe("The pose stream P7 publishes."));
  const std::string mesh_topic = declare_parameter(
    "mesh_topic", std::string("/world/mesh"), describe("The surface, as a Marker."));

  // --- QoS ------------------------------------------------------------------
  //
  // KeepLast(1) and best-effort on the two image strips: these are pictures for a
  // person at ~10 Hz, a dropped one costs nothing, and a RELIABLE reader would
  // ask the publisher to retransmit for a frame that is already out of date. The
  // stats topic is reliable with a queue, because a row arriving late is still
  // the truth about a window that happened and losing one shows as a STALE flag
  // for something that was fine.
  rclcpp::QoS image_qos(rclcpp::KeepLast(1));
  image_qos.best_effort();
  rclcpp::QoS stats_qos(rclcpp::KeepLast(20));
  stats_qos.reliable();
  rclcpp::QoS odom_qos(rclcpp::KeepLast(10));
  odom_qos.reliable();
  // The mesh is latched at the publisher, so a dashboard started after the last
  // extraction still gets a surface instead of a blank scene for ten seconds. A
  // VOLATILE reader against a TRANSIENT_LOCAL writer is *compatible* and simply
  // receives nothing — which this project has already spent a run discovering.
  rclcpp::QoS mesh_qos(rclcpp::KeepLast(1));
  mesh_qos.reliable().transient_local();

  stats_sub_ = create_subscription<pimesh_msgs::msg::PipelineStats>(
    stats_topic, stats_qos,
    [this](pimesh_msgs::msg::PipelineStats::ConstSharedPtr msg) {on_stats(std::move(msg));});
  rgb_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    rgb_topic, image_qos,
    [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {on_rgb(std::move(msg));});
  depth_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    depth_topic, image_qos,
    [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {on_depth(std::move(msg));});
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, odom_qos,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odom(std::move(msg));});
  mesh_sub_ = create_subscription<visualization_msgs::msg::Marker>(
    mesh_topic, mesh_qos,
    [this](visualization_msgs::msg::Marker::ConstSharedPtr msg) {on_mesh(std::move(msg));});

  save_dir_ = declare_parameter(
    "save_dir", std::string("/tmp"),
    describe(
      "Where the save button writes a PLY. The service takes a path and this "
      "node supplies the directory, because a browser must not get to choose "
      "where a process writes a file."));
  action_timeout_s_ = declare_parameter(
    "action_timeout_s", 10.0,
    describe_double(
      "How long a button waits for its service. Generous: saving a full-detail "
      "mesh is a few hundred thousand triangles to disk.", 0.5, 120.0));

  // **The services mesh_node already exposes, called rather than reimplemented.**
  // The dashboard does the same thing RViz would do and has no privileged path
  // into the map; if the surface can be saved from a terminal it can be saved
  // from here, and not otherwise.
  save_client_ = create_client<pimesh_msgs::srv::SaveMesh>("/world/save_mesh");
  reset_client_ = create_client<pimesh_msgs::srv::ResetMap>("/world/reset_map");

  started_ = std::chrono::steady_clock::now();
  last_rgb_ = started_;
  last_depth_ = started_;

  server_ = std::make_unique<WebServer>(config);
  if (!server_->start()) {
    // Fatal, and loudly. A dashboard that came up without its port is a process
    // that looks healthy in `ros2 node list` and serves nothing — which is the
    // exact failure shape this project refuses everywhere else.
    RCLCPP_FATAL(
      get_logger(), "could not listen: %s. Is another dashboard already running?",
      server_->error().c_str());
    throw std::runtime_error("dashboard_node: " + server_->error());
  }

  server_->set_action_handler(
    [this](const std::string & name) {return this->run_action(name);});

  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / stats_rate_hz)),
    [this] {this->tick();});

  RCLCPP_INFO(
    get_logger(),
    "dashboard up: http://localhost:%d  serving %s  stats %.0f Hz, rgb %.0f Hz, "
    "depth %.0f Hz, stale after %.1fs, %zu MB per client before dropping",
    config.port, config.root.c_str(), stats_rate_hz, 1.0 / rgb_period_s_,
    1.0 / depth_period_s_, stale_after_s_, config.send_limit_bytes / (1024u * 1024u));
}

DashboardNode::~DashboardNode()
{
  if (server_) {server_->stop();}
}

bool DashboardNode::due(std::chrono::steady_clock::time_point & last, double period_s)
{
  const auto now = std::chrono::steady_clock::now();
  // **The 0.9 is not slop, it is what stops two caps in series beating against
  // each other.** `keypoint_node` already throttles its preview to 10 Hz, and a
  // second 10 Hz gate here rejects any frame that arrives a hair early — which,
  // with two independent clocks, is about half of them. Measured before this:
  // the page received **5.48 Hz** of a 10 Hz stream and **2.80 Hz** of a 5 Hz cap
  // on a 10 Hz stream, and every number involved looked correct on its own.
  //
  // A cap is meant to be a ceiling on a faster source, so it has to admit a
  // source running at exactly its own rate.
  if (std::chrono::duration<double>(now - last).count() < period_s * 0.9) {return false;}
  last = now;
  return true;
}

void DashboardNode::on_stats(pimesh_msgs::msg::PipelineStats::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Row & row = rows_[msg->stage];
  row.stats = *msg;
  // **Receipt, not the stamp.** See the class comment: the capture row's stamp
  // comes off the Pi's clock.
  row.received = std::chrono::steady_clock::now();
}

void DashboardNode::on_rgb(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  if (!due(last_rgb_, rgb_period_s_)) {return;}
  server_->broadcast(Channel::Rgb, msg->data.data(), msg->data.size());
}

void DashboardNode::on_depth(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  if (!due(last_depth_, depth_period_s_)) {return;}
  server_->broadcast(Channel::Depth, msg->data.data(), msg->data.size());
}

void DashboardNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_odom_ = *msg;
  have_odom_ = true;
  odom_received_ = std::chrono::steady_clock::now();
  trail_.push_back(
    {static_cast<float>(msg->pose.pose.position.x),
      static_cast<float>(msg->pose.pose.position.y),
      static_cast<float>(msg->pose.pose.position.z)});
  if (trail_.size() > trail_limit_) {
    trail_.erase(trail_.begin(), trail_.begin() + static_cast<long>(trail_.size() - trail_limit_));
  }
}

void DashboardNode::on_mesh(visualization_msgs::msg::Marker::ConstSharedPtr msg)
{
  // **Sent on arrival, not on the tick**, because it arrives once every ten
  // seconds and holding it for a tick would add latency for nothing. It is the
  // one payload big enough to think about: mesh_node's cap of 120 k *triangles*
  // is 360 k vertices at 15 bytes each, so **5.4 MB** — an earlier version of
  // this comment said 4.3 MB, which is the position array with the colour bytes
  // forgotten. At one every ten seconds that is ~0.5 MB/s on a LAN, and it is
  // 64% of the server's 8 MiB send limit rather than the 51% the old figure
  // implied. Exceeding that limit is a silent drop, so the margin is worth
  // stating correctly; test_mesh_payload asserts it.
  // The layout, the refusals and the clamp are all in mesh_payload.hpp, where a
  // test can reach them: this is the one payload here whose reader is hand-written
  // JavaScript in another file, so the arithmetic is worth pinning byte for byte.
  const std::vector<std::uint8_t> payload = pack_mesh(msg->points, msg->colors);
  if (payload.empty()) {return;}

  ++mesh_versions_;
  server_->broadcast(Channel::Mesh, payload.data(), payload.size());
}

std::string DashboardNode::stats_json()
{
  const auto now = std::chrono::steady_clock::now();
  std::ostringstream out;
  out << "{\"uptime_s\":"
      << json::number(std::chrono::duration<double>(now - started_).count())
      << ",\"clients\":" << server_->clients()
      << ",\"ws_dropped\":" << server_->dropped()
      << ",\"mesh_versions\":" << mesh_versions_
      << ",\"stages\":[";

  std::lock_guard<std::mutex> lock(mutex_);
  bool first = true;
  for (const auto & entry : rows_) {
    const Row & row = entry.second;
    const double age = std::chrono::duration<double>(now - row.received).count();
    if (!first) {out << ",";}
    first = false;
    out << "{\"stage\":" << json::quote(entry.first)
        << ",\"rate_hz\":" << json::number(row.stats.rate_hz)
        << ",\"latency_ms\":" << json::number(row.stats.latency_ms)
        << ",\"latency_p95_ms\":" << json::number(row.stats.latency_p95_ms)
        << ",\"frames_in\":" << row.stats.frames_in
        << ",\"frames_out\":" << row.stats.frames_out
        // **Two fields, never summed.** depth_node dropping ~72% of what it is
        // offered is the single-slot mailbox working exactly as designed, and a
        // frame lost to QoS is a fault. One column for both would make the
        // healthy pipeline and the broken one look identical — which is the whole
        // reason PipelineStats carries them apart.
        << ",\"dropped_by_design\":" << row.stats.dropped_by_design
        << ",\"dropped_in_transport\":" << row.stats.dropped_in_transport
        << ",\"detail\":" << json::quote(row.stats.detail)
        << ",\"age_s\":" << json::number(age)
        << ",\"stale\":" << ((age > stale_after_s_) ? "true" : "false")
        << "}";
  }
  out << "]}";
  return out.str();
}

std::string DashboardNode::pose_json()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_odom_) {return "{\"have\":false}";}
  const double age =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - odom_received_).count();

  std::ostringstream out;
  const auto & p = last_odom_.pose.pose.position;
  const auto & q = last_odom_.pose.pose.orientation;
  out << "{\"have\":true,\"stale\":" << ((age > stale_after_s_) ? "true" : "false")
      << ",\"age_s\":" << json::number(age)
      << ",\"frame\":" << json::quote(last_odom_.header.frame_id)
      << ",\"position\":[" << json::number(p.x) << "," << json::number(p.y) << "," << json::number(p.z) << "]"
      << ",\"orientation\":[" << json::number(q.x) << "," << json::number(q.y) << "," << json::number(q.z)
      << "," << json::number(q.w) << "]"
      << ",\"trail\":[";
  for (std::size_t i = 0; i < trail_.size(); ++i) {
    if (i != 0) {out << ",";}
    out << json::number(trail_[i][0]) << "," << json::number(trail_[i][1]) << "," << json::number(trail_[i][2]);
  }
  out << "]}";
  return out.str();
}

std::string DashboardNode::run_action(const std::string & name)
{
  const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(action_timeout_s_));

  // **Called on the web server's thread, which is what makes waiting here safe.**
  // A service call that blocks inside a node's own executor callback deadlocks:
  // the reply can only be delivered by the executor, and the executor is sitting
  // in the callback waiting for it. From another thread the executor is free to
  // run, so the future completes.
  if (name == "save_mesh") {
    if (!save_client_->wait_for_service(std::chrono::seconds(2))) {
      return std::string("no /world/save_mesh (ready=") +
             (save_client_->service_is_ready() ? "1" : "0") +
             ") — is mesh_node running?\n";
    }
    auto request = std::make_shared<pimesh_msgs::srv::SaveMesh::Request>();
    char path[256];
    std::snprintf(
      path, sizeof(path), "%s/pimesh-%ld.ply", save_dir_.c_str(),
      static_cast<long>(std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count()));
    request->path = path;
    auto future = save_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return "save timed out\n";
    }
    const auto response = future.get();
    RCLCPP_INFO(get_logger(), "dashboard: save_mesh -> %s", request->path.c_str());
    return response->success ?
           ("saved " + request->path + "\n") : ("save failed: " + response->message + "\n");
  }

  if (name == "reset_map") {
    if (!reset_client_->wait_for_service(std::chrono::seconds(2))) {
      return std::string("no /world/reset_map (ready=") +
             (reset_client_->service_is_ready() ? "1" : "0") +
             ") — is mesh_node running?\n";
    }
    auto request = std::make_shared<pimesh_msgs::srv::ResetMap::Request>();
    auto future = reset_client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      return "reset timed out\n";
    }
    const auto response = future.get();
    RCLCPP_WARN(get_logger(), "dashboard: reset_map requested from a browser");
    return response->success ? "map reset\n" : ("reset failed: " + response->message + "\n");
  }

  // Named actions only. A dashboard that forwarded an arbitrary string to
  // something would be a remote-control surface, which this is not.
  return "unknown action\n";
}

void DashboardNode::tick()
{
  // **Built and sent even with nobody connected**, which is deliberate: the cost
  // is a few kilobytes of string a second, and doing it only when a client is
  // attached would make this node behave differently under measurement than in
  // use. The gate for this phase compares a run with a browser against a run
  // without one; a node that skips work when unobserved would pass that
  // comparison for the wrong reason.
  server_->broadcast(Channel::Stats, stats_json());
  server_->broadcast(Channel::Pose, pose_json());
}

}  // namespace pimesh_dashboard

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_dashboard::DashboardNode)
