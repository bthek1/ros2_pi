// The integrator. A pair, a pose, an alignment and an integrate, on a worker
// thread behind a one-slot mailbox — the shape every expensive stage in this
// workspace has.

#include "pimesh_mapping/fusion_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/imgproc.hpp"
#include "pimesh_core/image_buffer.hpp"
#include "pimesh_core/stats.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

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

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<std::int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

}  // namespace

// The 32FC1 header-over-a-message lives in pimesh_core/image_buffer.hpp
// beside the bgr8 one it is a copy of. It was in the anonymous namespace above
// until 2026-09-19, where no test could reach it — which is how four copies of
// `percentile` and a wrong FNV-1a basis got as far as they did.
using pimesh_core::depth_mat_over;

FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: Node("fusion_node", options),
  last_log_(0, 0, RCL_ROS_TIME)
{
  const std::string depth_topic = declare_parameter(
    "depth_topic", std::string("/depth"),
    describe("32FC1 metres from depth_node, in-process."));
  const std::string rgb_topic = declare_parameter(
    "rgb_topic", std::string("/depth/rgb"),
    describe(
      "The exact frame each depth map was inferred on. Paired by stamp with no "
      "tolerance: a tolerance would colour a wall with a different instant."));
  const std::string info_topic = declare_parameter(
    "info_topic", std::string("/camera_info"),
    describe("The intrinsics the depth map is unprojected with."));

  world_frame_ = declare_parameter(
    "world_frame", std::string("map"),
    describe(
      "The frame the volume lives in. map, not odom: a pose-graph backend "
      "correcting map -> odom must move the frames, not the surface."));
  optical_frame_ = declare_parameter(
    "optical_frame", std::string("camera_optical_frame"),
    describe("Named, never re-derived. pimesh_bringup publishes and unit-tests this edge."));

  const std::string volume_key = declare_parameter(
    "volume_key", std::string("world"),
    describe(
      "Which process-local volume this node fills. mesh_node reads the same key; "
      "a mismatch is two volumes and an empty mesh. See shared_volume.hpp."));

  TsdfVolume::Options volume;
  volume.voxel_size_m = static_cast<float>(declare_parameter(
      "voxel_size_m", 0.015,
      describe_double("Voxel edge in metres. 15 mm, from P5.", 0.003, 0.2)));
  volume.truncation_voxels = static_cast<int>(declare_parameter(
      "truncation_voxels", 4,
      describe_int(
        "Half-width of the signed-distance band, in voxels. Wider than the "
        "depth noise or observations never meet; narrower than a thin object.",
        1, 16)));
  volume.min_weight = static_cast<float>(declare_parameter(
      "min_weight", 3.0,
      describe_double(
        "Observations a voxel needs before it is meshed or ray-cast. The noise "
        "floor: the gap to voxels_allocated is this doing its job.", 1.0, 64.0)));
  volume.max_weight = static_cast<float>(declare_parameter(
      "max_weight", 64.0,
      describe_double(
        "Where the running average saturates, so the map can still change.",
        1.0, 10000.0)));
  volume.max_range_m = static_cast<float>(declare_parameter(
      "max_range_m", 6.0,
      describe_double(
        "Must match depth_node's clip. Beyond it a reading is the model's "
        "'far away or no idea' and integrating it builds a shell of the clip plane.",
        0.5, 100.0)));
  volume.min_range_m = static_cast<float>(declare_parameter(
      "min_range_m", 0.15,
      describe_double("Below this a reading is the model's near failure.", 0.01, 5.0)));
  volume.max_blocks = static_cast<std::size_t>(declare_parameter(
      "max_blocks", 300000,
      describe_int(
        "Hard ceiling on the map, in blocks of 6 kB. Past it the mapped parts "
        "keep updating and nothing new is taken on, and the node says so. Not a "
        "fix for anything — see tsdf_volume.hpp on why desk1 reaches it.",
        1000, 5000000)));
  volume.allocation_stride = static_cast<int>(declare_parameter(
      "allocation_stride", 8,
      describe_int(
        "Pixel stride of the block-allocation pass. The update pass that follows "
        "reads every pixel; this only decides which blocks exist.", 1, 32)));

  ScaleAligner::Options align;
  align_ = declare_parameter(
    "align", true,
    describe(
      "Per-frame depth scale alignment. false is the control run in "
      "tools/gates/fusion.sh — a knob whose off position nobody has measured."));
  align.min_overlap = declare_parameter(
    "min_overlap", 0.2,
    describe_double(
      "Below this valid-overlap fraction a frame is mostly new geometry and "
      "there is nothing to conform to. Refuse rather than guess.", 0.0, 1.0));
  align.max_correction = declare_parameter(
    "max_correction", 0.15,
    describe_double(
      "The clamp on one frame's correction. A failed depth map can produce a "
      "ratio of 2 with a healthy-looking overlap.", 0.0, 1.0));
  align.window = static_cast<std::size_t>(declare_parameter(
      "align_window", 50,
      describe_int(
        "Frames in the rolling-median baseline. The high-pass's time constant: "
        "wobble is corrected, renderer bias and drift are absorbed.", 1, 1000)));
  aligner_ = ScaleAligner(align);

  raycast_stride_ = static_cast<int>(declare_parameter(
      "raycast_stride", 16,
      describe_int(
        "Pixel stride of the alignment ray-cast. 16 gives 80x45 rays, which is "
        "thousands of samples for a median and a few milliseconds of marching.",
        1, 64)));
  tf_timeout_s_ = declare_parameter(
    "tf_timeout_ms", 50.0,
    describe_double(
      "How long to wait for the pose at a frame's stamp. keypoint_node publishes "
      "it ~50 ms before depth finishes, so this is slack and not a design rate.",
      0.0, 2000.0)) / 1000.0;

  agree_tolerance_ = declare_parameter(
    "agree_tolerance", 0.05,
    describe_double(
      "How close, as a fraction of the distance, the map has to be to an "
      "incoming reading to count as agreeing with it. Relative rather than "
      "absolute because depth_scale is unpinned, so centimetres are unknown "
      "units. gates/fusion.sh compares this figure between its two runs.",
      0.001, 1.0));

  const double stats_period_s = declare_parameter(
    "stats_period_s", 5.0,
    describe_double("How often to log and publish the summary.", 0.5, 120.0));

  volume_ = VolumeRegistry::get(volume_key);
  volume_->configure(volume);

  // --- QoS ------------------------------------------------------------------
  //
  // RELIABLE + KEEP_LAST(1) on the images, as everywhere in this project: a 3.7 MB
  // depth map fragments past the socket buffer under BEST_EFFORT and never
  // reassembles. Transient-local on CameraInfo so a node started after the camera
  // still gets the intrinsics rather than waiting forever on a publisher that has
  // already said everything it is going to say.
  rclcpp::QoS image_qos(rclcpp::KeepLast(1));
  image_qos.reliable();
  rclcpp::QoS info_qos(rclcpp::KeepLast(1));
  info_qos.reliable().transient_local();

  // ConstSharedPtr on both, and it is the measured choice rather than taste.
  // /depth has this node and depth_probe on it during a gate run; rclcpp moves the
  // buffer into the last ownership-taking subscription and copies it for every
  // other, while shared const pointers all get the same buffer. A unique_ptr here
  // would add a 3.7 MB copy per frame that no log mentions.
  //
  // **Colour first, and the order of these two lines is load-bearing.** A
  // single-threaded executor collects everything that became ready in one wait
  // cycle and runs the callbacks in the order the subscriptions were registered,
  // not the order the messages were published. `depth_node` publishes the colour
  // twin and then the depth map, so both are ready in the same cycle — and with
  // the depth subscription registered first, the depth callback ran before the
  // colour it was looking for had arrived. Measured before this swap: 67 of ~750
  // frames integrated colourless on bags/desk1, about 12%, with nothing whatever
  // wrong upstream. `process()` re-checks as well, because an ordering that holds
  // for this executor is not a thing to depend on twice.
  rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
    rgb_topic, image_qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_rgb(std::move(msg));});
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic, image_qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_depth(std::move(msg));});
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    info_topic, info_qos,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {on_camera_info(std::move(msg));});

  // The contract the dashboard reads in P8: every stage says what it measured
  // about itself and the dashboard computes nothing. Publishing it now rather
  // than when there is something to read it means the message shape is exercised
  // by a gate before a browser depends on it.
  stats_pub_ = create_publisher<pimesh_msgs::msg::PipelineStats>("/pipeline/stats", 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

  worker_ = std::thread([this] {work();});

  last_log_ = now();
  stats_timer_ = create_wall_timer(
    std::chrono::duration<double>(stats_period_s), [this] {log_stats();});

  RCLCPP_INFO(
    get_logger(),
    "fusion_node up: %s + %s -> volume '%s' in %s, %.0f mm voxels, truncation %d, "
    "weight >= %.0f, align %s",
    depth_topic.c_str(), rgb_topic.c_str(), volume_key.c_str(), world_frame_.c_str(),
    static_cast<double>(volume.voxel_size_m) * 1000.0, volume.truncation_voxels,
    static_cast<double>(volume.min_weight), align_ ? "on" : "OFF (control)");
}

FusionNode::~FusionNode()
{
  mailbox_.stop();
  if (worker_.joinable()) {worker_.join();}
}

void FusionNode::on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(k_mutex_);
  const bool first = !have_k_;
  k_ = cv::Matx33d(
    msg->k[0], msg->k[1], msg->k[2],
    msg->k[3], msg->k[4], msg->k[5],
    msg->k[6], msg->k[7], msg->k[8]);
  k_width_ = static_cast<int>(msg->width);
  k_height_ = static_cast<int>(msg->height);
  have_k_ = (k_(0, 0) != 0.0 && k_(1, 1) != 0.0);
  if (first && have_k_) {
    RCLCPP_INFO(
      get_logger(), "intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f at %dx%d",
      k_(0, 0), k_(1, 1), k_(0, 2), k_(1, 2), k_width_, k_height_);
  }
}

void FusionNode::on_rgb(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(rgb_mutex_);
  recent_rgb_.push_back(std::move(msg));
  while (recent_rgb_.size() > rgb_history_) {recent_rgb_.pop_front();}
}

void FusionNode::on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  depth_in_.fetch_add(1, std::memory_order_relaxed);

  // The whole callback: find the colour twin and put a pointer in a slot.
  // `depth_node` publishes the colour first and the depth immediately after, so
  // this almost always hits the back of the deque.
  auto frame = std::make_shared<Frame>();
  frame->arrived = std::chrono::steady_clock::now();
  frame->rgb = colour_for(msg->header.stamp);
  frame->depth = std::move(msg);
  mailbox_.push(std::move(frame));
}

sensor_msgs::msg::Image::ConstSharedPtr FusionNode::colour_for(
  const builtin_interfaces::msg::Time & stamp)
{
  const std::int64_t want = stamp_ns(stamp);
  std::lock_guard<std::mutex> lock(rgb_mutex_);
  // Newest first: `depth_node` publishes the twin immediately before the depth
  // map, so in normal operation this is the very first comparison.
  for (auto it = recent_rgb_.rbegin(); it != recent_rgb_.rend(); ++it) {
    if (stamp_ns((*it)->header.stamp) == want) {return *it;}
  }
  return nullptr;
}

void FusionNode::work()
{
  while (!mailbox_.stopped()) {
    auto frame = mailbox_.pop(std::chrono::milliseconds(100));
    if (frame) {process(*frame);}
  }
}

bool FusionNode::pose_at(
  const builtin_interfaces::msg::Time & stamp, cv::Affine3d & world_from_camera)
{
  try {
    // **At the frame's own stamp, not "the latest".** The camera is being swept by
    // hand, so 50 ms of pose error is several degrees, and a TSDF bakes the pose
    // it was given into every voxel it touches — there is no undoing it later.
    // This is a lookup *within one machine's view of the tree*: keypoint_node
    // stamps the pose with the same capture stamp this depth map carries, so the
    // two are on the same clock and the cross-machine NTP question never arises.
    const tf2::TimePoint when{std::chrono::nanoseconds(stamp_ns(stamp))};
    const auto tf = tf_buffer_->lookupTransform(
      world_frame_, optical_frame_, when, tf2::durationFromSec(tf_timeout_s_));
    const auto & q = tf.transform.rotation;
    const auto & t = tf.transform.translation;
    tf2::Matrix3x3 m(tf2::Quaternion(q.x, q.y, q.z, q.w));
    world_from_camera = cv::Affine3d(
      cv::Matx33d(
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]),
      cv::Vec3d(t.x, t.y, t.z));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "no %s <- %s at the frame's stamp (%s) — dropping the frame rather than "
      "integrating it at a guessed pose",
      world_frame_.c_str(), optical_frame_.c_str(), ex.what());
    return false;
  }
}

void FusionNode::process(Frame & frame)
{
  const auto started = std::chrono::steady_clock::now();
  const double lag_ms =
    std::chrono::duration<double, std::milli>(started - frame.arrived).count();

  // The second look. The subscription callback already tried; this runs on the
  // worker a moment later, by which time an out-of-order colour callback has
  // certainly happened. **Integrate anyway if it is still missing, colourless,
  // and count it** — a depth map with no twin is still geometry, and refusing it
  // would throw away a wall to avoid a grey patch. The counter is what makes it
  // visible: a run where this is common means the two topics have come apart,
  // which would otherwise show only as a mesh slowly losing its colour.
  if (!frame.rgb) {
    frame.rgb = colour_for(frame.depth->header.stamp);
    if (!frame.rgb) {unpaired_.fetch_add(1, std::memory_order_relaxed);}
  }

  const cv::Mat depth = depth_mat_over(*frame.depth);
  if (depth.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "depth frame with encoding '%s' %ux%u step %u is not usable 32FC1",
      frame.depth->encoding.c_str(), frame.depth->width, frame.depth->height,
      frame.depth->step);
    return;
  }

  cv::Matx33d k;
  {
    std::lock_guard<std::mutex> lock(k_mutex_);
    if (!have_k_) {
      no_k_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no %s yet — a volume built from invented intrinsics is confidently the "
        "wrong shape rather than absent", "/camera_info");
      return;
    }
    k = k_;
  }

  // **The intrinsics must be for this depth map's resolution.** They are silently
  // compatible otherwise — the numbers are all finite and the room comes out a
  // plausible shape at the wrong size, which is indistinguishable from the depth
  // scale being wrong and would send somebody to the tape measure.
  if (k_width_ != depth.cols || k_height_ != depth.rows) {
    const double sx = static_cast<double>(depth.cols) / static_cast<double>(k_width_);
    const double sy = static_cast<double>(depth.rows) / static_cast<double>(k_height_);
    k = cv::Matx33d(
      k(0, 0) * sx, 0.0, k(0, 2) * sx,
      0.0, k(1, 1) * sy, k(1, 2) * sy,
      0.0, 0.0, 1.0);
  }

  cv::Affine3d world_from_camera;
  if (!pose_at(frame.depth->header.stamp, world_from_camera)) {
    no_pose_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  // --- Alignment ------------------------------------------------------------
  //
  // The ray-cast runs at a fraction of the frame's resolution and the incoming
  // depth is downsampled to match. **INTER_NEAREST, not INTER_AREA**: averaging
  // across a depth discontinuity invents a distance halfway between a desk edge
  // and the wall behind it, and the ratio this feeds is supposed to be a
  // statistic over real measurements.
  const auto align_started = std::chrono::steady_clock::now();
  ScaleResult alignment;
  double raw_gap = 0.0;
  double raw_overlap = 0.0;
  double raw_agree = 0.0;
  bool have_gap = false;
  bool have_agree = false;

  const cv::Size small(
    std::max(1, depth.cols / raycast_stride_), std::max(1, depth.rows / raycast_stride_));
  const double sx = static_cast<double>(small.width) / static_cast<double>(depth.cols);
  const double sy = static_cast<double>(small.height) / static_cast<double>(depth.rows);
  const cv::Matx33d k_small(
    k(0, 0) * sx, 0.0, k(0, 2) * sx,
    0.0, k(1, 1) * sy, k(1, 2) * sy,
    0.0, 0.0, 1.0);

  cv::resize(depth, small_depth_, small, 0, 0, cv::INTER_NEAREST);
  volume_->with_volume(
    [&](const TsdfVolume & v) {v.raycast(k_small, world_from_camera, small, expected_);});

  // Measured on every run, aligned or not: this is the paired-surface number
  // tools/gates/fusion.sh compares between the two. "How far is the next
  // observation from the surface already built" is the same question the
  // predecessor asked of two views of a wall, asked once per frame instead.
  have_gap = surface_gap(expected_, small_depth_, aligner_.options().min_overlap,
      raw_gap, raw_overlap);
  have_agree = surface_agreement(expected_, small_depth_, agree_tolerance_, raw_agree);

  if (align_) {
    alignment = aligner_.scale_for(expected_, small_depth_);
    if (alignment.aligned) {
      aligned_.fetch_add(1, std::memory_order_relaxed);
      if (alignment.clamped) {clamped_.fetch_add(1, std::memory_order_relaxed);}
    }
  }
  const double align_ms = ms_since(align_started);

  // --- Integrate -------------------------------------------------------------
  const cv::Mat * to_integrate = &depth;
  if (align_ && alignment.aligned && alignment.scale != 1.0) {
    depth.convertTo(scaled_depth_, CV_32FC1, alignment.scale);
    to_integrate = &scaled_depth_;
    // The gap the aligner actually achieved, measured after the correction, so
    // the two runs of the gate are each reporting the disagreement of the frames
    // they really integrated.
    cv::Mat small_scaled;
    small_depth_.convertTo(small_scaled, CV_32FC1, alignment.scale);
    double gap = 0.0;
    double overlap = 0.0;
    if (surface_gap(expected_, small_scaled, aligner_.options().min_overlap, gap, overlap)) {
      raw_gap = gap;
      raw_overlap = overlap;
      have_gap = true;
    }
    double agree = 0.0;
    if (surface_agreement(expected_, small_scaled, agree_tolerance_, agree)) {
      raw_agree = agree;
      have_agree = true;
    }
  }

  const cv::Mat colour = frame.rgb ? pimesh_core::mat_over(*frame.rgb) : cv::Mat();

  const auto integrate_started = std::chrono::steady_clock::now();
  const auto result = volume_->with_volume(
    [&](TsdfVolume & v) {return v.integrate(*to_integrate, colour, k, world_from_camera);});
  const double integrate_ms = ms_since(integrate_started);

  if (result.blocks_refused > 0) {
    refused_.fetch_add(result.blocks_refused, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "the volume is full at %zu blocks — mapped space keeps updating, new space "
      "is being turned away (%zu blocks this frame). Raise max_blocks, or read "
      "tsdf_volume.hpp on why a sweep lays down thirty layers of the same wall",
      volume_->with_volume([](const TsdfVolume & v) {return v.block_count();}),
      result.blocks_refused);
  }

  volume_->note_integrated();
  integrated_.fetch_add(1, std::memory_order_relaxed);
  const double total_ms = ms_since(started);

  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    const auto finished = std::chrono::steady_clock::now();
    if (have_last_integration_) {
      interval_ms_.push_back(
        std::chrono::duration<double, std::milli>(finished - last_integration_).count());
    }
    last_integration_ = finished;
    have_last_integration_ = true;
    integrate_ms_.push_back(integrate_ms);
    align_ms_.push_back(align_ms);
    total_ms_.push_back(total_ms);
    lag_ms_.push_back(lag_ms);
    if (have_gap) {
      gap_m_.push_back(raw_gap);
      overlap_.push_back(raw_overlap);
    }
    if (have_agree) {agree_.push_back(raw_agree);}
    if (alignment.aligned) {scale_.push_back(alignment.scale);}
  }

  RCLCPP_DEBUG(
    get_logger(), "integrated blocks=%zu new=%zu voxels=%zu in %.2f ms",
    result.blocks_touched, result.blocks_new, result.voxels_updated, integrate_ms);
}

void FusionNode::log_stats()
{
  const auto now_ros = now();
  const double elapsed = (now_ros - last_log_).seconds();
  if (elapsed <= 0.0) {
    last_log_ = now_ros;
    return;
  }

  std::vector<double> integrate;
  std::vector<double> align;
  std::vector<double> total;
  std::vector<double> lag;
  std::vector<double> interval;
  std::vector<double> gap;
  std::vector<double> overlap;
  std::vector<double> scale;
  std::vector<double> agree;
  {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    integrate.swap(integrate_ms_);
    align.swap(align_ms_);
    total.swap(total_ms_);
    lag.swap(lag_ms_);
    interval.swap(interval_ms_);
    gap.swap(gap_m_);
    overlap.swap(overlap_);
    scale.swap(scale_);
    agree.swap(agree_);
  }

  auto mean_of = [](const std::vector<double> & v) {
      if (v.empty()) {return 0.0;}
      double sum = 0.0;
      for (const double x : v) {sum += x;}
      return sum / static_cast<double>(v.size());
    };

  const auto integrated = integrated_.load(std::memory_order_relaxed);
  const auto dropped = mailbox_.dropped();
  const auto window_out = integrated - last_integrated_;
  const double rate = static_cast<double>(window_out) / elapsed;

  const std::size_t blocks = volume_->with_volume(
    [](const TsdfVolume & v) {return v.block_count();});

  // **Node name in the line, and a `blocks=` field that cannot be confused with
  // another stage's.** `gates/keypoints.sh` read a per-frame cost with
  // `grep 'stats rate=' | tail -1` and started silently reading depth_node's idle
  // window the day depth_node joined the container — 0.00 ms asserted against an
  // 8 ms budget, printing PASS. Every gate here greps by node name for that
  // reason, and this line is written to be greppable that way.
  RCLCPP_INFO(
    get_logger(),
    "stats rate=%.1f integrate_mean=%.2f integrate_p95=%.2f align_mean=%.2f "
    "cost_mean=%.2f cost_p95=%.2f lag_mean=%.2f lag_p95=%.2f "
    "interval_p50=%.1f interval_max=%.1f gap_m=%.4f overlap=%.3f "
    "agree=%.4f "
    "scale_mean=%.4f blocks=%zu refused=%lu dropped=%zu unpaired=%lu no_pose=%lu "
    "aligned=%lu clamped=%lu",
    rate, mean_of(integrate), pimesh_core::percentile(integrate, 0.95),
    mean_of(align), mean_of(total), pimesh_core::percentile(total, 0.95),
    mean_of(lag), pimesh_core::percentile(lag, 0.95),
    pimesh_core::percentile(interval, 0.5),
    interval.empty() ? 0.0 : *std::max_element(interval.begin(), interval.end()),
    mean_of(gap), mean_of(overlap), mean_of(agree), mean_of(scale), blocks,
    static_cast<unsigned long>(refused_.load(std::memory_order_relaxed)),
    dropped - last_dropped_,
    static_cast<unsigned long>(unpaired_.load(std::memory_order_relaxed)),
    static_cast<unsigned long>(no_pose_.load(std::memory_order_relaxed)),
    static_cast<unsigned long>(aligned_.load(std::memory_order_relaxed)),
    static_cast<unsigned long>(clamped_.load(std::memory_order_relaxed)));

  auto msg = std::make_unique<pimesh_msgs::msg::PipelineStats>();
  msg->header.stamp = now_ros;
  msg->header.frame_id = world_frame_;
  msg->stage = "fusion";
  msg->rate_hz = static_cast<float>(rate);
  msg->latency_ms = static_cast<float>(mean_of(total));
  msg->latency_p95_ms = static_cast<float>(pimesh_core::percentile(total, 0.95));
  msg->frames_in = depth_in_.load(std::memory_order_relaxed);
  msg->frames_out = integrated;
  // The two kinds, kept apart as PipelineStats insists. A mailbox overwrite is
  // this node falling behind — which, unlike depth_node, is *not* the design here:
  // fusion is meant to keep up with the stage above it. A frame dropped for want
  // of a pose or of intrinsics is not a transport loss either, but it is the
  // closest honest home for it and the `detail` field says which.
  msg->dropped_by_design = dropped;
  msg->dropped_in_transport =
    no_pose_.load(std::memory_order_relaxed) + no_k_.load(std::memory_order_relaxed);
  char detail[256];
  std::snprintf(
    detail, sizeof(detail),
    "%s blocks=%zu agree=%.0f%% gap=%.1fcm no_pose=%lu unpaired=%lu",
    align_ ? "aligned" : "unaligned", blocks, mean_of(agree) * 100.0, mean_of(gap) * 100.0,
    static_cast<unsigned long>(no_pose_.load(std::memory_order_relaxed)),
    static_cast<unsigned long>(unpaired_.load(std::memory_order_relaxed)));
  msg->detail = detail;
  stats_pub_->publish(std::move(msg));

  last_log_ = now_ros;
  last_integrated_ = integrated;
  last_dropped_ = dropped;
}

}  // namespace pimesh_mapping

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_mapping::FusionNode)
