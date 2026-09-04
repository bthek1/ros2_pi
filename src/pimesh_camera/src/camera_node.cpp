#include "pimesh_camera/camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace pimesh_camera
{

namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  d.read_only = true;   // every one of these is read at startup only
  return d;
}

rcl_interfaces::msg::ParameterDescriptor describe_range(
  const std::string & description, int64_t from, int64_t to)
{
  auto d = describe(description);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = 1;
  d.integer_range.push_back(range);
  return d;
}

}  // namespace

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("camera_node", options)
{
  // Parameters are declared with a description and, where they have one, a
  // validated range. A range here is a startup error instead of a puzzling
  // runtime failure three stages downstream.
  const auto device = declare_parameter<std::string>(
    "device", "/dev/video0",
    describe(
      "V4L2 capture device. Prefer the serial-keyed /dev/v4l/by-id/... path: it "
      "survives replugs, and on the C922 index1 is the UVC metadata node, which "
      "produces no frames."));
  const auto width = declare_parameter<int>(
    "image_width", 1280, describe_range("capture width in pixels", 160, 4096));
  const auto height = declare_parameter<int>(
    "image_height", 720, describe_range("capture height in pixels", 120, 2160));
  const auto fps = declare_parameter<int>(
    "framerate", 60,
    describe_range(
      "frames per second requested from the driver. Ask for the camera's real "
      "maximum: the node publishes frames as they arrive, so a lower request "
      "only throttles the sensor.", 1, 240));
  const auto buffer_count = declare_parameter<int>(
    "buffer_count", 4,
    describe_range(
      "mmap buffers in the driver pool. Enough to ride out a scheduling hiccup "
      "without adding latency the pipeline then has to carry.", 2, 16));
  frame_id_ = declare_parameter<std::string>(
    "frame_id", "camera_optical_frame",
    describe(
      "frame_id stamped on every message. Depth, keypoints and the TSDF all do "
      "their geometry in this frame."));
  const auto camera_info_url = declare_parameter<std::string>(
    "camera_info_url", "",
    describe(
      "calibration YAML (file:// or package://). Empty publishes an "
      "uncalibrated CameraInfo and warns — zero distortion is not good enough "
      "for a pipeline that unprojects every pixel."));
  poll_timeout_ms_ = declare_parameter<int>(
    "poll_timeout_ms", 2000,
    describe_range(
      "how long to wait for a frame before treating the camera as stalled",
      100, 30000));
  stats_period_s_ = declare_parameter<double>(
    "stats_period_s", 1.0, describe("how often to publish /pipeline/stats"));

  // RELIABLE + KEEP_LAST(1): BEST_EFFORT delivers *zero* frames once a message
  // fragments past the socket buffer, and a depth of 1 means a slow consumer
  // gets the freshest frame rather than a backlog of stale ones.
  const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  // CameraInfo is latched: it changes almost never, and a subscriber that
  // joins late still needs the intrinsics.
  const auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    "image_raw/compressed", image_qos);
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", info_qos);
  stats_pub_ = create_publisher<pimesh_msgs::msg::PipelineStats>(
    "/pipeline/stats", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  // There is no calibration yet, and this node does not pretend otherwise.
  //
  // K is published as ALL ZEROS rather than as a plausible-looking guess. A
  // fabricated focal length would let every downstream stage compute confident
  // nonsense; a zero K makes an uncalibrated camera detectable in one branch
  // (`info.k[0] == 0.0`). Loading a real calibration file is deferred until
  // there is one to load — see docs/plans/future/bootstrap-future.md.
  if (!camera_info_url.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "camera_info_url='%s' was given, but this node cannot load calibration "
      "files yet — publishing UNCALIBRATED CameraInfo. See the calibration "
      "entry in docs/plans/future/bootstrap-future.md.",
      camera_info_url.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "publishing UNCALIBRATED CameraInfo (K is all zeros). Run the "
      "checkerboard before trusting any 3D output.");
  }
  camera_info_ = sensor_msgs::msg::CameraInfo();
  camera_info_.distortion_model = "plumb_bob";
  camera_info_.d.assign(5, 0.0);

  // Throws on a missing, busy or wrong-kind device. Failing in the constructor
  // is deliberate: the process exits non-zero instead of sitting there
  // publishing nothing, which is what the driver this replaces does.
  capture_ = std::make_unique<V4l2Capture>(
    device, static_cast<uint32_t>(width), static_cast<uint32_t>(height),
    static_cast<uint32_t>(fps), static_cast<uint32_t>(buffer_count));
  capture_->start();

  RCLCPP_INFO(
    get_logger(), "%s: %ux%u MJPEG, %.0f fps requested, %u buffers",
    capture_->device().c_str(), capture_->width(), capture_->height(),
    capture_->actual_fps(), capture_->buffer_count());

  capture_thread_ = std::thread(&CameraNode::capture_loop, this);
}

CameraNode::~CameraNode()
{
  running_.store(false);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

void CameraNode::capture_loop()
{
  using clock = std::chrono::steady_clock;
  auto window_start = clock::now();
  uint64_t window_frames = 0;
  double window_latency_ms = 0.0;
  bool warned_about_stamps = false;

  try {
    while (running_.load() && rclcpp::ok()) {
      Frame frame;
      if (!capture_->wait_frame(frame, poll_timeout_ms_)) {
        RCLCPP_WARN(
          get_logger(), "no frame for %d ms — camera stalled?", poll_timeout_ms_);
        continue;
      }

      const auto t_dequeued = clock::now();
      const rclcpp::Time stamp(frame.stamp_ns, RCL_ROS_TIME);

      // Say it once, loudly: if the driver is not giving capture times, every
      // downstream latency number is fiction.
      if (!warned_about_stamps && capture_->timestamp_source() != TimestampSource::Monotonic) {
        RCLCPP_WARN(
          get_logger(),
          "driver timestamp source is '%s', not CLOCK_MONOTONIC — stamps are "
          "arrival times, not capture times",
          to_string(capture_->timestamp_source()));
        warned_about_stamps = true;
      }

      // The driver's own sequence counter is the only honest way to see frames
      // the kernel dropped before we ever saw them.
      if (have_sequence_ && frame.sequence > last_sequence_ + 1) {
        dropped_ += frame.sequence - last_sequence_ - 1;
      }
      last_sequence_ = frame.sequence;
      have_sequence_ = true;

      auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
      msg->header.stamp = stamp;
      msg->header.frame_id = frame_id_;
      msg->format = "jpeg";
      // The one copy in this node, and it is unavoidable: the message must own
      // its bytes, and the mmap'd buffer goes back to the driver immediately
      // after. ~150 kB memcpy against a 16 ms frame budget.
      msg->data.resize(frame.size);
      std::memcpy(msg->data.data(), frame.data, frame.size);

      capture_->requeue(frame);

      auto info = camera_info_;
      info.header.stamp = stamp;
      info.header.frame_id = frame_id_;
      info.width = capture_->width();
      info.height = capture_->height();

      image_pub_->publish(std::move(msg));
      info_pub_->publish(info);

      ++frames_;
      ++window_frames;
      window_latency_ms +=
        std::chrono::duration<double, std::milli>(clock::now() - t_dequeued).count();

      const double elapsed = std::chrono::duration<double>(clock::now() - window_start).count();
      if (elapsed >= stats_period_s_) {
        publish_stats(
          static_cast<double>(window_frames) / elapsed,
          window_frames > 0 ? window_latency_ms / static_cast<double>(window_frames) : 0.0);
        window_start = clock::now();
        window_frames = 0;
        window_latency_ms = 0.0;
      }
    }
  } catch (const std::exception & e) {
    // Losing the camera mid-session is fatal, and it must LOOK fatal.
    RCLCPP_FATAL(get_logger(), "capture failed: %s", e.what());
    failed_.store(true);
    running_.store(false);
    rclcpp::shutdown();
  }
}

void CameraNode::publish_stats(double rate_hz, double latency_ms)
{
  pimesh_msgs::msg::PipelineStats stats;
  stats.header.stamp = now();
  stats.header.frame_id = frame_id_;
  stats.stage = "capture";
  stats.rate_hz = static_cast<float>(rate_hz);
  stats.latency_ms = static_cast<float>(latency_ms);
  stats.latency_p95_ms = 0.0f;   // one window, no distribution kept — see P8
  stats.processed = frames_;
  stats.dropped_mailbox = 0;     // nothing is dropped by design here
  stats.dropped_transport = dropped_;  // kernel gaps in the driver's sequence
  stats.stale = false;
  stats.detail = std::string("v4l2 mjpeg ") + to_string(capture_->timestamp_source());
  stats_pub_->publish(stats);
}

}  // namespace pimesh_camera

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_camera::CameraNode)
