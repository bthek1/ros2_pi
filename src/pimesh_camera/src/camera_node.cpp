// Capture on the Pi. The interesting twenty lines in this file are stamp_for();
// everything else is plumbing around them.

#include "pimesh_camera/camera_node.hpp"

#include <time.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace pimesh_camera
{
namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & text)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = text;
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describe_range(
  const std::string & text, std::int64_t low, std::int64_t high)
{
  auto descriptor = describe(text);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = low;
  range.to_value = high;
  descriptor.integer_range.push_back(range);
  return descriptor;
}

std::int64_t monotonic_now_ns()
{
  struct timespec ts {};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

}  // namespace

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("camera_node", options)
{
  V4l2Capture::Config config;
  config.device = declare_parameter(
    "device", config.device,
    describe("V4L2 capture node. On the C922 this is /dev/video0; /dev/video1 is "
             "its UVC metadata node and is not a capture device."));
  config.width = static_cast<std::uint32_t>(
    declare_parameter("width", 1280, describe_range("Frame width in pixels.", 160, 4096)));
  config.height = static_cast<std::uint32_t>(
    declare_parameter("height", 720, describe_range("Frame height in pixels.", 90, 2160)));
  config.fps = static_cast<std::uint32_t>(
    declare_parameter(
      "fps", 60,
      describe_range(
        "Frame rate requested of the driver. A request, not a setting: the C922 "
        "agrees to 60 and delivers 18-21 with exposure_dynamic_framerate set. "
        "Measure the rate; never quote this number.", 1, 120)));
  config.buffer_count = static_cast<std::uint32_t>(
    declare_parameter(
      "buffer_count", 4,
      describe_range(
        "mmap buffers in the driver's pool. Deeper is not better on a live "
        "stream — it is latency, not headroom.", 2, 16)));

  // camera_optical_frame, not camera_link. The image plane is in the optical
  // convention (z forward, x right, y down) and every geometric consumer of
  // this topic works in it; pimesh_bringup publishes the static edge that
  // relates it to the body. Stamping frames with camera_link here would put
  // every unprojected ray 90 degrees out with nothing failing.
  frame_id_ = declare_parameter(
    "frame_id", std::string("camera_optical_frame"),
    describe("TF frame the image plane is in. The optical convention, always."));

  const int timeout_ms = static_cast<int>(
    declare_parameter(
      "grab_timeout_ms", 2000,
      describe_range(
        "How long a single frame may take before the device is declared dead.", 50, 10000)));

  camera_info_ = build_camera_info();

  // --- QoS ------------------------------------------------------------------
  //
  // RELIABLE, KEEP_LAST(1), and both halves are load-bearing.
  //
  // RELIABLE because BEST_EFFORT delivers *zero* large frames: a ~120 kB
  // message fragments past the socket buffer and the fragments never
  // reassemble. This was measured in the predecessor and it is not a tuning
  // preference — it is the difference between a topic and an empty topic.
  //
  // KEEP_LAST(1) because this is live video. A depth-10 queue on a 47 Hz stream
  // is not a buffer, it is a fifth of a second of stale frames that a slow
  // consumer will work through in order while the room moves on. The freshest
  // frame is the only one worth having; the rest are dropped on purpose and
  // that is the design, not a fault.
  rclcpp::QoS image_qos(rclcpp::KeepLast(1));
  image_qos.reliable();

  // CameraInfo is transient-local: it is a fact about the camera, not an event,
  // so a subscriber that joins late must not have to wait for the next one. It
  // is small enough that durability costs nothing.
  rclcpp::QoS info_qos(rclcpp::KeepLast(1));
  info_qos.reliable().transient_local();

  // Relative names, so the whole camera moves together under a namespace. The
  // "/compressed" suffix is image_transport's convention, which is what lets
  // RViz's Image display and the dashboard subscribe with transport
  // `compressed` and find this without a remap. We publish the CompressedImage
  // directly rather than through image_transport: this node must build on the
  // Pi under Jazzy with the smallest possible dependency set, and there is no
  // raw image here for image_transport to have an opinion about.
  image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("image_raw/compressed", image_qos);
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", info_qos);

  // Open the device *in the constructor*, so that a missing or busy camera is a
  // construction failure. That is what turns it into a non-zero exit in main()
  // and a failed component load in a container, instead of a node that comes up
  // and idles.
  capture_ = std::make_unique<V4l2Capture>(config);

  RCLCPP_INFO(
    get_logger(), "%s: %ux%u MJPEG, %u buffers, driver asked for %u fps",
    capture_->device().c_str(), capture_->width(), capture_->height(),
    config.buffer_count, capture_->fps());
  RCLCPP_INFO(
    get_logger(), "publishing %s and %s in frame '%s'",
    image_pub_->get_topic_name(), info_pub_->get_topic_name(), frame_id_.c_str());

  if (!get_parameter("calibrated").as_bool()) {
    // Loud, every launch, until somebody runs the checkerboard. A CameraInfo
    // with plausible-looking nominal intrinsics is more dangerous than an empty
    // one, because everything downstream will happily unproject with it and the
    // error shows up as a mesh that is subtly the wrong shape.
    RCLCPP_WARN(
      get_logger(),
      "camera_info carries NOMINAL intrinsics, not a calibration "
      "(fx=%.1f fy=%.1f cx=%.1f cy=%.1f, zero distortion). "
      "Depth unprojection with these is approximate. Run the checkerboard and set "
      "camera_matrix/distortion_coefficients, then set calibrated:=true.",
      camera_info_.k[0], camera_info_.k[4], camera_info_.k[2], camera_info_.k[5]);
  }

  // A dedicated thread, not a timer. The capture path blocks in poll() waiting
  // for the driver, which is exactly the thing that must never happen on an
  // executor thread — it would hold the callback group for up to a frame
  // interval and stall every other callback in the process. This is the same
  // rule as "no work in a subscription callback beyond a bounded copy", seen
  // from the producing end.
  worker_ = std::thread([this, timeout_ms]() {this->capture_loop(timeout_ms);});
}

CameraNode::~CameraNode()
{
  running_ = false;
  if (worker_.joinable()) {worker_.join();}
  // capture_ is destroyed after the thread has joined, never before: the worker
  // dereferences it on every iteration.
}

sensor_msgs::msg::CameraInfo CameraNode::build_camera_info()
{
  declare_parameter(
    "calibrated", false,
    describe(
      "True once camera_matrix and distortion_coefficients come from a real "
      "checkerboard run. False makes the node warn on every startup, which is "
      "the intended behaviour until somebody does the calibration."));

  // Nominal C922 720p intrinsics: fx ~ 907 is the predecessor's figure for this
  // camera at this resolution, and the principal point is assumed at the image
  // centre. Nine numbers, row-major, the standard K.
  const auto k = declare_parameter(
    "camera_matrix", std::vector<double>{907.0, 0.0, 640.0, 0.0, 907.0, 360.0, 0.0, 0.0, 1.0},
    describe("Row-major 3x3 intrinsic matrix K."));
  const auto d = declare_parameter(
    "distortion_coefficients", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0},
    describe("plumb_bob coefficients k1 k2 p1 p2 k3."));

  sensor_msgs::msg::CameraInfo info;
  info.width = static_cast<std::uint32_t>(get_parameter("width").as_int());
  info.height = static_cast<std::uint32_t>(get_parameter("height").as_int());
  info.distortion_model = "plumb_bob";
  info.d = d;
  for (std::size_t i = 0; i < std::min<std::size_t>(9, k.size()); ++i) {info.k[i] = k[i]; }
  // R is identity and P is K with a zero fourth column: a single, unrectified
  // camera. Filling them in matters — a consumer that reads P and finds zeros
  // gets a projection matrix that maps everything to the origin.
  info.r[0] = info.r[4] = info.r[8] = 1.0;
  info.p[0] = info.k[0]; info.p[2] = info.k[2];
  info.p[5] = info.k[4]; info.p[6] = info.k[5];
  info.p[10] = 1.0;
  return info;
}

rclcpp::Time CameraNode::stamp_for(const Frame & frame)
{
  // ============================================================
  // The reason this node exists instead of `usb_cam`.
  // ============================================================
  //
  // The frame was captured at `frame.monotonic_ns` on CLOCK_MONOTONIC. The
  // stamp has to be on the ROS clock, which is the system clock. Those are two
  // different epochs, and the conversion between them is where usb_cam 0.8.1
  // goes wrong: it computes the offset between the two clocks **once per
  // process** and adds it to every frame forever. The result is that every
  // stamp in a session is displaced by the same random sub-second amount —
  // measured at 0.223, 0.362 and 0.979 s on three launches of the same binary —
  // and the amount is redrawn each launch. That is what makes a stamp-age
  // freshness gate drop 100% of frames, and it is why nothing in this project
  // is allowed to gate on absolute stamp age.
  //
  // The fix is to stop converting epochs at all and convert an *interval*
  // instead. Read the monotonic clock now, subtract the frame's monotonic
  // timestamp to get the frame's age — a duration, which is the same number on
  // any clock — and subtract that age from the current ROS time:
  //
  //     stamp = ros_now - (monotonic_now - frame.monotonic)
  //
  // Both readings are taken within microseconds of each other on the same
  // machine, so the two clocks' relative drift over that gap is irrelevant.
  // There is no per-process constant to be wrong, which is exactly what the
  // gate checks: two launches of this node must produce the same
  // stamp-vs-receipt offset, because there is nothing in here that could differ
  // between them.
  const rclcpp::Time ros_now = now();

  if (!frame.monotonic_valid) {
    if (!warned_no_monotonic_) {
      warned_no_monotonic_ = true;
      RCLCPP_WARN(
        get_logger(),
        "the driver is not giving monotonic buffer timestamps — falling back to "
        "stamping at dequeue. Stamps are still honest to within a frame, but "
        "they no longer carry the kernel's capture time.");
    }
    return ros_now;
  }

  const std::int64_t age_ns = monotonic_now_ns() - frame.monotonic_ns;

  // A negative age means the buffer is stamped in the future, and an age of
  // seconds means it is not the clock we think it is. Either way the arithmetic
  // above would produce a confidently wrong stamp, which is worse than a
  // slightly late one. Half a second is far outside anything a 4-buffer pool at
  // 47 Hz can produce (~85 ms at the very worst).
  constexpr std::int64_t kMaxPlausibleAgeNs = 500000000LL;
  if (age_ns < 0 || age_ns > kMaxPlausibleAgeNs) {
    if (!warned_implausible_age_) {
      warned_implausible_age_ = true;
      RCLCPP_WARN(
        get_logger(),
        "buffer timestamp is %.1f ms old, which is not plausible — stamping at "
        "dequeue instead. The driver's clock is not what its flags claim.",
        static_cast<double>(age_ns) / 1e6);
    }
    return ros_now;
  }

  return rclcpp::Time(ros_now.nanoseconds() - age_ns, ros_now.get_clock_type());
}

void CameraNode::publish(const Frame & frame)
{
  const rclcpp::Time stamp = stamp_for(frame);

  // unique_ptr, moved into publish(). Over the LAN this changes nothing — it
  // serialises either way — but it is the shape every publisher in this project
  // uses, because the moment a topic is consumed inside a container the move is
  // what makes the zero-copy path eligible. Writing it the other way here would
  // be a habit that costs 2.7 MB a frame somewhere else later.
  auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id_;
  msg->format = "jpeg";
  // The one copy in the entire capture path: kernel buffer to message. The
  // JPEG is passed through byte for byte — no decode, no re-encode, no quality
  // setting. Whatever the camera's encoder produced is what the dev box
  // decodes.
  msg->data.assign(frame.data, frame.data + frame.size);

  image_pub_->publish(std::move(msg));

  // CameraInfo goes out with every frame and with the *same* stamp, so a
  // consumer pairing them by time gets an exact match rather than a
  // nearest-neighbour guess. Transient-local means a late subscriber also gets
  // the last one immediately.
  auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(camera_info_);
  info->header.stamp = stamp;
  info->header.frame_id = frame_id_;
  info_pub_->publish(std::move(info));

  // Sequence gaps are frames the kernel dropped before userspace ever saw them
  // — a different fault from frames lost on the wire, and only visible here.
  if (have_sequence_ && frame.sequence > last_sequence_ + 1) {
    kernel_drops_ += frame.sequence - last_sequence_ - 1;
  }
  last_sequence_ = frame.sequence;
  have_sequence_ = true;
  ++frames_;
}

void CameraNode::capture_loop(int timeout_ms)
{
  // The whole of the Pi's hot path. Block for a frame, stamp it, copy it into a
  // message, publish, give the buffer back. There is nothing to schedule and
  // nothing to rate-limit: the driver's cadence is the loop's cadence, and the
  // node publishes every frame the camera produces.
  try {
    Frame frame;
    // A run of empty polls, not a single one. One timeout is a camera that went
    // quiet for a beat — which genuinely happens, most often right after an
    // exposure change. Several in a row at a two-second budget each is a device
    // that has stopped, and treating that as normal is how a node ends up
    // publishing nothing while looking fine.
    int quiet = 0;
    constexpr int kMaxQuiet = 3;

    while (running_.load() && rclcpp::ok()) {
      if (!capture_->grab(frame, timeout_ms)) {
        if (++quiet >= kMaxQuiet) {
          fail(
            "no frame in " + std::to_string(kMaxQuiet) + " x " + std::to_string(timeout_ms) +
            " ms — the device has stopped delivering. Check that nothing else opened it, "
            "and that the exposure controls are sane (bash tools/camera-reset.sh).");
          return;
        }
        RCLCPP_WARN(get_logger(), "no frame in %d ms (%d/%d)", timeout_ms, quiet, kMaxQuiet);
        continue;
      }
      quiet = 0;
      publish(frame);
    }
  } catch (const CaptureError & e) {
    // The device went away mid-stream: unplugged, or its USB link reset. There
    // is no recovering from it inside this process, and pretending otherwise
    // would mean an idle node again.
    fail(std::string("capture failed after ") + std::to_string(frames_) + " frames: " + e.what());
    return;
  }

  RCLCPP_INFO(
    get_logger(), "captured %lu frames, %lu dropped inside the kernel before dequeue",
    static_cast<unsigned long>(frames_), static_cast<unsigned long>(kernel_drops_));
}

void CameraNode::fail(const std::string & why)
{
  RCLCPP_FATAL(get_logger(), "%s", why.c_str());
  exit_code_ = 1;
  // Ends spin() in main, which returns exit_code(). A node that cannot capture
  // must not remain a node.
  rclcpp::shutdown();
}

}  // namespace pimesh_camera

// Registered so a container can construct this by name. The Pi runs it
// standalone today; registering costs nothing and keeps the rule ("a node that
// only works standalone is a bug") true for this node as well.
RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_camera::CameraNode)
