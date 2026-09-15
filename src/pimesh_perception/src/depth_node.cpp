// Inference on its own thread behind a one-slot mailbox. The interesting lines
// are the ConstSharedPtr subscription (see the header on why it is not a
// unique_ptr), the header copy that keeps /depth dated to when the light
// arrived, and the startup log that names the execution provider.

#include "pimesh_perception/depth_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/imgproc.hpp"
#include "pimesh_perception/depth_model.hpp"
#include "pimesh_perception/image_buffer.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace pimesh_perception
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
  const std::string & text, double low, double high)
{
  auto descriptor = describe(text);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = low;
  range.to_value = high;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

double ms_since(const std::chrono::steady_clock::time_point & start)
{
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

void add(std::atomic<double> & target, double value)
{
  double current = target.load(std::memory_order_relaxed);
  while (!target.compare_exchange_weak(current, current + value, std::memory_order_relaxed)) {}
}

void raise_to(std::atomic<double> & target, double value)
{
  double current = target.load(std::memory_order_relaxed);
  while (value > current &&
    !target.compare_exchange_weak(current, value, std::memory_order_relaxed)) {}
}

}  // namespace

DepthNode::DepthNode(const rclcpp::NodeOptions & options)
: Node("depth_node", options),
  last_log_(0, 0, RCL_ROS_TIME)
{
  const std::string input_topic = declare_parameter(
    "input_topic", std::string("/image_raw"),
    describe("Decoded bgr8 from decode_node, in-process."));
  const std::string depth_topic = declare_parameter(
    "depth_topic", std::string("/depth"),
    describe("32FC1 metres, carrying the input frame's stamp and camera_optical_frame."));
  const std::string rgb_topic = declare_parameter(
    "rgb_topic", std::string("/depth/rgb"),
    describe(
      "The exact frame each depth map was inferred on, republished unchanged and "
      "stamped identically, so an exact-sync consumer can pair the two. A "
      "separate republisher cannot do this: it would drop different frames."));

  // **The default is baked in at build time, not computed at runtime.** The
  // tempting version of this walks up from get_package_share_directory() to find
  // the workspace root, which is four `..` segments of assumption about a layout
  // that --symlink-install already bends. CMake knows the source directory for
  // certain, so it passes it as PIMESH_DEFAULT_MODEL_PATH and there is no path
  // arithmetic here at all. models/ is git-ignored and 99 MB, so it is read from
  // the source tree rather than installed into share/.
  std::string model_path = declare_parameter(
    "model_path", std::string(PIMESH_DEFAULT_MODEL_PATH),
    describe(
      "Absolute path to the ONNX model. The default is this workspace's "
      "models/depth_anything_v2_small.onnx, baked in at build time. "
      "Fetch it with bash tools/fetch-model.sh."));

  // **`use_cuda` is not a debugging convenience.** It is the control run
  // tools/gates/depth.sh needs: a per-frame budget that the CPU path has never
  // been shown to fail is a threshold nobody has watched exclude anything.
  const bool use_cuda = declare_parameter(
    "use_cuda", true,
    describe(
      "Request the CUDA execution provider. false forces CPU — the control run "
      "in tools/gates/depth.sh, not a fallback anyone should run for real."));

  // **Monocular depth is scale-ambiguous and this is the knob that fixes it.**
  // The model says "twice as far", never "three metres", so this constant is
  // arbitrary until a tape measure pins it — which is P5's job, and the
  // predecessor's came out at 2.69x. Do not tune it by eye against a mesh that
  // looks about right.
  depth_scale_ = static_cast<float>(declare_parameter(
      "depth_scale", 10.0,
      describe_range(
        "metres = depth_scale / model_output. Arbitrary until P5 measures one "
        "known distance; the room will be plausibly shaped and the wrong size.",
        0.001, 1000.0)));

  max_range_ = static_cast<float>(declare_parameter(
      "max_range_m", 6.0,
      describe_range(
        "Distances are clipped here, and the clip is applied to the model's "
        "inverse output *before* the reciprocal — 1/0 is inf, not a big number.",
        0.1, 100.0)));

  publish_rgb_ = declare_parameter(
    "publish_rgb", true,
    describe("Republish the frame each depth map was inferred on, for exact sync."));

  optical_frame_ = declare_parameter(
    "optical_frame", std::string("camera_optical_frame"),
    describe(
      "The frame id stamped on both outputs. A static edge published by "
      "pimesh_bringup and unit-tested to be the optical convention — this node "
      "names it and never re-derives the rotation."));

  const double stats_period_s = declare_parameter(
    "stats_period_s", 5.0,
    describe_range("How often to log the throughput summary.", 0.5, 120.0));

  // --- The engine, and the line that makes a silent CPU fallback visible -----
  std::string error;
  engine_ = make_depth_engine(model_path, use_cuda, error);
  if (!engine_) {
    RCLCPP_ERROR(get_logger(), "depth_node cannot start: %s", error.c_str());
    throw std::runtime_error(error);
  }

  const bool got_cuda = engine_->provider() == "CUDAExecutionProvider";
  RCLCPP_INFO(
    get_logger(), "inference provider: %s (model %s)",
    engine_->provider().c_str(), model_path.c_str());
  if (!engine_->diagnostic().empty()) {
    // WARN and not INFO when CUDA was asked for and not obtained: this is the
    // "the mesh got slow" failure, and it has exactly one line in which to be
    // noticed.
    if (use_cuda && !got_cuda) {
      RCLCPP_WARN(get_logger(), "%s", engine_->diagnostic().c_str());
      RCLCPP_WARN(
        get_logger(),
        "running on the CPU: expect ~213 ms/frame against ~51 ms on the GPU. "
        "See docs/info/troubleshooting.md — the usual cause is a build linked "
        "without -Wl,--disable-new-dtags.");
    } else {
      RCLCPP_INFO(get_logger(), "%s", engine_->diagnostic().c_str());
    }
  }

  input_.resize(kInputElements);
  output_.resize(static_cast<std::size_t>(kModelSize) * kModelSize);

  // --- Warm the session -----------------------------------------------------
  //
  // The first inference costs 300-860 ms: CUDA context creation, cuBLAS handle
  // setup and kernel autotuning. Paying that on the first real frame would put an
  // 800 ms outlier into the very measurement the gate reads, and would stall the
  // first frames of every session.
  {
    const auto warm_start = std::chrono::steady_clock::now();
    std::fill(input_.begin(), input_.end(), 0.0F);
    if (!engine_->infer(input_.data(), output_.data())) {
      RCLCPP_ERROR(
        get_logger(), "the warm-up inference failed: %s", engine_->diagnostic().c_str());
      throw std::runtime_error("warm-up inference failed");
    }
    RCLCPP_INFO(get_logger(), "session warm in %.0f ms", ms_since(warm_start));
  }

  // --- QoS ------------------------------------------------------------------
  //
  // RELIABLE + KEEP_LAST(1) throughout: freshest frame, no backlog. BEST_EFFORT
  // is not an option for the 3.7 MB depth publisher — megabyte-class messages
  // fragment past the socket buffer and never reassemble, which is the rule that
  // predates this project.
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable();

  depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic, qos);
  if (publish_rgb_) {
    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(rgb_topic, qos);
  }

  // **ConstSharedPtr, not unique_ptr, and the difference is 2.7 MB a frame.**
  // /image_raw has two consumers now — keypoint_node and this one. rclcpp moves
  // the buffer into the *last* ownership-taking subscription and copies it for
  // every other; a shared const pointer goes through the path that hands one
  // buffer to all of them. Measured: 0/574 frames at the published address with
  // two unique_ptr consumers, 504/504 with two ConstSharedPtr consumers.
  sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic, qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(std::move(msg));});

  stats_timer_ = create_wall_timer(
    std::chrono::duration<double>(stats_period_s), [this] {log_stats();});

  worker_ = std::thread([this] {work();});

  const std::string also = publish_rgb_ ? (" + " + rgb_topic) : std::string();
  RCLCPP_INFO(
    get_logger(), "depth_node up: %s -> %s%s, scale %.3f, clip %.1f m",
    input_topic.c_str(), depth_topic.c_str(), also.c_str(),
    static_cast<double>(depth_scale_), static_cast<double>(max_range_));
}

DepthNode::~DepthNode()
{
  mailbox_.stop();
  if (worker_.joinable()) {worker_.join();}
}

void DepthNode::on_image(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // The whole callback: a pointer into a slot. Anything more here holds an
  // executor thread while 51 ms of inference happens somewhere else.
  frames_in_.fetch_add(1, std::memory_order_relaxed);
  mailbox_.push(std::move(msg));
}

void DepthNode::work()
{
  while (!mailbox_.stopped()) {
    auto msg = mailbox_.pop(std::chrono::milliseconds(100));
    if (!msg) {continue;}
    process_frame(*msg);
  }
}

void DepthNode::process_frame(const sensor_msgs::msg::Image & msg)
{
  const auto started = std::chrono::steady_clock::now();

  // A cv::Mat header over the message's own pixels — no copy. Empty when the
  // encoding or the step arithmetic does not check out, which is a refusal
  // rather than a guess: a wrong step shears the image by a column per row and
  // looks like a camera fault.
  const cv::Mat bgr = mat_over(msg);
  if (bgr.empty()) {
    failures_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "frame with encoding '%s' %ux%u step %u is not usable bgr8",
      msg.encoding.c_str(), msg.width, msg.height, msg.step);
    return;
  }

  preprocess(bgr, input_.data());

  const auto infer_start = std::chrono::steady_clock::now();
  if (!engine_->infer(input_.data(), output_.data())) {
    failures_.fetch_add(1, std::memory_order_relaxed);
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "inference failed: %s",
      engine_->diagnostic().c_str());
    return;
  }
  const double infer_ms = ms_since(infer_start);

  // Relative inverse depth at 518x518 -> metres -> the frame's own resolution.
  //
  // The conversion happens *before* the resize, matching the predecessor so the
  // two are comparable. Both orders interpolate across depth discontinuities and
  // produce flying pixels at object edges; neither is free, and changing the
  // order would change every number without being obviously better.
  const cv::Mat relative(kModelSize, kModelSize, CV_32FC1, output_.data());
  to_metres(relative, depth_scale_, max_range_, metres_small_);
  cv::resize(metres_small_, metres_, bgr.size(), 0, 0, cv::INTER_LINEAR);

  // --- Publish, colour first ------------------------------------------------
  //
  // The colour twin goes out first so an exact-sync consumer can complete the
  // pair the moment /depth lands rather than holding a depth map waiting for it.
  if (rgb_pub_) {
    auto rgb_msg = std::make_unique<sensor_msgs::msg::Image>();
    fill_bgr8(*rgb_msg, bgr);
    rgb_msg->header.stamp = msg.header.stamp;
    rgb_msg->header.frame_id = optical_frame_;
    rgb_pub_->publish(std::move(rgb_msg));
  }

  auto depth_msg = std::make_unique<sensor_msgs::msg::Image>();
  depth_msg->header.stamp = msg.header.stamp;
  depth_msg->header.frame_id = optical_frame_;
  depth_msg->height = static_cast<std::uint32_t>(metres_.rows);
  depth_msg->width = static_cast<std::uint32_t>(metres_.cols);
  depth_msg->encoding = "32FC1";
  depth_msg->is_bigendian = 0;
  depth_msg->step = static_cast<std::uint32_t>(metres_.cols) * sizeof(float);
  depth_msg->data.resize(static_cast<std::size_t>(depth_msg->step) * depth_msg->height);
  for (int row = 0; row < metres_.rows; ++row) {
    std::memcpy(
      depth_msg->data.data() + static_cast<std::size_t>(row) * depth_msg->step,
      metres_.ptr(row), depth_msg->step);
  }
  depth_pub_->publish(std::move(depth_msg));

  const double total_ms = ms_since(started);
  frames_out_.fetch_add(1, std::memory_order_relaxed);
  add(infer_sum_ms_, infer_ms);
  add(total_sum_ms_, total_ms);
  raise_to(total_max_ms_, total_ms);
}

void DepthNode::log_stats()
{
  const auto now = this->now();
  if (last_log_.nanoseconds() == 0) {
    last_log_ = now;
    return;
  }
  const double elapsed = (now - last_log_).seconds();
  if (elapsed <= 0.0) {return;}

  const auto out = frames_out_.load(std::memory_order_relaxed);
  const auto dropped = mailbox_.dropped();
  const double infer_sum = infer_sum_ms_.load(std::memory_order_relaxed);
  const double total_sum = total_sum_ms_.load(std::memory_order_relaxed);

  const auto window_out = out - last_logged_out_;
  const double window_infer = infer_sum - last_infer_sum_ms_;
  const double window_total = total_sum - last_total_sum_ms_;

  // Guard the denominator: a window with no frames is a real condition (the bag
  // ended, the camera stopped) and must not print a division by zero as a
  // plausible-looking 0.00 ms.
  const double infer_mean = window_out ? window_infer / static_cast<double>(window_out) : 0.0;
  const double total_mean = window_out ? window_total / static_cast<double>(window_out) : 0.0;

  RCLCPP_INFO(
    get_logger(),
    "stats rate=%.1f provider=%s cost_mean=%.2f infer=%.2f max=%.2f dropped=%zu failures=%lu",
    static_cast<double>(window_out) / elapsed, engine_->provider().c_str(),
    total_mean, infer_mean, total_max_ms_.load(std::memory_order_relaxed),
    dropped - last_logged_dropped_,
    static_cast<unsigned long>(failures_.load(std::memory_order_relaxed)));

  last_log_ = now;
  last_logged_out_ = out;
  last_logged_dropped_ = dropped;
  last_infer_sum_ms_ = infer_sum;
  last_total_sum_ms_ = total_sum;
  total_max_ms_.store(0.0, std::memory_order_relaxed);
}

}  // namespace pimesh_perception

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_perception::DepthNode)
