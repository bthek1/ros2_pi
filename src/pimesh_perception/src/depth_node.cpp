#include "pimesh_perception/depth_node.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <utility>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace pimesh_perception
{

namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = description;
  d.read_only = true;
  return d;
}

rcl_interfaces::msg::ParameterDescriptor describe_int(
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

rcl_interfaces::msg::ParameterDescriptor describe_double(
  const std::string & description, double from, double to)
{
  auto d = describe(description);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  d.floating_point_range.push_back(range);
  return d;
}

double percentile(std::vector<double> samples, double fraction)
{
  if (samples.empty()) {
    return 0.0;
  }
  const size_t index = std::min(
    samples.size() - 1,
    static_cast<size_t>(fraction * static_cast<double>(samples.size())));
  std::nth_element(samples.begin(), samples.begin() + index, samples.end());
  return samples[index];
}

}  // namespace

DepthNode::DepthNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("depth_node", options)
{
  model_options_.model_path = declare_parameter<std::string>(
    "model_path", "",
    describe(
      "absolute path to the Depth Anything V2 Small ONNX file. Fetched and "
      "checksummed by `just fetch-model`; never committed, and never synced "
      "to the Pi."));
  model_options_.input_side = declare_parameter<int>(
    "input_side", 518,
    describe_int(
      "model input side. MUST be a multiple of 14 — the ViT works on 14x14 "
      "patches — and 518 = 37 x 14 is the size it was trained at.", 14, 1036));
  model_options_.use_cuda = declare_parameter<bool>(
    "use_cuda", true,
    describe(
      "try the CUDA execution provider. false forces the CPU path, which is "
      "for measuring the fallback deliberately rather than discovering it."));
  model_options_.depth_scale = declare_parameter<double>(
    "depth_scale", 10.0,
    describe_double(
      "turns the model's relative INVERSE depth into metres: z = scale / "
      "output. Monocular depth has no absolute scale, so this number is "
      "ARBITRARY until P5 pins it with a tape measure — the predecessor's "
      "room came out at 2.69. Nothing downstream may assume metres are real "
      "before then.", 0.01, 1000.0));
  model_options_.max_depth_m = declare_parameter<double>(
    "max_depth_m", 6.0,
    describe_double(
      "everything further reads as exactly this. Applied BEFORE the "
      "reciprocal, by flooring the inverse depth, so 1/x never explodes on "
      "the values the model means as 'background, no idea'.", 0.1, 100.0));

  const auto stats_period_s = declare_parameter<double>(
    "stats_period_s", 1.0, describe("how often to publish /pipeline/stats"));
  stale_after_s_ = declare_parameter<double>(
    "stale_after_s", 5.0,
    describe(
      "no frame for this long marks the stage stale. Longer than the other "
      "stages' 2 s because this one runs at ~19 Hz by design and a GPU hiccup "
      "should not flap the flag. Measured on RECEIPT time, never "
      "header.stamp."));
  optical_frame_ = declare_parameter<std::string>(
    "optical_frame", "camera_optical_frame",
    describe("frame_id stamped on /depth and /depth/rgb"));

  if (model_options_.model_path.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "model_path is empty — set it in config/pimesh.yaml. Run "
      "`just fetch-model` if the file is not there.");
    load_failed_.store(true);
  }

  if (!options.use_intra_process_comms()) {
    RCLCPP_WARN(
      get_logger(),
      "intra-process comms OFF — this node is COPYING every 2.7 MB frame out "
      "of decode_node. Expected only when running standalone.");
  }

  const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

  depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth", image_qos);
  rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/rgb", image_qos);
  stats_pub_ = create_publisher<pimesh_msgs::msg::PipelineStats>(
    "/pipeline/stats", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  window_start_ = std::chrono::steady_clock::now();
  last_frame_ = window_start_;

  worker_ = std::thread(&DepthNode::work_loop, this);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "rgb/image", image_qos,
    std::bind(&DepthNode::on_frame, this, std::placeholders::_1));

  stats_timer_ = create_wall_timer(
    std::chrono::duration<double>(stats_period_s),
    std::bind(&DepthNode::publish_stats, this));
}

DepthNode::~DepthNode()
{
  running_.store(false);
  mailbox_.close();
  if (worker_.joinable()) {
    worker_.join();
  }
}

void DepthNode::on_frame(ImageConstPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    last_frame_ = std::chrono::steady_clock::now();
    ever_received_ = true;
  }
  // Dropped here, deliberately and in quantity: inference runs at ~19 Hz
  // against a 42-60 Hz camera, so roughly two frames in three are overtaken
  // before the worker looks at the slot. The mailbox counts them.
  mailbox_.put(std::move(msg));
}

void DepthNode::work_loop()
{
  using clock = std::chrono::steady_clock;

  // Loading the model and warming a CUDA session takes seconds. Doing it here
  // rather than in the constructor keeps the component container responsive
  // while every other component loads — a container that blocks for four
  // seconds on one component looks exactly like a container that hung.
  if (!load_failed_.load()) {
    try {
      const auto t0 = clock::now();
      model_ = std::make_unique<DepthModel>(model_options_);
      const double load_ms =
        std::chrono::duration<double, std::milli>(clock::now() - t0).count();
      ready_.store(true);

      if (model_->provider() == Provider::kCuda) {
        RCLCPP_INFO(
          get_logger(),
          "%s, %dx%d, loaded and warmed in %.0f ms; '%s' -> '%s'",
          to_string(model_->provider()), model_options_.input_side,
          model_options_.input_side, load_ms,
          model_->input_name().c_str(), model_->output_name().c_str());
      } else {
        // Not an INFO line with a note in it. A CPU session is ~290 ms a frame
        // against the GPU's ~53 on this box: the pipeline still runs, produces
        // plausible output, and is five times too slow. It has to be
        // impossible to miss in a log.
        RCLCPP_ERROR(
          get_logger(),
          "%s — the GPU was NOT used. Expect ~290 ms/frame instead of ~53. "
          "Check `just gpu-probe`; the usual cause is a missing or "
          "mismatched CUDA runtime on LD_LIBRARY_PATH.",
          to_string(model_->provider()));
      }
    } catch (const std::exception & e) {
      load_failed_.store(true);
      RCLCPP_ERROR(
        get_logger(), "could not load the depth model: %s", e.what());
    }
  }

  ImageConstPtr frame;
  while (running_.load() && mailbox_.take(frame)) {
    if (!ready_.load()) {
      skipped_.fetch_add(1);
      continue;
    }
    if (frame->encoding != "bgr8" ||
      frame->data.size() < static_cast<std::size_t>(frame->step) * frame->height)
    {
      skipped_.fetch_add(1);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "unusable frame: encoding '%s', %zu bytes for %ux%u",
        frame->encoding.c_str(), frame->data.size(), frame->width, frame->height);
      continue;
    }

    const auto t0 = clock::now();

    // A cv::Mat header over the message's bytes — no copy. With intra-process
    // comms on these are the bytes decode_node wrote.
    const cv::Mat bgr(
      static_cast<int>(frame->height), static_cast<int>(frame->width), CV_8UC3,
      const_cast<uint8_t *>(frame->data.data()), frame->step);

    try {
      model_->infer(bgr, depth_);
    } catch (const std::exception & e) {
      skipped_.fetch_add(1);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "inference failed: %s", e.what());
      continue;
    }

    // The RGB twin goes out FIRST, so an exact-time synchroniser downstream
    // can complete the pair the moment /depth lands rather than holding the
    // depth map for a frame it has not seen yet.
    auto rgb_out = std::make_unique<sensor_msgs::msg::Image>();
    rgb_out->header = frame->header;          // the CAPTURE stamp, carried through
    rgb_out->header.frame_id = optical_frame_;
    rgb_out->height = frame->height;
    rgb_out->width = frame->width;
    rgb_out->encoding = "bgr8";
    rgb_out->is_bigendian = 0;
    rgb_out->step = frame->step;
    rgb_out->data = frame->data;              // a copy: the pair must be exact
    rgb_pub_->publish(std::move(rgb_out));

    auto depth_out = std::make_unique<sensor_msgs::msg::Image>();
    depth_out->header = frame->header;
    depth_out->header.frame_id = optical_frame_;
    depth_out->height = static_cast<uint32_t>(depth_.rows);
    depth_out->width = static_cast<uint32_t>(depth_.cols);
    depth_out->encoding = "32FC1";
    depth_out->is_bigendian = 0;
    depth_out->step = static_cast<uint32_t>(depth_.cols * sizeof(float));
    depth_out->data.resize(static_cast<size_t>(depth_out->step) * depth_out->height);
    std::memcpy(depth_out->data.data(), depth_.data, depth_out->data.size());
    depth_pub_->publish(std::move(depth_out));

    const double ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    processed_.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(window_mutex_);
      window_ms_.push_back(ms);
    }
  }
}

void DepthNode::publish_stats()
{
  std::vector<double> window;
  double elapsed = 0.0;
  bool stale = true;
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    const auto now_steady = std::chrono::steady_clock::now();
    window.swap(window_ms_);
    elapsed = std::chrono::duration<double>(now_steady - window_start_).count();
    window_start_ = now_steady;
    const double since_frame = std::chrono::duration<double>(now_steady - last_frame_).count();
    stale = !ever_received_ || since_frame > stale_after_s_;
  }

  const double total_ms = std::accumulate(window.begin(), window.end(), 0.0);

  pimesh_msgs::msg::PipelineStats stats;
  stats.header.stamp = now();
  stats.header.frame_id = optical_frame_;
  stats.stage = "depth";
  stats.rate_hz = elapsed > 0.0 ?
    static_cast<float>(static_cast<double>(window.size()) / elapsed) : 0.0f;
  stats.latency_ms = window.empty() ?
    0.0f : static_cast<float>(total_ms / static_cast<double>(window.size()));
  stats.latency_p95_ms = static_cast<float>(percentile(window, 0.95));
  stats.processed = processed_.load();
  // Expected to be LARGE here, and it is the design working: ~19 Hz of
  // inference against a 42-60 Hz camera means most frames are overtaken.
  stats.dropped_mailbox = mailbox_.dropped();
  stats.dropped_transport = skipped_.load();
  stats.stale = stale;

  // key=value, because a gate has to parse it. `provider` is the field the
  // whole phase turns on.
  char detail[320];
  std::snprintf(
    detail, sizeof(detail),
    "provider=%s state=%s side=%d depth_scale=%.3f max_depth_m=%.2f",
    ready_.load() ? to_string(model_->provider()) : "none",
    load_failed_.load() ? "failed" : (ready_.load() ? "ready" : "loading"),
    model_options_.input_side, model_options_.depth_scale,
    model_options_.max_depth_m);
  stats.detail = detail;

  stats_pub_->publish(stats);
}

}  // namespace pimesh_perception

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_perception::DepthNode)
