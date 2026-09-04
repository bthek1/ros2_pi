#include "pimesh_perception/decode_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <numeric>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "pimesh_perception/jpeg.hpp"

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

DecodeNode::DecodeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("decode_node", options)
{
  const auto stats_period_s = declare_parameter<double>(
    "stats_period_s", 1.0, describe("how often to publish /pipeline/stats"));
  stale_after_s_ = declare_parameter<double>(
    "stale_after_s", 2.0,
    describe(
      "no frame for this long marks the stage stale. Measured on RECEIPT time, "
      "never on header.stamp — see CLAUDE.md on why stamp ages cannot be "
      "trusted from a driver we do not control."));
  log_addresses_ = declare_parameter<int>(
    "log_buffer_addresses", 0,
    describe_range(
      "log the address of the first N published frames, so gate-ipc can prove "
      "the container really is zero-copy. 0 in normal operation.", 0, 1000));

  // Intra-process comms is a container flag, but a node can be loaded into a
  // plain process by mistake. Say which one this is, once, at startup: a
  // pipeline quietly serialising 2.7 MB per stage looks exactly like a working
  // one until you measure it.
  if (options.use_intra_process_comms()) {
    RCLCPP_INFO(get_logger(), "intra-process comms ON — downstream stages share this buffer");
  } else {
    RCLCPP_WARN(
      get_logger(),
      "intra-process comms OFF — every downstream stage gets its own COPY of "
      "each 2.7 MB frame. Expected only when running standalone.");
  }

  // Must match the camera's: RELIABLE because BEST_EFFORT delivers zero
  // megabyte-class frames once they fragment, KEEP_LAST(1) because a slow
  // consumer wants the freshest frame and not a backlog.
  const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

  // VOLATILE, deliberately. transient_local would latch the last frame for
  // late joiners — and would also disable the intra-process path, which is the
  // entire point of this node. A live video stream has nothing to offer a late
  // joiner anyway: the next frame is 30 ms away.
  image_pub_ = create_publisher<sensor_msgs::msg::Image>("rgb/image", image_qos);
  stats_pub_ = create_publisher<pimesh_msgs::msg::PipelineStats>(
    "/pipeline/stats", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  window_start_ = std::chrono::steady_clock::now();
  last_frame_ = window_start_;

  worker_ = std::thread(&DecodeNode::decode_loop, this);

  image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    "image_raw/compressed", image_qos,
    std::bind(&DecodeNode::on_frame, this, std::placeholders::_1));

  stats_timer_ = create_wall_timer(
    std::chrono::duration<double>(stats_period_s),
    std::bind(&DecodeNode::publish_stats, this));

  RCLCPP_INFO(
    get_logger(), "decoding %s → %s",
    image_sub_->get_topic_name(), image_pub_->get_topic_name());
}

DecodeNode::~DecodeNode()
{
  running_.store(false);
  mailbox_.close();          // wakes the worker parked on the condition variable
  if (worker_.joinable()) {
    worker_.join();
  }
}

void DecodeNode::on_frame(CompressedConstPtr msg)
{
  // The entire callback. Moving a shared_ptr is a refcount bump — the 150 kB
  // of JPEG is not touched, and the executor thread is free again immediately.
  // Whatever was waiting is dropped, and the mailbox counts it.
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    last_frame_ = std::chrono::steady_clock::now();
    ever_received_ = true;
  }
  mailbox_.put(std::move(msg));
}

void DecodeNode::decode_loop()
{
  using clock = std::chrono::steady_clock;

  CompressedConstPtr frame;
  while (running_.load() && mailbox_.take(frame)) {
    const auto t0 = clock::now();

    if (!decode_bgr8(frame->data, bgr_)) {
      // Normal over Wi-Fi: a frame that lost a fragment cannot be reassembled.
      // Count it and move on — throwing here would take the pipeline down every
      // few minutes.
      const auto n = failed_.fetch_add(1) + 1;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "undecodable frame (%zu bytes, format '%s'); %lu so far",
        frame->data.size(), frame->format.c_str(), static_cast<unsigned long>(n));
      continue;
    }

    // A unique_ptr, because that is what rclcpp needs in order to hand the
    // buffer downstream instead of serialising it. Filled by hand rather than
    // through cv_bridge, whose toImageMsg() returns a shared_ptr and would
    // therefore cost an extra copy on every frame.
    auto out = std::make_unique<sensor_msgs::msg::Image>();
    out->header = frame->header;          // the CAPTURE stamp, carried through
    out->height = static_cast<uint32_t>(bgr_.rows);
    out->width = static_cast<uint32_t>(bgr_.cols);
    out->encoding = "bgr8";
    out->is_bigendian = 0;
    out->step = static_cast<uint32_t>(bgr_.cols * 3);
    out->data.resize(static_cast<size_t>(out->step) * out->height);
    // The one copy in this node: cv::Mat owns its pixels and the message must
    // own its bytes. ~2.7 MB, ~0.3 ms — against a ~4 ms decode, and it buys
    // every downstream stage a copy-free pointer.
    std::memcpy(out->data.data(), bgr_.data, out->data.size());

    const double ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    {
      std::lock_guard<std::mutex> lock(window_mutex_);
      window_ms_.push_back(ms);
    }

    // Read the address BEFORE the move. Moving a std::vector transfers the
    // heap buffer without touching it, so this is the same address the
    // subscriber will see if — and only if — the intra-process path is used.
    const void * buffer = static_cast<const void *>(out->data.data());
    const int64_t stamp_ns = rclcpp::Time(out->header.stamp).nanoseconds();

    image_pub_->publish(std::move(out));
    decoded_.fetch_add(1);

    if (log_addresses_ > 0) {
      --log_addresses_;
      RCLCPP_INFO(get_logger(), "ipc published stamp=%ld buffer=%p", stamp_ns, buffer);
    }
  }
}

void DecodeNode::publish_stats()
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
  stats.header.frame_id = "camera_optical_frame";
  stats.stage = "decode";
  stats.rate_hz = elapsed > 0.0 ?
    static_cast<float>(static_cast<double>(window.size()) / elapsed) : 0.0f;
  stats.latency_ms = window.empty() ?
    0.0f : static_cast<float>(total_ms / static_cast<double>(window.size()));
  stats.latency_p95_ms = static_cast<float>(percentile(window, 0.95));
  stats.processed = decoded_.load();
  // Frames a newer one overtook. Expected to be zero here — decode keeps up
  // with the camera — and expected to be large at the depth stage.
  stats.dropped_mailbox = mailbox_.dropped();
  stats.dropped_transport = failed_.load();
  stats.stale = stale;
  stats.detail = "cv::imdecode bgr8";
  stats_pub_->publish(stats);
}

}  // namespace pimesh_perception

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_perception::DecodeNode)
