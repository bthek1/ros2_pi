// One reader, one decode. The interesting lines are the unique_ptr moving from
// the subscription into the mailbox and out of the worker into publish();
// everything else is counters.

#include "pimesh_perception/decode_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/imgcodecs.hpp"
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

}  // namespace

DecodeNode::DecodeNode(const rclcpp::NodeOptions & options)
: Node("decode_node", options),
  last_log_(0, 0, RCL_ROS_TIME),
  last_arrival_(0, 0, RCL_ROS_TIME)
{
  const std::string input_topic = declare_parameter(
    "input_topic", std::string("/image_raw/compressed"),
    describe("The Pi's compressed stream. The only topic in this project that crosses Wi-Fi."));
  const std::string output_topic = declare_parameter(
    "output_topic", std::string("/image_raw"),
    describe(
      "Decoded bgr8, for consumers inside this container. Published as a "
      "unique_ptr so intra-process comms can hand the buffer over; a subscriber "
      "outside the process forces a 2.7 MB serialisation of every frame."));

  // RELIABLE or BEST_EFFORT on the *reader*, and this is a genuinely open
  // question rather than a setting with a known answer.
  //
  // The rule this project measured is that a BEST_EFFORT *publisher* delivers
  // zero megabyte-class frames — fragments past the socket buffer never
  // reassemble. The reader is a different matter. A RELIABLE reader delivers in
  // sequence, so one lost fragment head-of-line blocks every frame behind it for
  // a heartbeat round trip; measured on this link at ~80 kB a frame, a RELIABLE
  // reader saw 10 gaps over 50 ms in 20 s against BEST_EFFORT's 3, and
  // rviz/camera.rviz asks for BEST_EFFORT on exactly that evidence.
  //
  // That measurement was taken on a *viewer*, where a dropped frame costs
  // nothing. Here a dropped frame is a frame the mesh never sees, so the
  // trade-off is not the same one and is not yet measured. The parameter exists
  // so the experiment is a launch argument rather than a rebuild, and the gaps
  // counter below is the instrument; until somebody runs it, the default is the
  // conservative one.
  const std::string reliability = declare_parameter(
    "input_reliability", std::string("reliable"),
    describe(
      "reliable | best_effort, on the subscription only. See the long comment "
      "here before changing it: the answer for a viewer is not the answer for a "
      "pipeline stage, and neither is free."));

  gap_threshold_ms_ = declare_parameter(
    "gap_threshold_ms", 50.0,
    describe_range(
      "Inter-arrival gap that counts as a stall in the stats line. 50 ms is ~3 "
      "frame intervals at 59 Hz, and is the threshold the RELIABLE-vs-BEST_EFFORT "
      "measurement of 2026-09-09 used.", 1.0, 1000.0));

  log_payloads_ = declare_parameter(
    "log_payloads", false,
    describe(
      "Log the address of every published buffer. Off by default — at 59 Hz it "
      "is 59 log lines a second — and switched on by tools/gates/ipc.sh, which "
      "needs the publisher half of the pointer-handover evidence."));

  const double stats_period_s = declare_parameter(
    "stats_period_s", 5.0,
    describe_range("How often to log the throughput summary.", 0.5, 120.0));

  // --- QoS ------------------------------------------------------------------
  //
  // KEEP_LAST(1) on both sides. On the subscription it means the middleware hands
  // us the freshest frame and forgets the rest, which is the same decision the
  // mailbox makes one layer up; on the publisher it matters only for a
  // subscriber outside the process, because the intra-process path has its own
  // buffer per subscription.
  rclcpp::QoS in_qos(rclcpp::KeepLast(1));
  if (reliability == "best_effort") {
    in_qos.best_effort();
  } else {
    in_qos.reliable();
  }

  rclcpp::QoS out_qos(rclcpp::KeepLast(1));
  out_qos.reliable();

  pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, out_qos);

  // A unique_ptr callback, and it is half of the zero-copy contract: the
  // intra-process path can only *move* a message into a callback that is willing
  // to own it. `const &` here would compile, run, and copy — and the copy is
  // invisible in every log and every topic tool.
  sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    input_topic, in_qos,
    [this](std::unique_ptr<sensor_msgs::msg::CompressedImage> msg) {
      this->on_frame(std::move(msg));
    });

  worker_ = std::thread([this] {this->work();});

  last_log_ = now();
  stats_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(stats_period_s)),
    [this] {this->log_stats();});

  RCLCPP_INFO(
    get_logger(), "decoding %s -> %s, subscription %s",
    input_topic.c_str(), pub_->get_topic_name(), reliability.c_str());
}

DecodeNode::~DecodeNode()
{
  // Stop before join, or the worker sits in pop() until its timeout and the
  // destructor blocks for as long as that is. Shutdown order is the one part of
  // a worker-thread node that is easy to get wrong and never exercised until the
  // day a gate asserts the process exited.
  mailbox_.stop();
  if (worker_.joinable()) {worker_.join();}
}

void DecodeNode::on_frame(std::unique_ptr<sensor_msgs::msg::CompressedImage> msg)
{
  // The callback's whole job: note the arrival and hand the pointer on. No
  // decode, no allocation, no logging on the hot path.
  const rclcpp::Time arrival = now();
  if (have_arrival_) {
    const double gap_ms = (arrival - last_arrival_).seconds() * 1e3;
    if (gap_ms > gap_threshold_ms_) {++gaps_;}
  }
  last_arrival_ = arrival;
  have_arrival_ = true;
  ++frames_in_;

  mailbox_.push(std::move(msg));
}

void DecodeNode::work()
{
  while (!mailbox_.stopped()) {
    auto msg = mailbox_.pop(std::chrono::milliseconds(100));
    if (msg) {decode_one(std::move(msg));}
  }
}

void DecodeNode::decode_one(std::unique_ptr<sensor_msgs::msg::CompressedImage> msg)
{
  const auto start = std::chrono::steady_clock::now();

  // A cv::Mat header over the JPEG bytes we were handed — imdecode wants a Mat
  // and this is not a copy of the 80 kB.
  const cv::Mat encoded(
    1, static_cast<int>(msg->data.size()), CV_8UC1,
    const_cast<unsigned char *>(msg->data.data()));

  // IMREAD_COLOR gives BGR, which is what the `bgr8` encoding string promises.
  // Into a member Mat, so the 2.7 MB is allocated once for the life of the node
  // rather than once per frame.
  cv::imdecode(encoded, cv::IMREAD_COLOR, &bgr_);
  if (bgr_.empty()) {
    ++failures_;
    // Throttled, because a camera sending garbage sends it at 59 Hz and a log
    // that scrolls is a log nobody reads.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "imdecode failed on a %zu byte payload (format '%s')",
      msg->data.size(), msg->format.c_str());
    return;
  }

  auto out = std::make_unique<sensor_msgs::msg::Image>();
  // The source frame's header, unchanged. Derived data keeps the header of what
  // it describes: the stamp is the kernel's capture time from the Pi, not the
  // moment this decode finished, and the frame_id is still the optical frame the
  // pixels were formed in.
  out->header = msg->header;
  fill_bgr8(*out, bgr_);

  const double cost_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  cost_sum_ms_ = cost_sum_ms_.load() + cost_ms;
  if (cost_ms > cost_max_ms_.load()) {cost_max_ms_ = cost_ms;}
  ++frames_out_;
  ++seq_;

  if (log_payloads_) {
    // Read the address *before* the move; afterwards the unique_ptr is null.
    // This is the publisher half of gates/ipc.sh's evidence, and it is printed
    // in the same shape pimesh_hello's does so one awk program reads both.
    RCLCPP_INFO(
      get_logger(), "decode seq=%lu payload=%p bytes=%zu cost=%.2fms",
      static_cast<unsigned long>(seq_), static_cast<const void *>(out.get()),
      out->data.size(), cost_ms);
  }

  pub_->publish(std::move(out));
}

void DecodeNode::log_stats()
{
  const rclcpp::Time stamp = now();
  const double span_s = (stamp - last_log_).seconds();
  if (span_s <= 0.0) {return;}

  const std::uint64_t in_now = frames_in_.load();
  const std::uint64_t out_now = frames_out_.load();
  const std::size_t dropped_now = mailbox_.dropped();

  const std::uint64_t in_delta = in_now - last_logged_in_;
  const std::uint64_t out_delta = out_now - last_logged_out_;
  const std::size_t dropped_delta = dropped_now - last_logged_dropped_;

  last_logged_in_ = in_now;
  last_logged_out_ = out_now;
  last_logged_dropped_ = dropped_now;
  last_log_ = stamp;

  if (in_delta == 0) {
    RCLCPP_WARN(get_logger(), "stats no frames in %.1fs — is the camera running?", span_s);
    return;
  }

  // `dropped` is the number the rest of this line has to be read against: a
  // stage decoding 13 of every 59 frames and a stage decoding all of them print
  // the same cost, and only one of them is keeping up.
  RCLCPP_INFO(
    get_logger(),
    "stats in=%.1fHz out=%.1fHz dropped=%zu failed=%lu gaps=%lu cost_mean=%.2fms cost_max=%.2fms",
    static_cast<double>(in_delta) / span_s,
    static_cast<double>(out_delta) / span_s,
    dropped_delta,
    static_cast<unsigned long>(failures_.load()),
    static_cast<unsigned long>(gaps_.load()),
    (out_now > 0) ? cost_sum_ms_.load() / static_cast<double>(out_now) : 0.0,
    cost_max_ms_.load());
}

}  // namespace pimesh_perception

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_perception::DecodeNode)
