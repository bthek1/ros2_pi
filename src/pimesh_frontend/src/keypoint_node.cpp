// ORB: corners, who each one is, and where it was a frame ago.
//
// The pose left this file on 2026-09-23 for odometry_node.cpp. What is here is
// the detector and the tracker, and the one output they produce.

#include "pimesh_frontend/keypoint_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "pimesh_core/image_buffer.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using pimesh_core::mat_over;
namespace pimesh_frontend
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

KeypointNode::KeypointNode(const rclcpp::NodeOptions & options)
: Node("keypoint_node", options),
  last_log_(0, 0, RCL_ROS_TIME),
  last_preview_(0, 0, RCL_ROS_TIME)
{
  OrbTracker::Config tracker_config;
  tracker_config.max_features = static_cast<int>(
    declare_parameter(
      "max_features", 500,
      describe_int(
        "ORB features per frame. 500 is enough that churn at the cap does not "
        "dominate and cheap enough for the full frame rate.", 50, 5000)));
  tracker_config.match_window = static_cast<std::size_t>(
    declare_parameter(
      "match_window", 10,
      describe_int(
        "Frames in the pooled matching window. 1 would be strict frame-to-frame "
        "matching, which loses ~25% of keypoints to detection churn rather than "
        "to motion. It is also what spans the gap between two depth frames: at "
        "59 Hz in and 17 Hz out, consecutive depth maps are ~3 frames apart, so "
        "a window under 4 would leave the 6-DoF estimator with no correspondences "
        "at all.", 1, 60)));
  tracker_config.max_distance = static_cast<int>(
    declare_parameter(
      "match_max_distance", 64,
      describe_int(
        "Hamming distance ceiling, of 256 bits. Unrelated ORB descriptors sit "
        "near 128 apart, so 64 is already generous; a lookalike corner is worse "
        "than no corner.", 8, 256)));

  preview_quality_ = static_cast<int>(
    declare_parameter(
      "preview_quality", 80, describe_int("JPEG quality of the annotated preview.", 10, 100)));
  const double preview_rate_hz = declare_parameter(
    "preview_rate_hz", 10.0,
    describe_double(
      "Cap on the annotated preview's rate. Encoding it costs more than detecting "
      "the features does, and it is for a person to look at — this is the one "
      "output in the pipeline whose rate is set by human eyes.", 0.5, 60.0));
  preview_period_s_ = 1.0 / preview_rate_hz;

  const double stats_period_s = declare_parameter(
    "stats_period_s", 5.0,
    describe_double("How often to log the summary line.", 0.5, 120.0));

  tracker_ = std::make_unique<OrbTracker>(tracker_config);

  // --- QoS ------------------------------------------------------------------
  rclcpp::QoS image_qos(rclcpp::KeepLast(1));
  image_qos.reliable();

  // **A deeper history than the image path, and it is a correctness requirement.**
  // odometry_node looks each keypoints message up by its exact stamp to pair it
  // with a depth map, and in rotation_only it composes a rotation increment out of
  // every one of them. A KEEP_LAST(1) writer holds only the newest sample for
  // retransmission, so a reader that fell a frame behind would lose that frame for
  // good — an increment that silently never happened, which under-rotates the pose
  // with nothing in any log to say so. The image path keeps 1 on purpose, because
  // there the freshest frame is the only one anybody wants.
  rclcpp::QoS keypoints_qos(rclcpp::KeepLast(120));
  keypoints_qos.reliable();

  // Both topic names on the same line as their create_publisher call, which is not
  // only formatting: tools/gates/view-configs.sh greps src/ for the names a .rviz
  // is allowed to reference, and grep works a line at a time. A name wrapped onto
  // the next line is a name that gate cannot see, and it would then report a
  // perfectly good config as naming a topic nothing publishes.
  keypoints_pub_ = create_publisher<pimesh_msgs::msg::Keypoints>("/keypoints", keypoints_qos);
  preview_pub_ =
    create_publisher<sensor_msgs::msg::CompressedImage>("/keypoints/image/compressed", image_qos);
  // A *shared const* pointer, not a unique_ptr. See the long comment on mailbox_
  // in the header: with more than one consumer on a topic, rclcpp copies the buffer
  // for every ownership-taking subscription but the last, and shares one buffer
  // between any number of const-shared ones. Measured at 0/574 frames shared the
  // wrong way round and every frame the right way.
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", image_qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {this->on_image(std::move(msg));});
  stats_pub_ = create_publisher<pimesh_msgs::msg::PipelineStats>("/pipeline/stats", 10);

  worker_ = std::thread([this] {this->work();});

  last_log_ = now();
  stats_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(stats_period_s)),
    [this] {this->log_stats();});

  RCLCPP_INFO(
    get_logger(),
    "%d ORB features, window %zu, Hamming <= %d. Corners, tracks and the pair each "
    "corner makes with the frame before it; the pose is odometry_node's.",
    tracker_config.max_features, tracker_config.match_window, tracker_config.max_distance);
}

KeypointNode::~KeypointNode()
{
  mailbox_.stop();
  if (worker_.joinable()) {worker_.join();}
}

void KeypointNode::on_image(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // Bounded, and the bound is a pointer move. Everything expensive is on the
  // worker.
  mailbox_.push(std::move(msg));
}

void KeypointNode::work()
{
  while (!mailbox_.stopped()) {
    auto msg = mailbox_.pop(std::chrono::milliseconds(100));
    if (msg) {process_frame(std::move(msg));}
  }
}

void KeypointNode::process_frame(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  // A header over the message's pixels. No copy: this is the buffer decode_node
  // allocated, handed over by pointer.
  const cv::Mat frame_bgr = mat_over(*msg);
  if (frame_bgr.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "unusable frame: %ux%u '%s' step=%u", msg->width, msg->height,
      msg->encoding.c_str(), msg->step);
    return;
  }

  // --- The measured work ------------------------------------------------------
  //
  // The clock starts here and stops below, and what lies between is detection and
  // matching — the whole of what this node does. *Not* the preview encode, which
  // is accounted separately: it is an output for a person, capped at 10 Hz, and
  // folding it into the per-frame cost would make the pipeline's own number depend
  // on whether anybody was watching. Nor the pose, which since 2026-09-23 is
  // odometry_node's and has its own row in /pipeline/stats.
  const auto start = std::chrono::steady_clock::now();

  cv::cvtColor(frame_bgr, gray_, cv::COLOR_BGR2GRAY);
  const TrackedFrame tracked = tracker_->process(gray_);
  const double cost_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

  cost_sum_ms_ = cost_sum_ms_.load() + cost_ms;
  if (cost_ms > cost_max_ms_.load()) {cost_max_ms_ = cost_ms;}
  detect_sum_ms_ = detect_sum_ms_.load() + tracked.detect_ms;
  match_sum_ms_ = match_sum_ms_.load() + tracked.match_ms;
  keypoints_sum_ = keypoints_sum_.load() + static_cast<double>(tracked.keypoints.size());
  // A frame with no features has no matched fraction — 0/0 is undefined, not zero —
  // so it is counted separately rather than averaged in. Averaging it in as zero
  // conflates "no corners in this part of the room" with "corners found and none
  // recognised", and only the second is this node's doing. Measured on bags/desk1:
  // the difference is ~5 points of matched fraction.
  if (tracked.keypoints.empty()) {
    ++empty_frames_;
  } else {
    matched_sum_ = matched_sum_.load() + tracked.matched_fraction();
    ++measured_frames_;
  }
  ++frames_;

  publish_keypoints(tracked, *msg);
  // Throttled on the node's own clock, not on the frame stamp: the stamp comes
  // from the Pi and a rate computed from it would be measuring NTP.
  const rclcpp::Time now_ros = now();
  if (!have_preview_time_ || (now_ros - last_preview_).seconds() >= preview_period_s_) {
    last_preview_ = now_ros;
    have_preview_time_ = true;
    publish_preview(tracked, *msg);
  }
}

void KeypointNode::publish_keypoints(
  const TrackedFrame & frame, const sensor_msgs::msg::Image & source)
{
  auto msg = std::make_unique<pimesh_msgs::msg::Keypoints>();
  // The source frame's header: derived data keeps the header of what it describes,
  // so this stamp is the kernel's capture time and not the moment ORB finished.
  msg->header = source.header;
  msg->image_width = source.width;
  msg->image_height = source.height;

  const std::size_t count = frame.keypoints.size();
  msg->x.resize(count);
  msg->y.resize(count);
  msg->size.resize(count);
  msg->angle.resize(count);
  msg->response.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    msg->x[i] = frame.keypoints[i].pt.x;
    msg->y[i] = frame.keypoints[i].pt.y;
    msg->size[i] = frame.keypoints[i].size;
    msg->angle[i] = frame.keypoints[i].angle;
    msg->response[i] = frame.keypoints[i].response;
  }
  // The track's own id, on every feature including the ones seen here for the
  // first time, with `is_new` carrying that fact separately. Until 2026-09-23 this
  // was -1 on a first sighting — one field saying two things, and the reason
  // odometry_node could not have been split off without changing it: a landmark
  // whose id is hidden on the frame a keyframe was taken from can never be matched
  // to the same landmark later.
  msg->track_id = frame.ids;
  msg->is_new.assign(frame.is_new.begin(), frame.is_new.end());

  // Where each of these corners was one frame ago, NaN where it was not mutually
  // matched. This is the *geometry's* pairing — a stricter, one-frame-apart match
  // than the pooled window the track ids come from — and it has to travel with the
  // message because the consumer that fits a rotation to it is no longer the node
  // that ran the matcher. A consumer reconstructing it from two frames' track ids
  // would get the looser pairing and a plausible, wrong residual.
  msg->prev_x.resize(count);
  msg->prev_y.resize(count);
  for (std::size_t i = 0; i < count && i < frame.previous_pixel.size(); ++i) {
    msg->prev_x[i] = frame.previous_pixel[i].x;
    msg->prev_y[i] = frame.previous_pixel[i].y;
  }

  // 32 bytes per feature, row-major, and `descriptor_bytes` is on the wire because
  // the Hamming threshold downstream is quoted in bits and a consumer needs the
  // denominator.
  msg->descriptor_bytes =
    frame.descriptors.empty() ? 0U : static_cast<std::uint32_t>(frame.descriptors.cols);
  if (!frame.descriptors.empty()) {
    msg->descriptors.resize(static_cast<std::size_t>(frame.descriptors.total()));
    for (int row = 0; row < frame.descriptors.rows; ++row) {
      std::memcpy(
        msg->descriptors.data() + static_cast<std::size_t>(row) * frame.descriptors.cols,
        frame.descriptors.ptr(row), static_cast<std::size_t>(frame.descriptors.cols));
    }
  }

  keypoints_pub_->publish(std::move(msg));
}

void KeypointNode::publish_preview(
  const TrackedFrame & frame, const sensor_msgs::msg::Image & source)
{
  const auto start = std::chrono::steady_clock::now();

  const cv::Mat frame_bgr = mat_over(source);
  if (frame_bgr.empty()) {return;}
  // A copy, because this draws on it and the buffer belongs to the message every
  // other consumer in this container is also holding.
  frame_bgr.copyTo(preview_);

  for (std::size_t i = 0; i < frame.keypoints.size(); ++i) {
    // Green where the feature was already being followed, yellow where this frame
    // is the first sighting. The ratio of the two is the whole story at a glance:
    // mostly green is tracking, mostly yellow is a tracker that is detecting
    // corners and recognising none of them.
    const cv::Scalar colour = frame.is_new[i] ?
      cv::Scalar(0, 220, 255) : cv::Scalar(0, 255, 0);
    const cv::Point centre(
      static_cast<int>(frame.keypoints[i].pt.x), static_cast<int>(frame.keypoints[i].pt.y));
    cv::circle(preview_, centre, 3, colour, 1, cv::LINE_AA);
  }

  auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
  msg->header = source.header;
  msg->format = "jpeg";
  const std::vector<int> params {cv::IMWRITE_JPEG_QUALITY, preview_quality_};
  cv::imencode(".jpg", preview_, jpeg_, params);
  msg->data = jpeg_;

  preview_cost_sum_ms_ = preview_cost_sum_ms_.load() +
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  ++previews_;

  preview_pub_->publish(std::move(msg));
}

void KeypointNode::log_stats()
{
  const rclcpp::Time stamp = now();
  const double span_s = (stamp - last_log_).seconds();
  if (span_s <= 0.0) {return;}

  const std::uint64_t frames_now = frames_.load();
  const std::size_t dropped_now = mailbox_.dropped();
  const std::uint64_t delta = frames_now - last_frames_;
  const std::size_t dropped_delta = dropped_now - last_dropped_;
  last_frames_ = frames_now;
  last_dropped_ = dropped_now;
  last_log_ = stamp;

  if (delta == 0) {
    RCLCPP_WARN(get_logger(), "stats no frames in %.1fs — is decode_node running?", span_s);
    return;
  }

  const double frames_total = static_cast<double>(frames_now);
  const double measured = static_cast<double>(measured_frames_.load());

  // One line, and every number in it is needed to read any of the others: a cost
  // without a drop count says nothing about keeping up, and a rate without the
  // matched fraction says nothing about whether the corners mean anything.
  RCLCPP_INFO(
    get_logger(),
    "stats rate=%.1fHz dropped=%zu kp=%.0f matched=%.3f empty=%lu cost_mean=%.2fms "
    "cost_max=%.2fms detect=%.2fms match=%.2fms preview=%.2fms",
    static_cast<double>(delta) / span_s,
    dropped_delta,
    keypoints_sum_.load() / frames_total,
    (measured > 0.0) ? matched_sum_.load() / measured : 0.0,
    static_cast<unsigned long>(empty_frames_.load()),
    cost_sum_ms_.load() / frames_total,
    cost_max_ms_.load(),
    detect_sum_ms_.load() / frames_total,
    match_sum_ms_.load() / frames_total,
    (previews_.load() > 0) ?
    preview_cost_sum_ms_.load() / static_cast<double>(previews_.load()) : 0.0);

  // --- The same numbers, on a topic, for P8's dashboard ------------------------
  //
  // **One row, where this node published two until 2026-09-23.** It was two stages
  // wearing one name — ORB at the camera's rate on one thread, the pose solve at
  // the depth rate on another — and one row would have had to pick a rate, leaving
  // every number beside it belonging to something else. The pose is odometry_node's
  // now and reports its own row; this one is the detector and nothing else.
  auto ks = std::make_unique<pimesh_msgs::msg::PipelineStats>();
  ks->header.stamp = stamp;
  ks->stage = "keypoints";
  ks->rate_hz = static_cast<float>(static_cast<double>(delta) / span_s);
  ks->latency_ms = static_cast<float>(cost_sum_ms_.load() / frames_total);
  ks->latency_p95_ms = static_cast<float>(cost_max_ms_.load());
  ks->frames_in = frames_now + mailbox_.dropped();
  ks->frames_out = frames_now;
  ks->dropped_by_design = mailbox_.dropped();
  ks->dropped_in_transport = 0;
  char detail[160];
  std::snprintf(
    detail, sizeof(detail), "kp=%.0f matched=%.3f empty=%lu preview=%.2fms",
    keypoints_sum_.load() / frames_total,
    (measured > 0.0) ? matched_sum_.load() / measured : 0.0,
    static_cast<unsigned long>(empty_frames_.load()),
    (previews_.load() > 0) ?
    preview_cost_sum_ms_.load() / static_cast<double>(previews_.load()) : 0.0);
  ks->detail = detail;
  stats_pub_->publish(std::move(ks));
}

}  // namespace pimesh_frontend

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_frontend::KeypointNode)
