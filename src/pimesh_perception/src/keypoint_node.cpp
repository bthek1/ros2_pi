// ORB, tracks, and a pose that knows what it does not know.

#include "pimesh_perception/keypoint_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "pimesh_perception/image_buffer.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

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
        "to motion.", 1, 60)));
  tracker_config.max_distance = static_cast<int>(
    declare_parameter(
      "match_max_distance", 64,
      describe_int(
        "Hamming distance ceiling, of 256 bits. Unrelated ORB descriptors sit "
        "near 128 apart, so 64 is already generous; a lookalike corner is worse "
        "than no corner.", 8, 256)));

  min_pairs_ = static_cast<std::size_t>(
    declare_parameter(
      "min_matched_pairs", 8,
      describe_int(
        "Consecutive pairs needed before a rotation is trusted. Below this the "
        "fit is interpolating noise.", 3, 500)));
  max_residual_rad_ = declare_parameter(
    "max_residual_rad", 0.03,
    describe_double(
      "Mean ray residual ceiling. 0.03 rad is 1.7 degrees, ~28 px at fx=953.",
      0.001, 1.0));
  reject_fraction_ = declare_parameter(
    "reject_fraction", 0.2,
    describe_double(
      "Fraction of worst pairs discarded before each refit. Never below the "
      "min_matched_pairs floor — rejecting until a fit looks good is how a robust "
      "estimator becomes a way of manufacturing agreement.", 0.0, 0.6));

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

  publish_tf_ = declare_parameter(
    "publish_tf", true,
    describe("Publish odom -> base_link. Rotation only; translation is zero by design."));
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"), describe("Parent frame."));
  base_frame_ = declare_parameter(
    "base_frame", std::string("base_link"), describe("Child frame — the body, not the camera."));

  const double stats_period_s = declare_parameter(
    "stats_period_s", 5.0,
    describe_double("How often to log the summary line.", 0.5, 120.0));

  tracker_ = std::make_unique<OrbTracker>(tracker_config);

  // --- QoS ------------------------------------------------------------------
  rclcpp::QoS image_qos(rclcpp::KeepLast(1));
  image_qos.reliable();

  // CameraInfo is transient-local at the publisher, so this matches it — and that
  // is what lets this node start after the camera and still get the intrinsics
  // without waiting for a new message. A VOLATILE reader here would silently wait
  // forever on a camera that had already said everything it was going to say.
  rclcpp::QoS info_qos(rclcpp::KeepLast(1));
  info_qos.reliable().transient_local();

  // Both topic names on the same line as their create_publisher call, which is not
  // only formatting: tools/gates/view-configs.sh greps src/ for the names a .rviz
  // is allowed to reference, and grep works a line at a time. A name wrapped onto
  // the next line is a name that gate cannot see, and it would then report a
  // perfectly good config as naming a topic nothing publishes.
  keypoints_pub_ = create_publisher<pimesh_msgs::msg::Keypoints>("/keypoints", image_qos);
  preview_pub_ =
    create_publisher<sensor_msgs::msg::CompressedImage>("/keypoints/image/compressed", image_qos);

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", image_qos,
    [this](std::unique_ptr<sensor_msgs::msg::Image> msg) {this->on_image(std::move(msg));});
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", info_qos,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {this->on_camera_info(msg);});

  // The basis for the optical -> body change of frame comes from TF, not from a
  // quaternion written here. There is exactly one publisher of that edge in this
  // project and exactly one place its numbers live; a second copy is how two
  // conventions end up one rotation apart with nothing failing.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  worker_ = std::thread([this] {this->work();});

  last_log_ = now();
  stats_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(stats_period_s)),
    [this] {this->log_stats();});

  // The regime, stated at startup, because "rotation only" is a property of this
  // node that every consumer of its TF has to know and no message carries.
  RCLCPP_INFO(
    get_logger(),
    "regime=rotation_only: %d ORB features, window %zu, Hamming <= %d; pose gated on "
    ">= %zu pairs and residual < %.3f rad. Translation is identically zero (P7 adds it).",
    tracker_config.max_features, tracker_config.match_window, tracker_config.max_distance,
    min_pairs_, max_residual_rad_);
}

KeypointNode::~KeypointNode()
{
  mailbox_.stop();
  if (worker_.joinable()) {worker_.join();}
}

void KeypointNode::on_image(std::unique_ptr<sensor_msgs::msg::Image> msg)
{
  // Bounded, and the bound is a pointer move. Everything expensive is on the
  // worker.
  mailbox_.push(std::move(msg));
}

void KeypointNode::on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  // P is the *projection* matrix and K the intrinsics; for a monocular camera with
  // no rectification they agree, and reading K is reading what the calibration
  // file says rather than what a rectifier would have produced.
  std::lock_guard<std::mutex> lock(k_mutex_);
  const bool first = !have_k_;
  k_ = cv::Matx33d(
    msg->k[0], msg->k[1], msg->k[2],
    msg->k[3], msg->k[4], msg->k[5],
    msg->k[6], msg->k[7], msg->k[8]);
  have_k_ = (k_(0, 0) != 0.0 && k_(1, 1) != 0.0);
  if (first && have_k_) {
    RCLCPP_INFO(
      get_logger(), "intrinsics fx=%.1f fy=%.1f cx=%.1f cy=%.1f from %s",
      k_(0, 0), k_(1, 1), k_(0, 2), k_(1, 2), msg->header.frame_id.c_str());
  }
}

void KeypointNode::work()
{
  while (!mailbox_.stopped()) {
    auto msg = mailbox_.pop(std::chrono::milliseconds(100));
    if (msg) {process_frame(std::move(msg));}
  }
}

void KeypointNode::process_frame(std::unique_ptr<sensor_msgs::msg::Image> msg)
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
  // The clock starts here and stops below, and what lies between is detection,
  // matching and the fit. *Not* the preview encode, which is accounted separately:
  // it is an output for a person, capped at 10 Hz, and folding it into the
  // per-frame cost would make the pipeline's own number depend on whether anybody
  // was watching. Both appear in the stats line so neither can hide in the other.
  const auto start = std::chrono::steady_clock::now();

  cv::cvtColor(frame_bgr, gray_, cv::COLOR_BGR2GRAY);
  const TrackedFrame tracked = tracker_->process(gray_);
  // The optical frame comes off the image's own header rather than a parameter of
  // this node: the frame those pixels were formed in is a fact about the frame,
  // and camera_node is the only thing entitled to state it.
  update_pose(tracked, msg->header.frame_id);

  const double cost_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

  cost_sum_ms_ = cost_sum_ms_.load() + cost_ms;
  if (cost_ms > cost_max_ms_.load()) {cost_max_ms_ = cost_ms;}
  detect_sum_ms_ = detect_sum_ms_.load() + tracked.detect_ms;
  match_sum_ms_ = match_sum_ms_.load() + tracked.match_ms;
  keypoints_sum_ = keypoints_sum_.load() + static_cast<double>(tracked.keypoints.size());
  matched_sum_ = matched_sum_.load() + tracked.matched_fraction();
  ++frames_;

  publish_keypoints(tracked, *msg);
  publish_pose(rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));

  // Throttled on the node's own clock, not on the frame stamp: the stamp comes
  // from the Pi and a rate computed from it would be measuring NTP.
  const rclcpp::Time now_ros = now();
  if (!have_preview_time_ || (now_ros - last_preview_).seconds() >= preview_period_s_) {
    last_preview_ = now_ros;
    have_preview_time_ = true;
    publish_preview(tracked, *msg);
  }
}

void KeypointNode::update_pose(const TrackedFrame & frame, const std::string & optical_frame)
{
  cv::Matx33d k;
  {
    std::lock_guard<std::mutex> lock(k_mutex_);
    if (!have_k_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no /camera_info yet — holding pose. Rays need K, and a pose from invented "
        "intrinsics is confidently wrong rather than absent.");
      ++pose_held_;
      return;
    }
    k = k_;
  }

  // The basis, once. A static transform is latched, so this succeeds on the first
  // frame in normal operation and keeps trying if bringup has not started yet.
  if (!have_basis_) {
    try {
      // TimePointZero: "the latest available", which for a latched static
      // transform is the only sensible request — asking for it *at* the frame's
      // stamp would compare the Pi's clock with this machine's and wait for a
      // transform that is already there.
      const auto tf = tf_buffer_->lookupTransform(
        base_frame_, optical_frame, tf2::TimePointZero);
      const auto & q = tf.transform.rotation;
      tf2::Matrix3x3 m(tf2::Quaternion(q.x, q.y, q.z, q.w));
      base_from_optical_ = cv::Matx33d(
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]);
      have_basis_ = true;
      RCLCPP_INFO(
        get_logger(), "basis %s <- %s from TF", base_frame_.c_str(), optical_frame.c_str());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no %s <- %s transform yet (%s) — holding pose",
        base_frame_.c_str(), optical_frame.c_str(), ex.what());
      ++pose_held_;
      return;
    }
  }

  if (frame.consecutive_pairs.size() < min_pairs_) {
    if (!holding_) {
      RCLCPP_INFO(
        get_logger(), "regime=hold: %zu consecutive pairs, need %zu",
        frame.consecutive_pairs.size(), min_pairs_);
      holding_ = true;
    }
    ++pose_held_;
    return;
  }

  std::vector<cv::Vec3d> from;
  std::vector<cv::Vec3d> to;
  from.reserve(frame.consecutive_pairs.size());
  to.reserve(frame.consecutive_pairs.size());
  for (const PixelPair & pair : frame.consecutive_pairs) {
    from.push_back(bearing(k, pair.previous.x, pair.previous.y));
    to.push_back(bearing(k, pair.current.x, pair.current.y));
  }

  const RotationFit fit =
    fit_rotation_robust(from, to, min_pairs_, max_residual_rad_, reject_fraction_);

  if (!fit.ok) {
    if (!holding_) {
      RCLCPP_INFO(
        get_logger(),
        "regime=hold: %zu/%zu pairs, residual %.4f rad (ceiling %.4f) — keeping the last pose",
        fit.pairs_used, fit.pairs_in, fit.residual_rad, max_residual_rad_);
      holding_ = true;
    }
    ++pose_held_;
    return;
  }

  if (holding_) {
    RCLCPP_INFO(
      get_logger(), "regime=rotation_only: recovered, %zu pairs at %.4f rad",
      fit.pairs_used, fit.residual_rad);
    holding_ = false;
  }

  // The increment is in the optical frame; the pose is in the body frame. Compose
  // in the body frame, which means changing the increment's basis first — and
  // *then* multiplying on the right, because this is a rotation of the camera
  // relative to where it was, not relative to odom.
  orientation_ = orientation_ * change_basis(base_from_optical_, fit.rotation);
  residual_sum_ = residual_sum_.load() + fit.residual_rad;
  ++pose_ok_;
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
  msg->track_id = frame.published_track_ids();

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

void KeypointNode::publish_pose(const rclcpp::Time & stamp)
{
  if (!publish_tf_ || !tf_broadcaster_ || !have_basis_) {return;}

  geometry_msgs::msg::TransformStamped tf;
  // The *frame's* stamp, not now(): this transform describes where the camera was
  // when those pixels were captured. A pose stamped with the moment the estimator
  // finished is a pose that claims the camera was somewhere it had already left.
  tf.header.stamp = stamp;
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;

  // Zero, and deliberately. Bearing rays cannot recover translation — with no
  // depth there is no scale, and with no baseline the essential matrix is
  // degenerate. Publishing an estimate here would be inventing one.
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;

  const cv::Vec4d q = quaternion_from_rotation(orientation_);
  tf.transform.rotation.x = q[0];
  tf.transform.rotation.y = q[1];
  tf.transform.rotation.z = q[2];
  tf.transform.rotation.w = q[3];

  tf_broadcaster_->sendTransform(tf);
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
  const std::uint64_t ok = pose_ok_.load();
  const std::uint64_t held = pose_held_.load();

  // One line, and every number in it is needed to read any of the others: a cost
  // without a drop count says nothing about keeping up, and a rate without the
  // matched fraction says nothing about whether the corners mean anything.
  RCLCPP_INFO(
    get_logger(),
    "stats rate=%.1fHz dropped=%zu kp=%.0f matched=%.3f cost_mean=%.2fms cost_max=%.2fms "
    "detect=%.2fms match=%.2fms preview=%.2fms pose_ok=%lu held=%lu reject_rate=%.3f "
    "residual=%.4frad",
    static_cast<double>(delta) / span_s,
    dropped_delta,
    keypoints_sum_.load() / frames_total,
    matched_sum_.load() / frames_total,
    cost_sum_ms_.load() / frames_total,
    cost_max_ms_.load(),
    detect_sum_ms_.load() / frames_total,
    match_sum_ms_.load() / frames_total,
    (previews_.load() > 0) ?
    preview_cost_sum_ms_.load() / static_cast<double>(previews_.load()) : 0.0,
    static_cast<unsigned long>(ok), static_cast<unsigned long>(held),
    (ok + held > 0) ? static_cast<double>(held) / static_cast<double>(ok + held) : 0.0,
    (ok > 0) ? residual_sum_.load() / static_cast<double>(ok) : 0.0);
}

}  // namespace pimesh_perception

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_perception::KeypointNode)
