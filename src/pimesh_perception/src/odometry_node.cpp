// The pose: where the camera is, from corners and a depth map.
//
// Split out of keypoint_node.cpp on 2026-09-23. Everything here ran there from
// P3 (the rotation) and P7 (the 6-DoF solve); what changed is where the corners
// come from — a topic rather than a member — and that this node's stats line is
// its own rather than the second of two rows.

#include "pimesh_perception/odometry_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pimesh_perception/image_buffer.hpp"
#include "pimesh_perception/keyframe_store.hpp"
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

std::int64_t stamp_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

/// One phrase for whichever solver produced the numbers, so the log line does not
/// print a residual in metres under a solver that measures pixels.
std::string describe_fit(bool pnp, double residual_px, double residual_m, double scale)
{
  char buffer[128];
  if (pnp) {
    std::snprintf(buffer, sizeof(buffer), " at %.2f px reprojection", residual_px);
  } else {
    std::snprintf(buffer, sizeof(buffer), " at %.4f m (depth scale %.4f)", residual_m, scale);
  }
  return std::string(buffer);
}

const char * regime_name(OdometryRegime regime)
{
  return regime == OdometryRegime::SixDof ? "sixdof" : "rotation_only";
}

}  // namespace

OdometryNode::OdometryNode(const rclcpp::NodeOptions & options)
: Node("odometry_node", options),
  keyframes_(KeyframeStore::Config{}),
  last_log_(0, 0, RCL_ROS_TIME)
{
  // --- The regime -----------------------------------------------------------
  //
  // A string rather than a bool, because the two are different estimators and not
  // one estimator with a feature switched off — and because a bool named
  // `use_depth` would read as a fallback, which it is not. An unknown value is
  // refused rather than defaulted: silently running the control regime while a
  // config file asks for the real one is precisely the class of failure this
  // project keeps finding.
  const std::string regime_text = declare_parameter(
    "odometry", std::string("sixdof"),
    describe(
      "sixdof | rotation_only. sixdof pairs each frame's corners with its own "
      "depth map and fits a rigid transform; rotation_only fits bearing rays and "
      "publishes zero translation, which is what P3 did and what "
      "tools/gates/odom.sh measures against."));
  if (regime_text == "sixdof") {
    regime_ = OdometryRegime::SixDof;
  } else if (regime_text == "rotation_only") {
    regime_ = OdometryRegime::RotationOnly;
  } else {
    throw rclcpp::exceptions::InvalidParameterValueException(
            "odometry must be 'sixdof' or 'rotation_only', not '" + regime_text + "'");
  }

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

  // --- P7's gates, and they are the point-cloud versions of the three above ---
  const std::string solver = declare_parameter(
    "pose_solver", std::string("pnp"),
    describe(
      "pnp | points. pnp poses each frame from the keyframe's 3D landmarks and "
      "this frame's *pixels*, so only one depth map is involved and the "
      "translation comes from reprojection geometry. points fits a rigid "
      "transform between two unprojected clouds, which is the obvious thing to do "
      "and is the control: see PnpFit in rgbd_odometry.hpp for the four "
      "measurements that ended with it."));
  if (solver == "pnp") {
    solve_pnp_ = true;
  } else if (solver == "points") {
    solve_pnp_ = false;
  } else {
    throw rclcpp::exceptions::InvalidParameterValueException(
            "pose_solver must be 'pnp' or 'points', not '" + solver + "'");
  }
  reprojection_px_ = declare_parameter(
    "pnp_inlier_px", 3.0,
    describe_double(
      "RANSAC inlier threshold for the pose solve, in pixels. Generous beside the "
      "0.4955 px this camera's calibration reproject at, because the 3D points it "
      "is fitting come from a monocular depth network and carry its error, not the "
      "lens's.", 0.5, 30.0));
  max_reprojection_px_ = declare_parameter(
    "max_reprojection_px", 2.0,
    describe_double(
      "Mean inlier reprojection error past which the pose is refused and the last "
      "one held. **In pixels, which is the point**: the metres this pipeline works "
      "in are arbitrary until depth_scale is pinned with a tape measure, so a gate "
      "in metres is a gate on an unknown unit.", 0.1, 20.0));
  translation_tau_s_ = declare_parameter(
    "translation_tau_s", 0.5,
    describe_double(
      "Time constant of the low-pass on the measured position, in seconds; 0 "
      "disables it. The hand moves centimetres a second while the estimate's "
      "error is white at the depth rate and ~30 times larger per sample, so the "
      "two are separable by band even though no better estimator would separate "
      "them by accuracy.", 0.0, 10.0));
  max_speed_m_s_ = declare_parameter(
    "max_speed_m_s", 2.0,
    describe_double(
      "Speed past which a pose is refused as implausible rather than published. "
      "In the map's arbitrary units — landmarks are clipped at max_depth_m, so "
      "2.0 is about a third of the visible depth per second, which no hand-held "
      "sweep reaches. This is the gate a reprojection error cannot supply: PnP "
      "reports how well a pose explains the pixels, never whether the 3D points "
      "behind them were where the depth network said.", 0.01, 1000.0));
  max_turn_rad_s_ = declare_parameter(
    "max_turn_rad_s", 6.0,
    describe_double(
      "The same refusal for rotation. 6 rad/s is ~340 deg/s, past which a rolling "
      "shutter would have smeared the frame beyond tracking anyway.", 0.01, 100.0));
  max_hold_frames_ = static_cast<std::size_t>(
    declare_parameter(
      "max_hold_frames", 5,
      describe_int(
        "Consecutive depth frames without a usable fit before the reference view "
        "is replaced at the held pose. Without it, losing track once loses it for "
        "the rest of the session — measured as pose_ok frozen at 188 while held "
        "climbed past 560.", 1, 200)));
  keyframe_min_shared_ = static_cast<std::size_t>(
    declare_parameter(
      "keyframe_min_shared", 60,
      describe_int(
        "Landmarks still shared with the reference view, below which a new "
        "reference is taken whatever the angle and distance thresholds say. A "
        "keyframe the camera has turned away from satisfies both of those right up "
        "until it does not, and its last surviving correspondences are all in one "
        "corner of the frame.", 8, 500)));
  min_landmark_pairs_ = static_cast<std::size_t>(
    declare_parameter(
      "min_landmark_pairs", 12,
      describe_int(
        "Landmarks common to two depth-backed frames before a 6-DoF transform is "
        "trusted. Higher than min_matched_pairs because three points already "
        "determine a rigid transform exactly, so a handful of them fits its own "
        "noise perfectly and reports a residual of zero.", 4, 500)));
  max_point_residual_m_ = declare_parameter(
    "max_point_residual_m", 0.05,
    describe_double(
      "Mean landmark residual ceiling, in metres of the map's own units. Note "
      "those units are arbitrary until depth_scale is pinned with a tape measure "
      "— this number is a threshold on self-consistency, not on real distance.",
      0.001, 2.0));
  point_reject_fraction_ = declare_parameter(
    "point_reject_fraction", 0.3,
    describe_double(
      "Fraction of worst landmark pairs discarded before each refit. Higher than "
      "reject_fraction because a landmark outlier is unbounded: a ray can be at "
      "most 180 degrees wrong, a depth reading off the far side of an occlusion "
      "edge is metres out.", 0.0, 0.6));
  // **Measured, not reasoned — see ScaleHandling in rgbd_odometry.hpp.** false is
  // the control: it reported an 89.5 m path over a 45 s desk sweep and a surface
  // worse than rotation-only odometry, because a rigid fit has nowhere to put the
  // depth network's few percent of frame-to-frame scale wobble except into
  // translation along the view axis.
  scale_handling_ = declare_parameter(
    "divide_out_depth_scale", true,
    describe(
      "Estimate the scale difference between two frames' landmarks and discard "
      "it, keeping only rotation and translation. Depth Anything V2 estimates "
      "*relative* depth and its scale breathes; fusion_node's aligner exists for "
      "the same reason and hits its 15% clamp on 1 frame in 7 of bags/desk1.")) ?
    ScaleHandling::DivideOut : ScaleHandling::Rigid;
  depth_patch_ = static_cast<int>(
    declare_parameter(
      "depth_patch", 3,
      describe_int(
        "Side of the square window a keypoint's depth is taken as the median of. "
        "1 is a single pixel, which is the wrong reading to take at a corner: ORB "
        "puts features on edges by construction, and an edge is where the depth "
        "map steps between foreground and background.", 1, 15)));
  depth_patch_spread_ = declare_parameter(
    "depth_patch_spread", 0.1,
    describe_double(
      "Relative spread across that window past which the reading is refused as "
      "straddling a depth step. Relative rather than absolute, because depth "
      "error grows with distance.", 0.005, 1.0));
  min_depth_m_ = declare_parameter(
    "min_depth_m", 0.2,
    describe_double("Readings nearer than this are the model's noise floor.", 0.01, 5.0));
  max_depth_m_ = declare_parameter(
    "max_depth_m", 6.0,
    describe_double(
      "Readings past this are the model's 'far away or no idea' and must match "
      "depth_node's max_range_m. A landmark at the clip plane is a landmark on a "
      "surface that does not exist.", 0.5, 100.0));
  history_frames_ = static_cast<std::size_t>(
    declare_parameter(
      "history_frames", 90,
      describe_int(
        "Frames of ORB output kept waiting for their depth map. depth_node costs "
        "~55 ms, so a depth map arrives ~3 frames after its image; 90 frames is "
        "~1.5 s at 59 Hz, which is headroom of a different order rather than a "
        "tight fit.", 4, 600)));

  const double keyframe_angle_deg = declare_parameter(
    "keyframe_angle_deg", 18.0,
    describe_double(
      "View change admitting a new keyframe. Either this or keyframe_distance_m, "
      "not both: a camera panning on the spot travels no distance and still sees "
      "a different room.", 1.0, 90.0));
  const double keyframe_distance_m = declare_parameter(
    "keyframe_distance_m", 0.3,
    describe_double("Motion admitting a new keyframe.", 0.01, 10.0));
  const std::int64_t max_keyframes = declare_parameter(
    "max_keyframes", 500,
    describe_int(
      "Ceiling on the keyframe store. ~37 kB each — the descriptors alone are 16 "
      "kB — so 500 is ~18 MB. The newest keyframe is what each frame is posed "
      "against; matching against *every* one of them, to recognise a place seen "
      "minutes ago, is the deferred loop-closure work.", 1, 100000));
  keyframes_ = KeyframeStore(
    KeyframeStore::Config{
      keyframe_angle_deg * CV_PI / 180.0,
      keyframe_distance_m,
      static_cast<std::size_t>(max_keyframes)});

  publish_tf_ = declare_parameter(
    "publish_tf", true,
    describe("Publish odom -> base_link. Rotation only in the rotation_only regime."));
  odom_frame_ = declare_parameter("odom_frame", std::string("odom"), describe("Parent frame."));
  base_frame_ = declare_parameter(
    "base_frame", std::string("base_link"), describe("Child frame — the body, not the camera."));

  const std::string keypoints_topic = declare_parameter(
    "keypoints_topic", std::string("/keypoints"),
    describe(
      "ORB output from keypoint_node, in-process. Must name keypoint_node's own "
      "published topic: a mismatch is a node that never sees a corner and holds "
      "its pose for the whole session, which looks exactly like a camera that is "
      "not moving."));
  const std::string depth_topic = declare_parameter(
    "depth_topic", std::string("/depth"),
    describe(
      "32FC1 metres from depth_node, in-process. Must name depth_node's own "
      "depth_topic: a mismatch is a node that never sees a depth map and holds "
      "its pose for the whole session, which looks exactly like a camera that is "
      "not moving."));

  const double stats_period_s = declare_parameter(
    "stats_period_s", 5.0,
    describe_double("How often to log the summary line.", 0.5, 120.0));

  // --- QoS ------------------------------------------------------------------
  rclcpp::QoS image_qos(rclcpp::KeepLast(1));
  image_qos.reliable();

  // CameraInfo is transient-local at the publisher, so this matches it — and that
  // is what lets this node start after the camera and still get the intrinsics
  // without waiting for a new message. A VOLATILE reader here would silently wait
  // forever on a camera that had already said everything it was going to say.
  rclcpp::QoS info_qos(rclcpp::KeepLast(1));
  info_qos.reliable().transient_local();

  // The pose, as a message as well as a TF edge. Not a duplicate: TF is a tree
  // every consumer queries by time, and an Odometry stream is a *sequence* — which
  // is what an RViz Odometry display with a large `Keep` draws a trail from, and
  // what makes the trajectory visible without a nav_msgs/Path publisher existing
  // anywhere in this project. KeepLast(50) rather than 1, because a trail whose
  // samples are dropped is a trail with holes in it.
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(rclcpp::KeepLast(50)));

  // **KeepLast(1) would be wrong here and it is the one QoS in this node that
  // differs from the image path's.** Every keypoints message matters: the depth
  // rendezvous looks each one up by exact stamp, and in rotation_only the pairs
  // chain, so a message the middleware dropped to keep the queue at one is a
  // rotation increment that silently never happened. Depth below keeps 1, because
  // a depth map is a measurement against a keyframe rather than a link in a chain.
  rclcpp::QoS keypoints_qos(rclcpp::KeepLast(120));
  keypoints_qos.reliable();

  // A *shared const* pointer on both, never a unique_ptr: `/keypoints` has
  // keypoint_probe beside this node and `/depth` has fusion_node, so each is a
  // fan-out, and rclcpp copies the buffer for every ownership-taking subscription
  // but the last. See the note in the header.
  keypoints_sub_ = create_subscription<pimesh_msgs::msg::Keypoints>(
    keypoints_topic, keypoints_qos,
    [this](pimesh_msgs::msg::Keypoints::ConstSharedPtr msg) {this->on_keypoints(std::move(msg));});
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", info_qos,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {this->on_camera_info(msg);});
  // Subscribed in both regimes. In rotation_only nothing is done with the frames
  // beyond counting them, and that is on purpose: the control run then puts the
  // same load on /depth as the measured one, so a rate difference between the two
  // is a difference in the estimator rather than in how many consumers the
  // topic had.
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic, image_qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {this->on_depth(std::move(msg));});

  // The basis for the optical -> body change of frame comes from TF, not from a
  // quaternion written here. There is exactly one publisher of that edge in this
  // project and exactly one place its numbers live; a second copy is how two
  // conventions end up one rotation apart with nothing failing.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  stats_pub_ = create_publisher<pimesh_msgs::msg::PipelineStats>("/pipeline/stats", 10);

  pose_worker_ = std::thread([this] {this->pose_work();});

  last_log_ = now();
  stats_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(stats_period_s)),
    [this] {this->log_stats();});

  // The regime, stated at startup, because it is a property of this node that
  // every consumer of its TF has to know and no message carries.
  RCLCPP_INFO(
    get_logger(), "regime=%s: %s", regime_name(regime_),
    regime_ == OdometryRegime::SixDof ?
    "6-DoF from depth-backed landmarks on exact RGB-D pairs; the pose is published "
    "at the depth rate, at each depth map's own stamp." :
    "Bearing rays only — translation is identically zero. This is P3's estimator, "
    "kept as tools/gates/odom.sh's control run.");
  if (regime_ == OdometryRegime::SixDof) {
    RCLCPP_INFO(
      get_logger(),
      "each frame is measured against a keyframe, not against its predecessor. Pose "
      "gated on >= %zu shared landmarks and residual < %.3f m, from %d px patches with "
      "spread < %.0f%% over [%.2f, %.2f] m; a new keyframe at %.0f deg, %.2f m, or "
      "fewer than %zu shared. Solver: %s.",
      min_landmark_pairs_, max_point_residual_m_, depth_patch_,
      100.0 * depth_patch_spread_, min_depth_m_, max_depth_m_,
      keyframe_angle_deg, keyframe_distance_m, keyframe_min_shared_,
      solve_pnp_ ? "PnP on the keyframe's landmarks and this frame's pixels" :
      (scale_handling_ == ScaleHandling::DivideOut ?
      "a similarity fit between two point clouds" :
      "a rigid fit between two point clouds"));
  } else {
    RCLCPP_INFO(
      get_logger(),
      "pose gated on >= %zu pairs and residual < %.3f rad.",
      min_pairs_, max_residual_rad_);
  }
}

OdometryNode::~OdometryNode()
{
  depth_mailbox_.stop();
  if (pose_worker_.joinable()) {pose_worker_.join();}
}

void OdometryNode::on_keypoints(pimesh_msgs::msg::Keypoints::ConstSharedPtr msg)
{
  ++keypoint_frames_;

  const std::size_t count = msg->x.size();

  // --- The bounded copy ------------------------------------------------------
  //
  // Done here in the callback rather than on a worker, and it is the one thing in
  // this node that is allowed to be: it is a fixed-size copy of ~500 points and a
  // 16 kB descriptor block, with no decode, no inference and no solve in it. The
  // alternative — a mailbox and a thread — would be *worse than slow*, because a
  // mailbox is newest-wins: every message it dropped would be a frame whose ORB
  // output the depth rendezvous below can never find.
  auto record = std::make_shared<FrameRecord>();
  record->stamp_ns = stamp_ns(msg->header.stamp);
  record->optical_frame = msg->header.frame_id;
  record->pixels.resize(count);
  for (std::size_t i = 0; i < count; ++i) {
    record->pixels[i] = cv::Point2f(msg->x[i], msg->y[i]);
  }
  record->ids = msg->track_id;

  // A cv::Mat over its **own** storage, not over the message's. The message is a
  // shared const pointer that goes out of scope when this callback returns, and a
  // Mat header over its bytes would outlive them in the history — a use-after-free
  // that reads as plausible descriptors for as long as the allocator leaves the
  // page alone. `mat_over` exists for the case where the message *is* kept; this
  // is the case where it is not.
  const std::uint32_t stride = msg->descriptor_bytes;
  if (stride > 0 && msg->descriptors.size() == count * stride) {
    record->descriptors = cv::Mat(
      static_cast<int>(count), static_cast<int>(stride), CV_8U);
    std::memcpy(record->descriptors.data, msg->descriptors.data(), msg->descriptors.size());
  }

  {
    std::lock_guard<std::mutex> lock(history_mutex_);
    history_.push_back(std::move(record));
    while (history_.size() > history_frames_) {history_.pop_front();}
  }

  if (regime_ != OdometryRegime::RotationOnly) {return;}

  // --- The control regime's estimator ----------------------------------------
  //
  // Also in the callback, and unlike the copy above this is a *fit* — Kabsch over
  // a few hundred bearing pairs with up to three refits. It is here because it
  // cannot be anywhere else: the pairs in each message span the **detector's**
  // previous frame, so this chain composes correctly only if every message is
  // processed in order and none is skipped. A newest-wins mailbox would silently
  // drop increments and under-rotate; a growing queue is what this pipeline is
  // built to avoid. The cost is published as `rotation_cost` in the stats line
  // rather than assumed, because a claim about work in a callback should be a
  // number somebody can read.
  //
  // It is also the regime nothing but tools/gates/odom.sh runs.
  const auto start = std::chrono::steady_clock::now();

  std::vector<PixelPair> pairs;
  pairs.reserve(count);
  for (std::size_t i = 0; i < count && i < msg->prev_x.size(); ++i) {
    const cv::Point2f previous(msg->prev_x[i], msg->prev_y[i]);
    if (!TrackedFrame::matched_previous(previous)) {continue;}
    pairs.push_back(PixelPair{previous, cv::Point2f(msg->x[i], msg->y[i])});
  }

  update_pose(pairs, msg->header.frame_id);
  publish_pose(rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));

  rotation_cost_sum_ms_ = rotation_cost_sum_ms_.load() +
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

void OdometryNode::on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  depth_mailbox_.push(std::move(msg));
}

void OdometryNode::on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
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

void OdometryNode::pose_work()
{
  while (!depth_mailbox_.stopped()) {
    auto msg = depth_mailbox_.pop(std::chrono::milliseconds(100));
    if (msg) {process_depth(std::move(msg));}
  }
}

std::shared_ptr<const OdometryNode::FrameRecord> OdometryNode::record_at(std::int64_t want)
{
  std::lock_guard<std::mutex> lock(history_mutex_);
  // Newest first: a depth map is ~3 frames behind its image, so the match is near
  // the back and the search is a handful of comparisons rather than 90.
  for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
    if ((*it)->stamp_ns == want) {return *it;}
  }
  return nullptr;
}
void OdometryNode::process_depth(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  ++depth_frames_;
  if (regime_ != OdometryRegime::SixDof) {return;}

  const auto start = std::chrono::steady_clock::now();

  cv::Matx33d k;
  {
    std::lock_guard<std::mutex> lock(k_mutex_);
    if (!have_k_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no /camera_info yet — holding pose. Unprojection needs K, and a pose from "
        "invented intrinsics is confidently wrong rather than absent.");
      ++shift_held_;
      return;
    }
    k = k_;
  }

  const cv::Mat depth = depth_mat_over(*msg);
  if (depth.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "depth frame with encoding '%s' %ux%u step %u is not usable 32FC1",
      msg->encoding.c_str(), msg->width, msg->height, msg->step);
    ++shift_held_;
    return;
  }

  const std::int64_t when = stamp_ns(msg->header.stamp);
  const std::shared_ptr<const FrameRecord> record = record_at(when);
  if (!record) {
    // Not a warning per frame: at startup the depth maps of frames that went past
    // before this node had a tracker land here, and so does anything that outran
    // the history. The counter is what says whether it is a startup transient or a
    // history too shallow to hold the depth stage's latency.
    ++depth_unmatched_;
    ++shift_held_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "no ORB output at the depth map's own stamp — %lu so far. The history is %zu "
      "frames; a persistent count means depth has fallen further behind than that.",
      static_cast<unsigned long>(depth_unmatched_.load()), history_frames_);
    return;
  }

  if (!ensure_basis(record->optical_frame)) {
    ++shift_held_;
    return;
  }

  // --- This frame's landmarks -------------------------------------------------
  //
  // Sorted by track id, so the intersection with the reference below is a linear
  // merge. Built as tuples and sorted once rather than inserted into a map: ~250
  // entries, and a flat vector is both faster and the shape the merge wants.
  struct Entry
  {
    std::int32_t id;
    cv::Vec3d point;
    cv::Vec3d ray;
    cv::Point2f pixel;
  };
  std::vector<Entry> entries;
  entries.reserve(record->pixels.size());
  // Kept in *detection* order beside the sorted view, because the keyframe store
  // indexes its landmarks by the descriptor row they came from and sorted-by-id
  // order is not that.
  std::vector<std::int32_t> landmark_rows;
  std::vector<cv::Vec3d> landmark_points;
  landmark_rows.reserve(record->pixels.size());
  landmark_points.reserve(record->pixels.size());

  for (std::size_t i = 0; i < record->pixels.size(); ++i) {
    double metres = 0.0;
    if (!sample_depth(
        depth, record->pixels[i].x, record->pixels[i].y, depth_patch_,
        min_depth_m_, max_depth_m_, depth_patch_spread_, metres))
    {
      continue;
    }
    const cv::Vec3d point = unproject(k, record->pixels[i].x, record->pixels[i].y, metres);
    entries.push_back(
      Entry{record->ids[i], point, bearing(k, record->pixels[i].x, record->pixels[i].y),
        record->pixels[i]});
    landmark_rows.push_back(static_cast<std::int32_t>(i));
    landmark_points.push_back(point);
  }
  std::sort(
    entries.begin(), entries.end(),
    [](const Entry & l, const Entry & r) {return l.id < r.id;});

  RgbdView view;
  view.stamp_ns = when;
  view.ids.reserve(entries.size());
  view.points.reserve(entries.size());
  view.bearings.reserve(entries.size());
  view.pixels.reserve(entries.size());
  for (const Entry & entry : entries) {
    view.ids.push_back(entry.id);
    view.points.push_back(entry.point);
    view.bearings.push_back(entry.ray);
    view.pixels.push_back(entry.pixel);
  }
  landmark_sum_ = landmark_sum_.load() + static_cast<double>(view.points.size());

  // --- The correspondence, by track id, against the *reference* view -----------
  //
  // **By id, not by re-matching descriptors.** The tracker has already decided
  // which corner is which, once, against a ten-frame window that forgives the
  // detection churn a second matching pass would trip over — and a second opinion
  // about identity would be a second place for the two to disagree, silently.
  //
  // Against the reference rather than the previous frame: see `reference_` in the
  // header for the measurement that decided it. The pooled window is also what
  // makes it possible at all — a track id survives a feature flickering out for a
  // frame, which is how a landmark is still recognisable half a second later.
  //
  // **The current frame is matched by its keypoints, not by its landmarks.** PnP
  // wants the reference's 3D point and this frame's pixel, and a pixel needs no
  // depth of its own — so the correspondence set is built against every corner
  // ORB found here, roughly twice as many as have a usable depth reading. Only the
  // `points` control needs both sides in 3D.
  std::vector<cv::Vec3d> from;          // reference landmarks, reference optical frame
  std::vector<cv::Point2f> seen;        // where they are on this frame's sensor
  std::vector<cv::Vec3d> to;            // this frame's landmarks — the control only
  std::vector<cv::Vec3d> from_points;   // their reference twins — the control only
  if (reference_.valid()) {
    // This frame's corners, sorted by id, for the linear merge. Every corner, not
    // only the ones with depth.
    std::vector<std::pair<std::int32_t, cv::Point2f>> corners;
    corners.reserve(record->ids.size());
    for (std::size_t i = 0; i < record->ids.size(); ++i) {
      corners.emplace_back(record->ids[i], record->pixels[i]);
    }
    std::sort(
      corners.begin(), corners.end(),
      [](const auto & l, const auto & r) {return l.first < r.first;});

    std::size_t a = 0;
    std::size_t b = 0;
    while (a < reference_.ids.size() && b < corners.size()) {
      if (reference_.ids[a] < corners[b].first) {
        ++a;
      } else if (corners[b].first < reference_.ids[a]) {
        ++b;
      } else {
        from.push_back(reference_.points[a]);
        seen.push_back(corners[b].second);
        ++a;
        ++b;
      }
    }

    if (!solve_pnp_) {
      std::size_t x = 0;
      std::size_t y = 0;
      while (x < reference_.ids.size() && y < view.ids.size()) {
        if (reference_.ids[x] < view.ids[y]) {
          ++x;
        } else if (view.ids[y] < reference_.ids[x]) {
          ++y;
        } else {
          from_points.push_back(reference_.points[x]);
          to.push_back(view.points[y]);
          ++x;
          ++y;
        }
      }
    }
  }
  const std::size_t shared = solve_pnp_ ? from.size() : to.size();
  pair_sum_ = pair_sum_.load() + static_cast<double>(shared);

  // --- The solve ---------------------------------------------------------------
  bool advanced = false;
  double residual_m = 0.0;
  double residual_px = 0.0;
  double fitted_scale = 1.0;
  std::size_t pairs_used = 0;
  cv::Affine3d odom_from_camera;
  bool fit_ok = false;

  // The orientation as it stands, read once — used by the control path, and as the
  // pose to hold when nothing can be fitted.
  cv::Affine3d pose_now;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_now = pose_;
  }
  const cv::Affine3d odom_from_camera_now = pose_now * base_from_optical_;

  if (reference_.valid() && shared >= min_landmark_pairs_) {
    if (solve_pnp_) {
      const PnpFit fit = fit_pose_pnp(
        k, from, seen, min_landmark_pairs_, reprojection_px_, max_reprojection_px_);
      if (fit.ok) {
        odom_from_camera = reference_.odom_from_camera * camera_step(fit.motion());
        residual_px = fit.residual_px;
        pairs_used = fit.inliers;
        fit_ok = true;
        reprojection_sum_px_ = reprojection_sum_px_.load() + fit.residual_px;
        inlier_sum_ = inlier_sum_.load() + static_cast<double>(fit.inliers);
      }
    } else {
      // The control: a rigid fit between two unprojected clouds, which is the
      // obvious thing to do and is what four measurements said does not work here.
      const RigidFit fit = fit_rigid_robust(
        from_points, to, min_landmark_pairs_,
        max_point_residual_m_, point_reject_fraction_, 2, scale_handling_);
      if (fit.ok) {
        odom_from_camera = reference_.odom_from_camera * camera_step(fit.motion());
        residual_m = fit.residual_m;
        fitted_scale = fit.scale;
        pairs_used = fit.pairs_used;
        fit_ok = true;
      }
    }
  }

  // --- Is that a motion a camera could have made? ------------------------------
  //
  // Checked against the *last accepted* pose and the time since it, not against
  // the reference: a reference can be a second old, and what is implausible is a
  // speed rather than a displacement.
  // **`have_shift_` is bookkeeping for this gate, not for the filter**, and that
  // distinction cost a run: it started life inside the `translation_tau_s > 0`
  // branch, so with the filter off the gate below was skipped entirely and
  // reported `implausible=0` over a trajectory with a 6.95 m step in it. A guard
  // that is only armed when an unrelated feature is enabled is worse than no
  // guard, because its counter reads as evidence.
  if (fit_ok && have_shift_) {
    const double dt = static_cast<double>(when - last_shift_ns_) * 1e-9;
    if (dt > 0.0) {
      const cv::Affine3d posed = odom_from_camera * base_from_optical_.inv();
      const double moved =
        cv::norm(cv::Vec3d(posed.translation()) - cv::Vec3d(pose_now.translation()));
      const double turned = angle_between(pose_now.rotation(), posed.rotation());
      if (moved > max_speed_m_s_ * dt || turned > max_turn_rad_s_ * dt) {
        ++implausible_;
        RCLCPP_WARN(
          get_logger(),
          "refusing a pose that moved %.2f m and turned %.2f rad in %.0f ms — %.1f m/s and "
          "%.1f rad/s against ceilings of %.1f and %.1f. The fit was confident (%zu inliers "
          "at %.2f px); a reprojection error says nothing about whether the landmarks were "
          "where the depth network put them.",
          moved, turned, dt * 1e3, moved / dt, turned / dt, max_speed_m_s_, max_turn_rad_s_,
          pairs_used, residual_px);
        fit_ok = false;
      }
    }
  }

  if (fit_ok) {
    if (holding_) {
      RCLCPP_INFO(
        get_logger(),
        "regime=sixdof: recovered, %zu of %zu shared landmarks%s",
        pairs_used, shared, describe_fit(solve_pnp_, residual_px, residual_m, fitted_scale).c_str());
      holding_ = false;
    }
    holds_ = 0;
    // **Set, not accumulated.** The fit answers where this camera is *relative to
    // the reference*, so the pose is the reference's composed with it — once. An
    // increment applied to the running pose would add the whole keyframe-to-now
    // transform again at every frame, which is the mistake that made a simulated
    // keyframe run come out worse than frame-to-frame.
    cv::Affine3d posed = odom_from_camera * base_from_optical_.inv();

    // The low-pass, on the measured position. `alpha` is derived from the actual
    // interval between accepted measurements rather than assumed to be 1/17 s, so
    // a run of holds does not leave the filter lagging by however long they took:
    // after a 1 s gap at tau = 0.5 s, alpha is 0.86 and the filter has all but
    // caught up, which is the behaviour a fixed alpha would get wrong in exactly
    // the situation that matters.
    if (translation_tau_s_ > 0.0) {
      cv::Vec3d smoothed(posed.translation());
      if (have_shift_) {
        const double dt = static_cast<double>(when - last_shift_ns_) * 1e-9;
        const double alpha = (dt > 0.0) ? 1.0 - std::exp(-dt / translation_tau_s_) : 1.0;
        cv::Vec3d previous;
        {
          std::lock_guard<std::mutex> lock(pose_mutex_);
          previous = cv::Vec3d(pose_.translation());
        }
        smoothed = previous + alpha * (smoothed - previous);
      }
      posed = cv::Affine3d(posed.rotation(), smoothed);
      // The reference is anchored on the *filtered* pose, so what the next frame
      // is measured against is the pose that was actually published. Anchoring on
      // the raw one would make the filter a display convenience with the noise
      // still in the map.
      odom_from_camera = posed * base_from_optical_;
    }
    {
      std::lock_guard<std::mutex> lock(pose_mutex_);
      trajectory_m_ = trajectory_m_.load() +
        cv::norm(cv::Vec3d(posed.translation()) - cv::Vec3d(pose_.translation()));
      pose_ = posed;
    }
    last_shift_ns_ = when;
    have_shift_ = true;
    point_residual_sum_ = point_residual_sum_.load() + residual_m;
    scale_sum_ = scale_sum_.load() + fitted_scale;
    ++shift_ok_;
    // The pose is published at every depth stamp, accepted or held, because
    // fusion_node looks it up at exactly these stamps and waits up to its
    // tf_timeout_ms for each. A gap here is not a missing sample; it is a depth
    // frame dropped rather than integrated.
    advanced = true;
  } else {
    if (reference_.valid() && !holding_) {
      RCLCPP_INFO(
        get_logger(),
        "regime=hold: %zu shared landmarks with the reference view (floor %zu), and no "
        "pose under %.2f px — keeping the last one",
        shared, min_landmark_pairs_,
        solve_pnp_ ? max_reprojection_px_ : max_point_residual_m_);
      holding_ = true;
    }
    ++shift_held_;
    ++holds_;
    odom_from_camera = odom_from_camera_now;
  }

  publish_pose(rclcpp::Time(msg->header.stamp, RCL_ROS_TIME));

  // --- Retiring the reference --------------------------------------------------
  //
  // Four ways, and the last two are not about geometry at all.
  //
  // The shared-landmark floor: a keyframe the camera has turned away from
  // satisfies both thresholds right up until it does not, and its last surviving
  // correspondences sit in one corner of the frame, which is exactly the geometry
  // a translation fit is worst on.
  //
  // **And a run of holds, which is a deadlock rather than a refinement.** The
  // reference was only replaced on a successful fit in the first version of this,
  // so the first time tracking was lost it was lost for good: measured on
  // bags/desk1 as `pose_ok` frozen at 188 while `held` climbed past 560. Taking a
  // new reference at the held pose bakes in whatever drift the lost stretch cost,
  // and that is the right trade — it is a pose that stopped being updated against
  // one that stopped being updated *forever*.
  const bool lost = advanced && shared < keyframe_min_shared_;
  const bool moved = advanced && keyframes_.would_insert(odom_from_camera);
  const bool stuck = !advanced && holds_ >= max_hold_frames_;
  if (!reference_.valid() || lost || moved || stuck) {
    if (stuck) {
      ++keyframes_on_stall_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%zu depth frames without a usable fit — taking a new reference view here. The "
        "translation since the last good one is lost, not estimated.", holds_);
      holds_ = 0;
      // The speed gate measures from the last *accepted* pose, so a stall has to
      // move its clock too. Leaving it behind makes `dt` grow with the stall and
      // the ceiling grow with it, which is the gate quietly relaxing exactly when
      // tracking is worst.
      last_shift_ns_ = when;
      have_shift_ = true;
    } else if (lost && !moved) {
      ++keyframes_on_loss_;
    }

    Keyframe frame;
    frame.stamp_ns = when;
    frame.odom_from_camera = odom_from_camera;
    frame.descriptors = record->descriptors;
    frame.track_ids = record->ids;
    frame.bearings.reserve(record->pixels.size());
    for (const cv::Point2f & pixel : record->pixels) {
      frame.bearings.push_back(bearing(k, pixel.x, pixel.y));
    }
    frame.landmark_row = std::move(landmark_rows);
    frame.landmarks = std::move(landmark_points);
    keyframes_.maybe_insert(std::move(frame));

    view.odom_from_camera = odom_from_camera;
    reference_ = std::move(view);
  }

  pose_cost_sum_ms_ = pose_cost_sum_ms_.load() +
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
}

bool OdometryNode::ensure_basis(const std::string & optical_frame)
{
  std::lock_guard<std::mutex> lock(basis_mutex_);
  if (have_basis_) {return true;}

  try {
    // TimePointZero: "the latest available", which for a latched static
    // transform is the only sensible request — asking for it *at* the frame's
    // stamp would compare the Pi's clock with this machine's and wait for a
    // transform that is already there.
    const auto tf = tf_buffer_->lookupTransform(base_frame_, optical_frame, tf2::TimePointZero);
    const auto & q = tf.transform.rotation;
    const auto & t = tf.transform.translation;
    tf2::Matrix3x3 m(tf2::Quaternion(q.x, q.y, q.z, q.w));
    base_from_optical_ = cv::Affine3d(
      cv::Matx33d(
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]),
      cv::Vec3d(t.x, t.y, t.z));
    have_basis_ = true;
    RCLCPP_INFO(
      get_logger(), "basis %s <- %s from TF", base_frame_.c_str(), optical_frame.c_str());
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "no %s <- %s transform yet (%s) — holding pose",
      base_frame_.c_str(), optical_frame.c_str(), ex.what());
    return false;
  }
}

void OdometryNode::update_pose(
  const std::vector<PixelPair> & pairs, const std::string & optical_frame)
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

  if (!ensure_basis(optical_frame)) {
    ++pose_held_;
    return;
  }

  if (pairs.size() < min_pairs_) {
    if (!holding_) {
      RCLCPP_INFO(
        get_logger(), "regime=hold: %zu consecutive pairs, need %zu",
        pairs.size(), min_pairs_);
      holding_ = true;
    }
    ++pose_held_;
    return;
  }

  std::vector<cv::Vec3d> from;
  std::vector<cv::Vec3d> to;
  from.reserve(pairs.size());
  to.reserve(pairs.size());
  for (const PixelPair & pair : pairs) {
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

  // **`camera_step` is the correction this line waited six days for.** The fit maps
  // the previous frame's rays onto this frame's — it is the motion of the *rays*,
  // which is the camera's motion inverted. Composing the fit itself, as this did
  // from P3 until 2026-09-19, publishes a pose that turns left when the camera pans
  // right; nothing failed, because a TF frame that moves when you pan looks
  // correct. See rgbd_odometry.hpp and test_rgbd_odometry's CameraStep suite.
  //
  // The increment is in the optical frame and the pose is in the body frame, so
  // the basis changes first — and it multiplies on the right, because this is a
  // rotation of the camera relative to where it was, not relative to odom.
  const cv::Matx33d step = change_basis(
    base_from_optical_.rotation(), camera_step(fit.rotation));
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose_ = cv::Affine3d(pose_.rotation() * step, pose_.translation());
  }
  residual_sum_ = residual_sum_.load() + fit.residual_rad;
  ++pose_ok_;
}

void OdometryNode::publish_pose(const rclcpp::Time & stamp)
{
  {
    std::lock_guard<std::mutex> lock(basis_mutex_);
    if (!have_basis_) {return;}
  }

  cv::Affine3d pose;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    pose = pose_;
  }
  const cv::Vec3d t(pose.translation());
  const cv::Vec4d q = quaternion_from_rotation(pose.rotation());

  if (publish_tf_ && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf;
    // The *frame's* stamp, not now(): this transform describes where the camera was
    // when those pixels were captured. A pose stamped with the moment the estimator
    // finished is a pose that claims the camera was somewhere it had already left.
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = t[0];
    tf.transform.translation.y = t[1];
    tf.transform.translation.z = t[2];
    tf.transform.rotation.x = q[0];
    tf.transform.rotation.y = q[1];
    tf.transform.rotation.z = q[2];
    tf.transform.rotation.w = q[3];
    tf_broadcaster_->sendTransform(tf);
  }

  auto odom = std::make_unique<nav_msgs::msg::Odometry>();
  odom->header.stamp = stamp;
  odom->header.frame_id = odom_frame_;
  odom->child_frame_id = base_frame_;
  odom->pose.pose.position.x = t[0];
  odom->pose.pose.position.y = t[1];
  odom->pose.pose.position.z = t[2];
  odom->pose.pose.orientation.x = q[0];
  odom->pose.pose.orientation.y = q[1];
  odom->pose.pose.orientation.z = q[2];
  odom->pose.pose.orientation.w = q[3];
  // A large diagonal, not a -1 and not zeros. This node published `-1` in
  // element 0 from P7 until 2026-09-23, on the belief that nav_msgs documents it
  // the way sensor_msgs/Imu does; it does not, and the result was a matrix that
  // is not positive semidefinite, which RViz reported at the pose rate for the
  // length of every session. See unconstrained_covariance() for the whole story
  // and for why zeros is the worse of the two legal answers.
  //
  // `twist` stays zero-*valued* — nothing here estimates a velocity — and
  // carries the same unconstrained covariance, which is what says so.
  const auto covariance = unconstrained_covariance();
  std::copy(covariance.begin(), covariance.end(), odom->pose.covariance.begin());
  std::copy(covariance.begin(), covariance.end(), odom->twist.covariance.begin());
  odom_pub_->publish(std::move(odom));
}

double OdometryNode::net_displacement()
{
  // **Reported beside the path length because the two answer different
  // questions, and only together do they say anything.** A path length is the sum
  // of every step, so an estimator whose steps are pure noise reports a *large*
  // one — 89.5 m over a 45 s desk sweep, in the run that made P7's estimator get
  // rewritten. The net displacement of that same run was a fraction of it, which
  // is the signature of a random walk rather than of a camera going somewhere.
  // P7's claim is about where the camera got to; the path length is what says how
  // smoothly it got there.
  std::lock_guard<std::mutex> lock(pose_mutex_);
  return cv::norm(cv::Vec3d(pose_.translation()));
}


void OdometryNode::log_stats()
{
  const rclcpp::Time stamp = now();
  const double span_s = (stamp - last_log_).seconds();
  if (span_s <= 0.0) {return;}

  const std::uint64_t keypoints_now = keypoint_frames_.load();
  const std::uint64_t keypoints_delta = keypoints_now - last_keypoint_frames_;
  const std::uint64_t depth_now = depth_frames_.load();
  const std::uint64_t depth_delta = depth_now - last_depth_frames_;
  last_keypoint_frames_ = keypoints_now;
  last_depth_frames_ = depth_now;
  last_log_ = stamp;

  if (keypoints_delta == 0) {
    RCLCPP_WARN(
      get_logger(), "stats no keypoints in %.1fs — is keypoint_node running?", span_s);
    return;
  }

  const std::uint64_t ok = pose_ok_.load();
  const std::uint64_t held = pose_held_.load();
  const std::uint64_t shifted = shift_ok_.load();
  const std::uint64_t shift_held = shift_held_.load();
  const double shifted_d = static_cast<double>(shifted);
  const double depth_total = static_cast<double>(depth_now);
  const double keypoints_total = static_cast<double>(keypoints_now);

  // Poses published per second over *this window*. In sixdof that is one per
  // depth frame, in rotation_only one per keypoints frame.
  const double pose_rate_hz = (regime_ == OdometryRegime::SixDof) ?
    static_cast<double>(depth_delta) / span_s :
    static_cast<double>(keypoints_delta) / span_s;

  // One line, and every number in it is needed to read any of the others.
  //
  // `regime=` is first so that a gate reading this line never has to infer which
  // estimator produced the numbers after it, and `traj=` is the headline of P7:
  // rotation_only reports exactly 0.000 there whatever the camera did.
  RCLCPP_INFO(
    get_logger(),
    "stats regime=%s rate=%.1fHz kp_in=%.1fHz pose_ok=%lu held=%lu reject_rate=%.3f "
    "residual=%.4frad rotation_cost=%.2fms traj=%.4fm net=%.4fm depth_in=%lu "
    "depth_lost=%lu shift_ok=%lu shift_held=%lu landmarks=%.0f shared=%.0f "
    "inliers=%.0f reproj_px=%.3f point_residual=%.4fm depth_scale=%.4f "
    "pose_cost=%.2fms keyframes=%zu keyframe_kb=%.1f kf_on_loss=%lu kf_on_stall=%lu "
    "implausible=%lu",
    regime_name(regime_),
    pose_rate_hz,
    static_cast<double>(keypoints_delta) / span_s,
    static_cast<unsigned long>(ok), static_cast<unsigned long>(held),
    (ok + held > 0) ? static_cast<double>(held) / static_cast<double>(ok + held) : 0.0,
    (ok > 0) ? residual_sum_.load() / static_cast<double>(ok) : 0.0,
    (keypoints_total > 0.0) ? rotation_cost_sum_ms_.load() / keypoints_total : 0.0,
    trajectory_m_.load(),
    net_displacement(),
    static_cast<unsigned long>(depth_now),
    static_cast<unsigned long>(depth_unmatched_.load()),
    static_cast<unsigned long>(shifted), static_cast<unsigned long>(shift_held),
    (depth_total > 0.0) ? landmark_sum_.load() / depth_total : 0.0,
    (depth_total > 0.0) ? pair_sum_.load() / depth_total : 0.0,
    (shifted > 0) ? inlier_sum_.load() / shifted_d : 0.0,
    (shifted > 0) ? reprojection_sum_px_.load() / shifted_d : 0.0,
    (shifted > 0) ? point_residual_sum_.load() / shifted_d : 0.0,
    (shifted > 0) ? scale_sum_.load() / shifted_d : 1.0,
    (depth_total > 0.0) ? pose_cost_sum_ms_.load() / depth_total : 0.0,
    keyframes_.size(),
    static_cast<double>(keyframes_.bytes()) / 1024.0,
    static_cast<unsigned long>(keyframes_on_loss_.load()),
    static_cast<unsigned long>(keyframes_on_stall_.load()),
    static_cast<unsigned long>(implausible_.load()));

  // --- The same numbers, on a topic, for P8's dashboard ------------------------
  //
  // **One row, where this was the second of two until the split.** The node
  // boundary and the stage boundary are the same thing now, so the dashboard's
  // `odometry` row is a node's own account of itself rather than a slice of
  // another node's. See PipelineStats.msg on why `stage` is a string.
  auto os = std::make_unique<pimesh_msgs::msg::PipelineStats>();
  os->header.stamp = stamp;
  os->stage = "odometry";
  const std::uint64_t posed = (regime_ == OdometryRegime::SixDof) ? shifted : ok;
  const std::uint64_t refused = (regime_ == OdometryRegime::SixDof) ? shift_held : held;
  os->rate_hz = static_cast<float>(pose_rate_hz);
  os->latency_ms = static_cast<float>(
    (regime_ == OdometryRegime::SixDof) ?
    ((depth_total > 0.0) ? pose_cost_sum_ms_.load() / depth_total : 0.0) :
    ((keypoints_total > 0.0) ? rotation_cost_sum_ms_.load() / keypoints_total : 0.0));
  os->latency_p95_ms = 0.0F;
  os->frames_in = posed + refused;
  os->frames_out = posed;
  // **Neither counter, and that is the honest answer.** A held pose is not a
  // frame dropped by design and it is not one lost in transport: the frame
  // arrived, was read, and the estimator declined to answer. `detail` carries it
  // rather than either column, because a refusal counted as a drop would make a
  // node that is working correctly on a blank wall look like one losing data.
  os->dropped_by_design = 0;
  os->dropped_in_transport = depth_unmatched_.load();
  char detail[192];
  std::snprintf(
    detail, sizeof(detail),
    "%s held=%lu traj=%.2fm net=%.2fm reproj=%.2fpx shared=%.0f keyframes=%zu",
    regime_name(regime_), static_cast<unsigned long>(refused),
    trajectory_m_.load(), net_displacement(),
    (shifted > 0) ? reprojection_sum_px_.load() / shifted_d : 0.0,
    (depth_total > 0.0) ? pair_sum_.load() / depth_total : 0.0,
    keyframes_.size());
  os->detail = detail;
  stats_pub_->publish(std::move(os));

  // A sixdof run with no depth is a session that will publish no pose at all, and
  // the symptom — a TF tree with a missing edge — sends people to look at the
  // static transforms. Said once per window rather than once, because the cause is
  // usually that depth_node is still loading its model.
  if (regime_ == OdometryRegime::SixDof && depth_now == 0) {
    RCLCPP_WARN(
      get_logger(),
      "regime=sixdof and nothing has arrived on the depth topic — no pose will be "
      "published until it does. Is depth_node running, and does its depth_topic "
      "match this node's?");
  }
}

}  // namespace pimesh_perception

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_perception::OdometryNode)
