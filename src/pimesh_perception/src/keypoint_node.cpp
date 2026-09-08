#include "pimesh_perception/keypoint_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <utility>

#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <Eigen/Geometry>

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
  // Read-only: the tracker's window and feature cap shape state it is already
  // holding, and changing one mid-run would leave that state describing a
  // configuration the node no longer has.
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

// base_link axes from camera_optical_frame axes, as a rotation.
//
// This is the SAME transform frames.launch.py publishes as a static edge
// (roll -pi/2, yaw -pi/2), written out because it is a REP-103 constant rather
// than a configuration: optical is z-forward / x-right / y-down, body is
// x-forward / y-left / z-up. Looking it up through a tf2 listener would add a
// second failure mode — a node that estimates nothing until TF is ready — for
// a matrix that cannot change without breaking the whole convention.
const Eigen::Matrix3d & base_from_optical()
{
  static const Eigen::Matrix3d m = (Eigen::Matrix3d() <<
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0).finished();
  return m;
}

}  // namespace

KeypointNode::KeypointNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("keypoint_node", options)
{
  OrbTracker::Options orb;
  orb.max_features = declare_parameter<int>(
    "max_features", 500,
    describe_int(
      "ORB feature cap per frame. Cost is roughly linear in this; 500 is what "
      "the predecessor measured at ~14 ms/frame in Python, decode included.",
      50, 4000));
  orb.pyramid_levels = declare_parameter<int>(
    "pyramid_levels", 4,
    describe_int(
      "image-pyramid levels ORB detects on. The cheapest real knob in this "
      "stage: 8 (OpenCV's default) cost 9.6 ms/frame at 1280x720 on this box, "
      "4 cost 6.8 ms. The extra levels buy scale invariance, and between two "
      "frames 30 ms apart there is almost no scale change to be invariant to. "
      "Raise it when something matches across a WIDE baseline — P7's "
      "keyframes, or loop closure.", 1, 16));
  orb.scale_factor = declare_parameter<double>(
    "scale_factor", 1.2,
    describe_double("scale step between pyramid levels", 1.05, 4.0));
  orb.window = declare_parameter<int>(
    "match_window", 10,
    describe_int(
      "how many recent frames the POOLED match sees. 1 is consecutive-only "
      "and loses ~25% of keypoints to detection churn; 10 held ~90% matched. "
      "Does not affect the odometry, which always matches strictly against "
      "the previous frame.", 1, 60));
  orb.max_distance = declare_parameter<int>(
    "match_max_distance", 64,
    describe_int(
      "Hamming bits of 256 above which a match is rejected as a "
      "similar-looking corner rather than the same physical point.", 1, 256));

  gates_.min_pairs = static_cast<std::size_t>(declare_parameter<int>(
    "min_matched_pairs", 8,
    describe_int(
      "below this many strict pairs the rotation fit is fitting noise, and "
      "the node HOLDS the last pose instead of publishing a guess.", 3, 500)));
  gates_.max_residual_rad = declare_parameter<double>(
    "max_residual_rad", 0.03,
    describe_double(
      "mean per-pair ray residual above which the matches do not describe one "
      "rotation — a moving object in frame, or a burst of false matches.",
      0.0, 1.0));

  const auto stats_period_s = declare_parameter<double>(
    "stats_period_s", 1.0, describe("how often to publish /pipeline/stats"));
  stale_after_s_ = declare_parameter<double>(
    "stale_after_s", 2.0,
    describe(
      "no frame for this long marks the stage stale. Measured on RECEIPT "
      "time, never on header.stamp."));
  const auto preview_hz = declare_parameter<double>(
    "preview_rate_hz", 10.0,
    describe_double(
      "annotated preview rate. Deliberately far below the frame rate: the "
      "preview LEAVES the container, so it is drawn, JPEG-encoded and "
      "serialised, and doing that at 30-60 Hz would spend most of this "
      "stage's budget on a picture for humans. 0 disables it.", 0.0, 120.0));
  const auto jpeg_quality = declare_parameter<int>(
    "jpeg_quality", 80, describe_int("preview JPEG quality", 1, 100));
  publish_tf_ = declare_parameter<bool>(
    "publish_tf", true,
    describe(
      "own the odom -> base_link edge. EXACTLY ONE node may: two publishers "
      "on one edge is the failure that smears a mesh and appears in no log."));
  odom_frame_ = declare_parameter<std::string>(
    "odom_frame", "odom", describe("parent frame of the estimated edge"));
  base_frame_ = declare_parameter<std::string>(
    "base_frame", "base_link", describe("child frame of the estimated edge"));

  preview_period_s_ = preview_hz > 0.0 ? 1.0 / preview_hz : 0.0;
  jpeg_params_ = {cv::IMWRITE_JPEG_QUALITY, static_cast<int>(jpeg_quality)};
  tracker_ = std::make_unique<OrbTracker>(orb);

  if (!options.use_intra_process_comms()) {
    RCLCPP_WARN(
      get_logger(),
      "intra-process comms OFF — this node is COPYING every 2.7 MB frame out "
      "of decode_node. Expected only when running standalone.");
  }

  // Must match decode_node's publisher exactly. A QoS mismatch on an
  // intra-process pair produces no warning at all — just a topic with no
  // subscribers — so this is one of the few places where "looks fine" and
  // "works" come apart silently.
  const auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

  keypoints_pub_ = create_publisher<pimesh_msgs::msg::Keypoints>("keypoints", image_qos);
  preview_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    "keypoints/image/compressed", image_qos);
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "camera/orientation", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  stats_pub_ = create_publisher<pimesh_msgs::msg::PipelineStats>(
    "/pipeline/stats", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  // The intrinsics come from the Pi over the LAN. This is a second network
  // subscriber, which the one-reader rule would normally forbid — but that
  // rule is about megabyte-class frames saturating Wi-Fi, and CameraInfo is
  // ~500 bytes. transient_local so a late-joining container still gets the
  // latched value instead of waiting for the next publish.
  info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&KeypointNode::on_camera_info, this, std::placeholders::_1));

  reset_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/reset",
    std::bind(
      &KeypointNode::on_reset, this, std::placeholders::_1, std::placeholders::_2));

  window_start_ = std::chrono::steady_clock::now();
  last_frame_ = window_start_;
  last_preview_ = window_start_;

  worker_ = std::thread(&KeypointNode::work_loop, this);
  if (preview_period_s_ > 0.0) {
    preview_worker_ = std::thread(&KeypointNode::preview_loop, this);
  }

  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "rgb/image", image_qos,
    std::bind(&KeypointNode::on_frame, this, std::placeholders::_1));

  stats_timer_ = create_wall_timer(
    std::chrono::duration<double>(stats_period_s),
    std::bind(&KeypointNode::publish_stats, this));

  RCLCPP_INFO(
    get_logger(), "ORB %d features over %d pyramid levels, window %d, "
    "hamming <= %d; %s -> %s",
    orb.max_features, orb.pyramid_levels, orb.window, orb.max_distance,
    image_sub_->get_topic_name(), keypoints_pub_->get_topic_name());
}

KeypointNode::~KeypointNode()
{
  running_.store(false);
  mailbox_.close();
  preview_mailbox_.close();
  if (worker_.joinable()) {
    worker_.join();
  }
  if (preview_worker_.joinable()) {
    preview_worker_.join();
  }
}

void KeypointNode::on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
{
  CameraMatrix k{};
  std::copy(msg->k.begin(), msg->k.end(), k.begin());
  const bool ok = is_calibrated(k);

  std::lock_guard<std::mutex> lock(intrinsics_mutex_);
  k_ = k;
  if (ok && !calibrated_) {
    RCLCPP_INFO(
      get_logger(), "intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f — odometry enabled",
      k[0], k[4], k[2], k[5]);
  }
  calibrated_ = ok;
  if (!ok && !warned_uncalibrated_) {
    warned_uncalibrated_ = true;
    // Zeros are the honest signal from an uncalibrated camera, and inventing a
    // focal length here would turn "no pose" into "a confident wrong pose".
    // Detection, matching and the preview all still work; only the odometer
    // is off.
    RCLCPP_WARN(
      get_logger(),
      "CameraInfo has K all zeros — the camera is UNCALIBRATED. Detecting and "
      "matching only; no rotation estimate, and odom -> base_link stays "
      "identity. Run camera_calibration and give camera_node the file.");
  }
}

void KeypointNode::on_reset(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  // Flagged, not done here: the orientation and the tracker belong to the
  // worker thread, and reaching into them from a service callback would be a
  // data race on every field this node has.
  reset_requested_.store(true);
  response->success = true;
  response->message = "orientation and tracker will reset on the next frame";
}

void KeypointNode::on_frame(ImageConstPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    last_frame_ = std::chrono::steady_clock::now();
    ever_received_ = true;
  }
  mailbox_.put(std::move(msg));
}

void KeypointNode::work_loop()
{
  using clock = std::chrono::steady_clock;

  ImageConstPtr frame;
  while (running_.load() && mailbox_.take(frame)) {
    const auto t0 = clock::now();

    if (frame->encoding != "bgr8" || frame->data.size() <
      static_cast<std::size_t>(frame->step) * frame->height)
    {
      undecodable_.fetch_add(1);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "unusable frame: encoding '%s', %zu bytes for %ux%u",
        frame->encoding.c_str(), frame->data.size(), frame->width, frame->height);
      continue;
    }

    if (reset_requested_.exchange(false)) {
      tracker_->reset();
      orientation_ = Eigen::Matrix3d::Identity();
      RCLCPP_INFO(get_logger(), "reset: orientation is identity, tracker cleared");
    }

    // A cv::Mat HEADER over the message's bytes — no copy. With intra-process
    // comms on, those bytes are the ones decode_node wrote, so this stage
    // reads the original buffer rather than a duplicate of it. const_cast is
    // safe because nothing below writes through `bgr`.
    const cv::Mat bgr(
      static_cast<int>(frame->height), static_cast<int>(frame->width), CV_8UC3,
      const_cast<uint8_t *>(frame->data.data()), frame->step);

    // ORB detects on intensity. This conversion is the one unavoidable
    // per-frame copy in the stage, ~0.4 ms for 1280x720.
    cv::cvtColor(bgr, gray_, cv::COLOR_BGR2GRAY);

    const TrackedFrame tracked = tracker_->track(gray_);
    const RotationEstimate estimate = update_orientation(tracked);
    (void)estimate;

    // Zero-copy out: a unique_ptr lets rclcpp hand the SAME arrays to every
    // component downstream. 500 features is ~16 kB of descriptors alone, and
    // P7 will want all of it.
    auto out = std::make_unique<pimesh_msgs::msg::Keypoints>();
    out->header = frame->header;      // the CAPTURE stamp, carried through
    out->image_width = frame->width;
    out->image_height = frame->height;
    out->count = static_cast<uint32_t>(tracked.count());
    out->x.resize(tracked.count());
    out->y.resize(tracked.count());
    out->size.resize(tracked.count());
    out->angle.resize(tracked.count());
    out->response.resize(tracked.count());
    for (std::size_t i = 0; i < tracked.count(); ++i) {
      const auto & kp = tracked.keypoints[i];
      out->x[i] = kp.pt.x;
      out->y[i] = kp.pt.y;
      out->size[i] = kp.size;
      out->angle[i] = kp.angle;
      out->response[i] = kp.response;
    }
    out->match_index = tracked.match_index;
    out->track_id = tracked.track_id;
    out->descriptor_size = static_cast<uint16_t>(
      tracked.descriptors.empty() ? 0 : tracked.descriptors.cols);
    if (!tracked.descriptors.empty()) {
      out->descriptors.resize(
        static_cast<std::size_t>(tracked.descriptors.rows) *
        static_cast<std::size_t>(tracked.descriptors.cols));
      // The descriptor block is contiguous as detectAndCompute returns it, so
      // one memcpy rather than a row loop.
      std::memcpy(
        out->descriptors.data(), tracked.descriptors.data, out->descriptors.size());
    }
    keypoints_pub_->publish(std::move(out));

    enqueue_preview(frame, tracked);
    publish_pose(frame->header);

    const double ms = std::chrono::duration<double, std::milli>(clock::now() - t0).count();
    processed_.fetch_add(1);
    {
      std::lock_guard<std::mutex> lock(window_mutex_);
      window_ms_.push_back(ms);
      matched_fractions_.push_back(tracked.matched_fraction());
    }
  }
}

RotationEstimate KeypointNode::update_orientation(const TrackedFrame & tracked)
{
  CameraMatrix k{};
  bool calibrated = false;
  {
    std::lock_guard<std::mutex> lock(intrinsics_mutex_);
    k = k_;
    calibrated = calibrated_;
  }

  RotationEstimate estimate;
  if (!calibrated) {
    estimate.reject = RotationReject::kNoIntrinsics;
    reject_no_intrinsics_.fetch_add(1);
    pose_rejected_.fetch_add(1);
    return estimate;
  }

  estimate = estimate_rotation(
    rays_from_pixels(tracked.prev_matched, k),
    rays_from_pixels(tracked.curr_matched, k),
    gates_);

  if (!estimate.ok) {
    // HOLD the last pose. Publishing a guess would be worse than publishing
    // nothing new: fusion integrates depth at whatever pose it is handed, and
    // one bad rotation smears the room permanently.
    pose_rejected_.fetch_add(1);
    switch (estimate.reject) {
      case RotationReject::kTooFewPairs: reject_few_.fetch_add(1); break;
      case RotationReject::kResidual: reject_residual_.fetch_add(1); break;
      default: break;
    }
    RCLCPP_DEBUG(
      get_logger(), "pose gate rejected (%s): %zu pairs, mean residual %.4f rad",
      to_string(estimate.reject), estimate.pairs_used, estimate.mean_residual_rad);
    return estimate;
  }

  // estimate_rotation maps the PREVIOUS frame's rays onto this one's. The
  // camera itself turned by the inverse of that — a point drifting left across
  // the image means the camera swung right — so the world-side composition
  // multiplies by R transposed.
  orientation_ = orthonormalize(orientation_ * estimate.rotation.transpose());
  pose_ok_.fetch_add(1);
  return estimate;
}

void KeypointNode::publish_pose(const std_msgs::msg::Header & header)
{
  // Conjugate the optical-axes orientation into base_link axes: R_base =
  // B * R_optical * B^T, where B is the constant above. Rotating a rotation
  // between frames is a conjugation, not a multiplication — getting this wrong
  // produces a pose that looks plausible and turns about the wrong axis.
  const Eigen::Matrix3d base =
    base_from_optical() * orientation_ * base_from_optical().transpose();
  const Eigen::Quaterniond q(base);

  geometry_msgs::msg::PoseStamped pose;
  // The INPUT frame's stamp, not now(): this orientation describes the moment
  // that frame was captured, and P7 will need to align it with a depth map
  // taken at the same instant. Our own capture stamps are trustworthy —
  // camera_node stamps at VIDIOC_DQBUF from CLOCK_MONOTONIC.
  pose.header.stamp = header.stamp;
  pose.header.frame_id = odom_frame_;
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  pose_pub_->publish(pose);

  if (!tf_broadcaster_) {
    return;
  }
  // The same estimate again, as a transform. Translation stays exactly zero —
  // this odometer measures orientation only, and a fabricated translation
  // would be indistinguishable from a measured one downstream.
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = header.stamp;
  tf.header.frame_id = odom_frame_;
  tf.child_frame_id = base_frame_;
  tf.transform.rotation = pose.pose.orientation;
  tf_broadcaster_->sendTransform(tf);
}

void KeypointNode::enqueue_preview(
  const ImageConstPtr & frame, const TrackedFrame & tracked)
{
  if (preview_period_s_ <= 0.0) {
    return;
  }
  // Rate-limit HERE, on the tracking thread, rather than in the preview
  // thread: the split of the keypoints into two colour groups is a copy, and
  // there is no reason to pay it for a frame nobody will draw.
  const auto now_steady = std::chrono::steady_clock::now();
  if (std::chrono::duration<double>(now_steady - last_preview_).count() < preview_period_s_) {
    return;
  }
  last_preview_ = now_steady;

  // Yellow = new this frame, green = seen in the pooled window. Split now,
  // drawn later — the classification belongs to this frame's tracker state,
  // which the next frame will overwrite.
  PreviewJob job;
  job.frame = frame;
  for (std::size_t i = 0; i < tracked.count(); ++i) {
    (tracked.recently_seen[i] ? job.matched : job.fresh).push_back(tracked.keypoints[i]);
  }
  preview_mailbox_.put(std::move(job));
}

void KeypointNode::preview_loop()
{
  PreviewJob job;
  cv::Mat annotated;
  while (running_.load() && preview_mailbox_.take(job)) {
    const auto & frame = *job.frame;
    // The same zero-copy view the tracker used. This thread only reads it, and
    // the shared_ptr in the job is what keeps the buffer alive.
    const cv::Mat bgr(
      static_cast<int>(frame.height), static_cast<int>(frame.width), CV_8UC3,
      const_cast<uint8_t *>(frame.data.data()), frame.step);

    // Green drawn second so a tracked point wins any overlap — the eye should
    // be drawn to what is holding, not to what is churning.
    // DRAW_RICH_KEYPOINTS renders each feature's size and orientation, so the
    // overlay shows what ORB actually found rather than just where it looked.
    cv::drawKeypoints(
      bgr, job.fresh, annotated, cv::Scalar(0, 255, 255),
      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::drawKeypoints(
      annotated, job.matched, annotated, cv::Scalar(0, 255, 0),
      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    auto preview = std::make_unique<sensor_msgs::msg::CompressedImage>();
    preview->header = frame.header;
    preview->format = "jpeg";
    cv::imencode(".jpg", annotated, preview->data, jpeg_params_);
    preview_pub_->publish(std::move(preview));
    previews_.fetch_add(1);
  }
}

void KeypointNode::publish_stats()
{
  std::vector<double> window;
  std::vector<double> fractions;
  double elapsed = 0.0;
  bool stale = true;
  {
    std::lock_guard<std::mutex> lock(window_mutex_);
    const auto now_steady = std::chrono::steady_clock::now();
    window.swap(window_ms_);
    fractions.swap(matched_fractions_);
    elapsed = std::chrono::duration<double>(now_steady - window_start_).count();
    window_start_ = now_steady;
    const double since_frame = std::chrono::duration<double>(now_steady - last_frame_).count();
    stale = !ever_received_ || since_frame > stale_after_s_;
  }

  const double total_ms = std::accumulate(window.begin(), window.end(), 0.0);
  const double matched = fractions.empty() ?
    0.0 :
    std::accumulate(fractions.begin(), fractions.end(), 0.0) /
    static_cast<double>(fractions.size());

  bool calibrated = false;
  {
    std::lock_guard<std::mutex> lock(intrinsics_mutex_);
    calibrated = calibrated_;
  }

  pimesh_msgs::msg::PipelineStats stats;
  stats.header.stamp = now();
  stats.header.frame_id = "camera_optical_frame";
  stats.stage = "keypoints";
  stats.rate_hz = elapsed > 0.0 ?
    static_cast<float>(static_cast<double>(window.size()) / elapsed) : 0.0f;
  stats.latency_ms = window.empty() ?
    0.0f : static_cast<float>(total_ms / static_cast<double>(window.size()));
  stats.latency_p95_ms = static_cast<float>(percentile(window, 0.95));
  stats.processed = processed_.load();
  stats.dropped_mailbox = mailbox_.dropped();
  stats.dropped_transport = undecodable_.load();
  stats.stale = stale;

  // key=value, because a gate has to parse this. `detail` is free-form by
  // design, but free-form and unparseable are not the same thing.
  char detail[320];
  std::snprintf(
    detail, sizeof(detail),
    "regime=%s matched=%.3f features=%d levels=%d window=%d previews=%lu "
    "pose_ok=%lu pose_rej=%lu rej_few=%lu rej_resid=%lu rej_nointr=%lu",
    calibrated ? "rotation_only" : "detect_only",
    matched, tracker_->options().max_features, tracker_->options().pyramid_levels,
    tracker_->options().window,
    static_cast<unsigned long>(previews_.load()),
    static_cast<unsigned long>(pose_ok_.load()),
    static_cast<unsigned long>(pose_rejected_.load()),
    static_cast<unsigned long>(reject_few_.load()),
    static_cast<unsigned long>(reject_residual_.load()),
    static_cast<unsigned long>(reject_no_intrinsics_.load()));
  stats.detail = detail;

  stats_pub_->publish(stats);
}

}  // namespace pimesh_perception

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_perception::KeypointNode)
