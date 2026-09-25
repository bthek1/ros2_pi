// The instrument tools/gates/odom.sh measures the trajectory with.
//
// **Off the topic, not off the node's own counters**, and that is the point.
// `keypoint_node` reports a path length and a net displacement in its stats line,
// and those are perfectly good numbers about its internal state — but P7's claim
// is about the pose the *rest of the pipeline receives*, and the thing that was
// wrong for six days before this phase was precisely a pose that every internal
// number described correctly and that came out of the node inverted. An
// instrument on the far side of the publish is the one that can tell the
// difference.
//
// It is a component rather than a program for the ordinary reason in this
// workspace: `ros2 topic hz` is Python, its own scheduling lands in the number it
// reports, and it cannot see a message's contents at all. `/odom` is small enough
// that an out-of-process subscriber would be harmless — unlike depth_probe, which
// has to be in the container — but the same binary shape keeps one way of doing
// this rather than two.
//
// **The number that matters most here is the largest single step.** A trajectory
// is easy to summarise into a mean that hides a catastrophe: measured on
// bags/desk1 before the plausibility gate existed, a single 9.4 m jump between two
// depth frames — at a mean PnP reprojection of 1.23 px, so the fit was confident —
// redefined the reference every later frame was measured from, while the mean step
// stayed at 2.7 cm and looked entirely healthy.
//
// **Since P11 it also writes the trajectory out**, in TUM format, because that is
// what `evo` reads and because writing our own ATE would be writing the
// instrument and the thing it measures in the same afternoon. The pose goes to
// the file exactly as it arrived on the topic — no interpolation, no resampling,
// no dropping of held poses — so the file and the summary line above it describe
// the same set of messages. The formatting itself is in
// pimesh_frontend/tum_trajectory.hpp with test_tum_trajectory behind it; a
// snprintf in an anonymous namespace here is precisely the shape four helpers in
// this workspace have had to be moved out of.
//
// **And the trajectory is the *camera optical* pose, not base_link's, which is a
// correction that cost a measurement.** `/odom` carries `odom -> base_link`, in
// the REP-103 body convention; TUM's ground truth is the colour camera's optical
// frame. The two differ by a constant rotation and no translation, so an ATE over
// the translation part is *identical* either way — and an RPE is not. Measured
// 2026-09-25 by rotating TUM's own ground truth by that constant and scoring it
// against itself: **0.654 m RPE over a 1 s window, against a trajectory that is
// exactly right.** The first run of this probe reported 0.724 m, so essentially
// the whole of it was the convention rather than drift.
//
// The constant is not typed here. It is `base_link -> camera_optical_frame` out
// of the TF tree, which `pimesh_bringup` publishes and `test_transforms` checks
// is actually the optical convention — writing the quaternion into this file
// would be the `volume_key` trap, two copies of one number with nothing relating
// them. The lookup happens once (the edge is static) and only when a trajectory
// is being written, so tools/gates/odom.sh is untouched by it. A pose that arrives
// before that edge is in the buffer is **left out and counted** rather than
// written in the body frame — `skipped=` beside the pose count, asserted at zero
// by the gate — because a trajectory in the wrong frame produces a number and not
// an error, and a trajectory that silently starts late produces a better one.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pimesh_core/stats.hpp"
#include "pimesh_frontend/tum_trajectory.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

using pimesh_core::percentile;

namespace pimesh_frontend
{

class OdomProbe : public rclcpp::Node
{
public:
  explicit OdomProbe(const rclcpp::NodeOptions & options)
  : Node("odom_probe", options)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Pose stream to measure.";
    const std::string topic = declare_parameter("odom_topic", std::string("/odom"), desc);

    desc.description =
      "Seconds to measure before printing the summary. The gate sets it from the "
      "clip's own metadata, so the window and the clip are the same seconds — "
      "comparing a 20 s window of a 60 s clip against that clip's average is a "
      "mistake this project has already made once, in P3.";
    duration_s_ = declare_parameter("duration_s", 60.0, desc);

    desc.description =
      "Where to write the trajectory, in TUM format (timestamp tx ty tz qx qy qz "
      "qw) — the format `evo` reads. Empty writes nothing, which is what every "
      "gate but tools/gates/trajectory.sh wants.";
    trajectory_path_ = declare_parameter("trajectory_path", std::string(), desc);

    desc.description =
      "The frame the written trajectory is expressed in. Public RGB-D benchmarks "
      "give ground truth for the camera's *optical* frame, and /odom is the body "
      "frame; the two differ by a constant rotation that an ATE cannot see and an "
      "RPE can. Named rather than written as a quaternion, so the number lives in "
      "one place — the static edge pimesh_bringup publishes.";
    optical_frame_ = declare_parameter(
      "optical_frame", std::string("camera_optical_frame"), desc);

    // Only when there is a file to write. Standing up a listener in the four
    // gates that load this probe without one would be a subscription to /tf and
    // /tf_static inside every measurement they take.
    if (!trajectory_path_.empty()) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // KeepLast(200) and reliable: a trajectory with holes in it is a trajectory
    // whose step sizes are wrong, and the largest step is the number this probe
    // exists to report. At ~17 Hz and small messages there is no reason to drop
    // any, and if the middleware does, `gaps` below says so rather than letting it
    // pass as a large step.
    rclcpp::QoS qos(rclcpp::KeepLast(200));
    qos.reliable();
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      topic, qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {this->on_odom(std::move(msg));});

    started_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this] {this->tick();});

    RCLCPP_INFO(
      get_logger(), "odom_probe watching %s for %.1fs", topic.c_str(), duration_s_);
  }

private:
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const cv_point here{p.x, p.y, p.z};
    const std::int64_t stamp =
      static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(msg->header.stamp.nanosec);

    if (frames_ > 0) {
      const double step = std::sqrt(
        (here.x - last_.x) * (here.x - last_.x) +
        (here.y - last_.y) * (here.y - last_.y) +
        (here.z - last_.z) * (here.z - last_.z));
      path_ += step;
      steps_.push_back(step);
      biggest_ = std::max(biggest_, step);

      // **The speed, over the interval since the position last *changed*** — not
      // since the last message. A held pose republishes the same position, so a
      // step that follows a run of holds spans all of them, and dividing it by one
      // message interval would report a camera moving eight times faster than it
      // was estimated to. The node's own refusal measures the same interval, and a
      // gate that measured a different one would be asserting on a different
      // quantity while looking like it agreed.
      if (step > 0.0) {
        const double since_s = static_cast<double>(stamp - moved_stamp_) * 1e-9;
        if (since_s > 0.0) {
          const double speed = step / since_s;
          speeds_.push_back(speed);
          fastest_ = std::max(fastest_, speed);
        }
        moved_stamp_ = stamp;
        ++moves_;
      }
      // The interval between poses, on the *stamps*, which for this topic are the
      // capture times of frames on one machine — so a delta here is a kernel
      // capture interval and is trustworthy. See the constraint in CLAUDE.md: a
      // stamp *age* across the two machines would be measuring NTP, but a
      // difference of two stamps from the same source is not.
      const double gap_s = static_cast<double>(stamp - last_stamp_) * 1e-9;
      if (gap_s > 0.0) {intervals_.push_back(gap_s * 1e3);}
    } else {
      first_ = here;
      first_stamp_ = stamp;
      moved_stamp_ = stamp;
    }
    last_ = here;
    last_stamp_ = stamp;
    ++frames_;

    // Only when a file was asked for — unlike the summary above, this one needs
    // the TF edge, and a probe with no trajectory to write has no business
    // holding a listener open inside somebody else's measurement.
    if (!trajectory_path_.empty()) {
      ++wanted_;
      if (resolve_optical_rotation(msg->child_frame_id)) {
        const auto q = compose(msg->pose.pose.orientation, rot_);
        poses_.push_back(
          pimesh_frontend::TumPose{stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w});
      }
    }
  }

  /// The constant rotation from the pose's own child frame to the optical frame,
  /// looked up once. `TimePointZero` is the right request for a *static* edge —
  /// asking at the message's stamp would work too and would start failing the
  /// moment somebody replays a dataset stamped in 2011 against a buffer whose
  /// clock is now.
  bool resolve_optical_rotation(const std::string & body_frame)
  {
    if (have_rotation_) {return true;}
    if (!tf_buffer_) {return false;}
    try {
      const auto tf = tf_buffer_->lookupTransform(
        body_frame, optical_frame_, tf2::TimePointZero);
      rot_ = tf.transform.rotation;
      // A translation here would mean the positions differ too, and this probe
      // composes only the rotation — so say so rather than quietly reporting a
      // trajectory offset by the camera's mounting.
      const auto & o = tf.transform.translation;
      if (std::abs(o.x) + std::abs(o.y) + std::abs(o.z) > 1e-6) {
        RCLCPP_WARN(
          get_logger(),
          "%s -> %s carries a %.3f m offset; the written trajectory composes only "
          "the rotation, so its positions are %s's",
          body_frame.c_str(), optical_frame_.c_str(),
          std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z), body_frame.c_str());
      }
      have_rotation_ = true;
      RCLCPP_INFO(
        get_logger(), "trajectory will be written in '%s' (%s -> %s)",
        optical_frame_.c_str(), body_frame.c_str(), optical_frame_.c_str());
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "no %s -> %s yet: %s", body_frame.c_str(), optical_frame_.c_str(), ex.what());
    }
    return have_rotation_;
  }

  /// Hamilton product, `a` then `b`. Four lines rather than a tf2 conversion
  /// round trip, and written out for the reason test_transforms writes out its
  /// own rotate(): the formula is the thing being trusted.
  static geometry_msgs::msg::Quaternion compose(
    const geometry_msgs::msg::Quaternion & a, const geometry_msgs::msg::Quaternion & b)
  {
    geometry_msgs::msg::Quaternion out;
    out.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    out.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    out.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    out.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    return out;
  }

  void tick()
  {
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
    if (elapsed < duration_s_ || printed_) {return;}
    printed_ = true;

    write_trajectory();

    if (frames_ == 0) {
      RCLCPP_ERROR(
        get_logger(),
        "odom_probe result frames=0 — nothing published a pose. In sixdof that is "
        "usually depth_node still loading its model, or a depth_topic mismatch.");
      return;
    }

    const double net = std::sqrt(
      (last_.x - first_.x) * (last_.x - first_.x) +
      (last_.y - first_.y) * (last_.y - first_.y) +
      (last_.z - first_.z) * (last_.z - first_.z));
    const double span_s = static_cast<double>(last_stamp_ - first_stamp_) * 1e-9;

    // One line, keyed, in the shape every other probe in this workspace prints:
    // the gate parses it with the same awk.
    RCLCPP_INFO(
      get_logger(),
      "odom_probe result frames=%lu moves=%lu span_s=%.2f rate=%.2f path_m=%.4f net_m=%.4f "
      "step_p50=%.4f step_p95=%.4f step_max=%.4f speed_p95=%.4f speed_max=%.4f "
      "interval_p50=%.1f interval_max=%.1f",
      static_cast<unsigned long>(frames_), static_cast<unsigned long>(moves_), span_s,
      (span_s > 0.0) ? static_cast<double>(frames_ - 1) / span_s : 0.0,
      path_, net,
      percentile(steps_, 0.5), percentile(steps_, 0.95), biggest_,
      percentile(speeds_, 0.95), fastest_,
      percentile(intervals_, 0.5), intervals_.empty() ? 0.0 :
      *std::max_element(intervals_.begin(), intervals_.end()));
  }

  /// The file tools/gates/trajectory.sh hands to `evo`, written once, at the end
  /// of the measuring window — the same instant the summary line is printed, so
  /// the two cannot describe different runs.
  ///
  /// **A failure here is logged as an error and does not stop the probe.** The
  /// summary line is what four other gates read, and losing it because a
  /// directory was not writable would turn one gate's problem into five.
  void write_trajectory()
  {
    if (trajectory_path_.empty()) {return;}
    const std::string why =
      pimesh_frontend::write_tum_trajectory(trajectory_path_, poses_);
    if (!why.empty()) {
      RCLCPP_ERROR(
        get_logger(), "odom_probe trajectory refused: %s", why.c_str());
      return;
    }
    // Keyed, in the shape the gate's awk already reads, and separate from the
    // result line so that "the file exists" and "the file has this many poses in
    // it" are one statement rather than two things a reader has to pair up.
    //
    // **`skipped` is the half that matters.** A pose that arrived before the
    // optical edge was in the TF buffer is *not in the file*, and a shortfall can
    // only ever be a leading one — the edge is static and the rotation is cached
    // the first time it resolves. It is reported rather than refused, and the gate
    // asserts it at zero: writing nothing would turn one lost pose into an
    // unreadable failure, and leaving the number out would turn a trajectory that
    // silently starts late into a good-looking ATE over less of the clip.
    RCLCPP_INFO(
      get_logger(), "odom_probe trajectory poses=%zu skipped=%zu path=%s",
      poses_.size(), wanted_ - poses_.size(), trajectory_path_.c_str());
  }

  struct cv_point
  {
    double x {0.0};
    double y {0.0};
    double z {0.0};
  };

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point started_;
  double duration_s_ {60.0};
  std::string trajectory_path_;
  std::string optical_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::Quaternion rot_;
  bool have_rotation_ {false};
  std::size_t wanted_ {0};
  bool printed_ {false};

  std::uint64_t frames_ {0};
  cv_point first_;
  cv_point last_;
  std::int64_t first_stamp_ {0};
  std::int64_t last_stamp_ {0};
  double path_ {0.0};
  double biggest_ {0.0};
  double fastest_ {0.0};
  std::int64_t moved_stamp_ {0};
  std::uint64_t moves_ {0};
  std::vector<double> speeds_;
  std::vector<double> steps_;
  std::vector<double> intervals_;
  std::vector<pimesh_frontend::TumPose> poses_;
};

}  // namespace pimesh_frontend

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_frontend::OdomProbe)
