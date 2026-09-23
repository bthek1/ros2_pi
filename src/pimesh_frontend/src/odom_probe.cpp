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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "pimesh_core/stats.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

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
  }

  void tick()
  {
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
    if (elapsed < duration_s_ || printed_) {return;}
    printed_ = true;

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
};

}  // namespace pimesh_frontend

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_frontend::OdomProbe)
