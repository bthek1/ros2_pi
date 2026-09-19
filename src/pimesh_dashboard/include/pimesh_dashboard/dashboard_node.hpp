#ifndef PIMESH_DASHBOARD__DASHBOARD_NODE_HPP_
#define PIMESH_DASHBOARD__DASHBOARD_NODE_HPP_

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "pimesh_dashboard/web_server.hpp"
#include "pimesh_msgs/msg/pipeline_stats.hpp"
#include "pimesh_msgs/srv/reset_map.hpp"
#include "pimesh_msgs/srv/save_mesh.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace pimesh_dashboard
{

/// One browser tab that shows the pipeline, and cannot slow it down.
///
/// **It runs in its own process, not in the container, and that is a requirement
/// rather than a convenience.** Every other dev-box node shares one process
/// precisely so a 2.7 MB frame is handed on as a pointer; this one subscribes to
/// three small topics and holds a socket open to something outside the machine's
/// control. `docs/info/dashboard.md` states the rule as *it must be able to die* —
/// kill the browser, kill this node, and the mesh does not notice. A component in
/// the container could not make that promise: a crash here would take the TSDF
/// with it.
///
/// It is still registered as a component, because a node that only works
/// standalone is a bug in this workspace and the registration costs nothing.
///
/// **The dashboard never computes.** Every number it sends is a number some node
/// measured about itself and published on `/pipeline/stats`; this node aggregates
/// and forwards, and does no arithmetic on any of it beyond turning a table into
/// JSON. That is what makes the panel and `ros2 topic echo` unable to disagree —
/// and it is why a stage that stops publishing shows **STALE** rather than its
/// last value: the alternative is a panel that is confidently describing a
/// pipeline that stopped a minute ago.
///
/// **Staleness is measured on receipt, never on `header.stamp`.** The capture row
/// is stamped on the Pi and read here, and the gap between the two clocks is
/// NTP's business — it measured +8 ms and −19 ms an hour apart on 2026-09-09 with
/// nothing changed. A staleness rule on stamps would be a rule about time
/// synchronisation wearing a pipeline-health costume.
class DashboardNode : public rclcpp::Node
{
public:
  explicit DashboardNode(const rclcpp::NodeOptions & options);
  ~DashboardNode() override;

private:
  /// One stage's last word, and when it reached us.
  struct Row
  {
    pimesh_msgs::msg::PipelineStats stats;
    std::chrono::steady_clock::time_point received;
  };

  void on_stats(pimesh_msgs::msg::PipelineStats::ConstSharedPtr msg);
  void on_rgb(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void on_depth(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_mesh(visualization_msgs::msg::Marker::ConstSharedPtr msg);
  void tick();

  /// True if enough time has passed since `last` for a channel capped at
  /// `period`, and stamps `last` when it is. The one piece of pacing that lives
  /// on the subscription side, because a JPEG that is not going to be sent should
  /// not be copied first.
  bool due(std::chrono::steady_clock::time_point & last, double period_s);

  std::string stats_json();
  std::string pose_json();
  /// The page's two buttons, called on the server's thread. They call the same
  /// services RViz would; this node implements neither.
  std::string run_action(const std::string & name);

  rclcpp::Subscription<pimesh_msgs::msg::PipelineStats>::SharedPtr stats_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr depth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr mesh_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<pimesh_msgs::srv::SaveMesh>::SharedPtr save_client_;
  rclcpp::Client<pimesh_msgs::srv::ResetMap>::SharedPtr reset_client_;
  std::string save_dir_;
  double action_timeout_s_ {10.0};

  std::unique_ptr<WebServer> server_;

  std::mutex mutex_;
  /// Keyed by stage name, so a stage this node has never heard of appears as its
  /// own row rather than as UNKNOWN — which is the reason `PipelineStats.stage`
  /// is a string and not an enum. Adding a stage upstream must not need a change
  /// here.
  std::map<std::string, Row> rows_;
  nav_msgs::msg::Odometry last_odom_;
  bool have_odom_ {false};
  std::chrono::steady_clock::time_point odom_received_;
  /// The trajectory tail the page draws. A ring of positions, not the whole
  /// history: the point of the trail is the shape of recent motion, and an
  /// unbounded one would be a memory leak with a picture attached.
  std::vector<std::array<float, 3>> trail_;
  std::size_t trail_limit_ {600};

  std::chrono::steady_clock::time_point last_rgb_;
  std::chrono::steady_clock::time_point last_depth_;
  std::chrono::steady_clock::time_point started_;

  double rgb_period_s_ {0.1};
  double depth_period_s_ {0.2};
  double stale_after_s_ {2.0};
  std::uint64_t mesh_versions_ {0};
};

}  // namespace pimesh_dashboard

#endif  // PIMESH_DASHBOARD__DASHBOARD_NODE_HPP_
