// The thin main, so `ros2 run pimesh_dashboard dashboard_node` works.
//
// Unlike every other node on the dev box, this one is *meant* to be run this way:
// the container is everything or nothing precisely so a frame is handed on as a
// pointer, and a dashboard has nothing to gain from that and a crash to cost.
// See dashboard_node.hpp.

#include <memory>

#include "pimesh_dashboard/nodes/dashboard_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pimesh_dashboard::DashboardNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
