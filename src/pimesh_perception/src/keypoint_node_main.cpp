// Standalone entry point. Same bargain as decode_node_main: the node's real
// home is the container, and running it alone means it COPIES every frame out
// of decode instead of sharing the buffer — the constructor says so at startup.
//
// It earns its place as a debugging tool. `ros2 run pimesh_perception
// keypoint_node` against a running decode is the fastest way to answer "is ORB
// finding anything in this room at all", with the container's composition
// removed from the list of suspects.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pimesh_perception/keypoint_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pimesh_perception::KeypointNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
