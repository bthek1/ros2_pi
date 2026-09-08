// Standalone entry point. Same bargain as the other two: the node's real home
// is the container, and running it alone means it copies every frame out of
// decode instead of sharing the buffer.
//
// It earns its place because this node has the most ways to fail before it
// ever sees an image — a missing model file, a CUDA runtime that will not
// load, a session that silently fell back to the CPU. `ros2 run
// pimesh_perception depth_node` puts all of that in one terminal with nothing
// else in it.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pimesh_perception/depth_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pimesh_perception::DepthNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
