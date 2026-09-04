// Standalone entry point. The node's real home is the container — running it
// as its own process means every downstream stage pays for a serialised copy,
// and the startup log says so out loud.
//
// It exists because a component that only works when composed is as hard to
// debug as one that only works standalone. `ros2 run pimesh_perception
// decode_node` is the fastest way to see whether the Pi's stream decodes at
// all, with nothing else in the picture.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pimesh_perception/decode_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pimesh_perception::DecodeNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
