// Standalone: `ros2 run pimesh_perception decode_node`.
//
// The pipeline never runs it this way — outside the container there is nobody to
// hand a pointer to, and every subscriber pays a 2.7 MB serialisation per frame.
// It exists because a node that only works inside a container is as awkward to
// debug as one that only works outside it, and because `ros2 run` is how you find
// out whether a crash is the node's or the container's.

#include <memory>

#include "pimesh_perception/decode_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pimesh_perception::DecodeNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
