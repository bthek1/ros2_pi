// Standalone: `ros2 run pimesh_depth depth_node`.
//
// The pipeline never runs it this way — outside the container the 2.7 MB frame
// arrives by serialisation rather than by pointer. It exists because a node that
// only works inside a container is as awkward to debug as one that only works
// outside it, and because `ros2 run` is how you find out whether a crash is the
// node's or the container's.
//
// It is also the quickest way to see the provider line on its own, without a
// container's startup around it:
//
//   ros2 run pimesh_depth depth_node --ros-args -p use_cuda:=false

#include <memory>

#include "pimesh_depth/nodes/depth_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pimesh_depth::DepthNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
