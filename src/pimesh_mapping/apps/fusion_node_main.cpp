// Standalone: `ros2 run pimesh_mapping fusion_node`.
//
// The pipeline never runs it this way — outside the container the 3.7 MB depth
// map arrives by serialisation rather than by pointer, and `mesh_node` in another
// process cannot see the volume this one fills (see shared_volume.hpp). It exists
// because a node that only works inside a container is as awkward to debug as one
// that only works outside it, and because `ros2 run` is how you find out whether
// a crash is the node's or the container's.

#include <memory>

#include "pimesh_mapping/nodes/fusion_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pimesh_mapping::FusionNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
