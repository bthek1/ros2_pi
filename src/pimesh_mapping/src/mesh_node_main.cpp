// Standalone: `ros2 run pimesh_mapping mesh_node`.
//
// **It will start and it will refuse to do anything**, with a message naming the
// volume key it was looking for — because the volume lives in the process that
// fills it and there is no fusion_node here. That is the honest behaviour for a
// node whose input is shared memory rather than a topic, and it is the same shape
// depth_node has on a machine with no ONNX Runtime. See shared_volume.hpp.
//
// It exists anyway, because `ros2 run` is how you find out whether a crash is the
// node's or the container's, and because the two services are reachable this way
// for inspection.

#include <memory>

#include "pimesh_mapping/mesh_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pimesh_mapping::MeshNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
