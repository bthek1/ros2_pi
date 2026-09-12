// Standalone: `ros2 run pimesh_perception keypoint_node`.
//
// Useful for debugging the node in isolation and wrong for running the pipeline:
// outside the container it subscribes to `/image_raw` across a process boundary,
// which serialises 2.7 MB per frame and throws away what P2 established. If the
// frame rate collapses when you do this, that is the measurement working.

#include <memory>

#include "pimesh_perception/keypoint_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pimesh_perception::KeypointNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
