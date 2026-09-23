// Standalone: `ros2 run pimesh_frontend odometry_node`.
//
// Useful for debugging the node in isolation and wrong for running the pipeline:
// outside the container it subscribes to `/keypoints` and `/depth` across a
// process boundary, which serialises a 3.7 MB depth map per frame and throws away
// what P2 established. If the rate collapses when you do this, that is the
// measurement working.

#include <memory>

#include "pimesh_frontend/odometry_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pimesh_frontend::OdometryNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
