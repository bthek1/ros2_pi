// Standalone entry point. The node is a component so it *can* be composed, but
// on the Pi it runs alone — there is nothing to compose it with.
//
// The whole reason this file is not the three-line boilerplate: a camera
// process must exit NON-ZERO when it cannot capture. The driver this replaces
// logs one ERROR on a missing device and then idles forever, which from the
// outside is indistinguishable from a healthy node publishing into a topic
// nobody reads.

#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pimesh_camera/camera_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<pimesh_camera::CameraNode> node;
  try {
    node = std::make_shared<pimesh_camera::CameraNode>(rclcpp::NodeOptions());
  } catch (const std::exception & e) {
    // No logger yet on this path in the failure case, so say it on stderr too.
    RCLCPP_FATAL(rclcpp::get_logger("camera_node"), "cannot start: %s", e.what());
    std::fprintf(stderr, "camera_node: cannot start: %s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  const bool failed = node->failed();
  node.reset();
  rclcpp::shutdown();
  return failed ? 1 : 0;
}
