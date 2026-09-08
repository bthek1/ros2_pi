// The standalone entry point for HelloNode.
//
// It is thin on purpose. The node has to behave identically whether it is this
// process or one of several components inside a container, and a container runs
// none of this file — so anything that lives here is behaviour the composed
// build silently does not have. init, construct, spin, shutdown.
//
// The one addition is the catch. rclcpp throws when a parameter's value
// violates its descriptor's range, and that exception surfaces out of the
// constructor. Letting it escape would still exit non-zero, but through
// std::terminate and a core dump; turning it into one FATAL line and exit 1 is
// the difference between a node that refuses a bad parameter and a node that
// crashed on one.

#include <exception>
#include <memory>

#include "pimesh_hello/hello_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<pimesh_hello::HelloNode>(rclcpp::NodeOptions()));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("hello_node"), "refusing to start: %s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
