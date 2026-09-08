// Standalone entry point for EchoNode. Thin, for the same reason
// hello_node_main.cpp is: the container runs none of it.

#include <exception>
#include <memory>

#include "pimesh_hello/echo_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<pimesh_hello::EchoNode>(rclcpp::NodeOptions()));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("echo_node"), "refusing to start: %s", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
