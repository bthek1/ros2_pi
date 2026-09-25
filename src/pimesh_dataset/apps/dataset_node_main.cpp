// The standalone entry point for DatasetNode.
//
// Thin, like every other main here — a node that only works standalone is a bug,
// and this one is composed into the container in every real use. What it adds is
// the same thing `camera_node_main.cpp` adds: every refusal in the constructor
// becomes one FATAL line and exit 1, rather than a process that is up,
// discoverable, and publishing nothing.
//
// It exists mostly so that `ros2 run pimesh_dataset dataset_node --ros-args -p
// dataset_dir:=...` is a way to check a sequence parses without standing up the
// whole container and a GPU.

#include <exception>
#include <memory>

#include "pimesh_dataset/nodes/dataset_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int status = 0;
  try {
    auto node = std::make_shared<pimesh_dataset::DatasetNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("dataset_node"), "refusing to start: %s", e.what());
    status = 1;
  }

  rclcpp::shutdown();
  return status;
}
