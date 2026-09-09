// The standalone entry point for CameraNode, and the whole of this package's
// "fail loudly" behaviour.
//
// It is thin, like every other main in this workspace — the node must behave
// identically composed or standalone, so anything that lives here is behaviour
// the composed build does not have. But the two things it does are the phase's
// requirement rather than decoration:
//
//  1. A device that cannot be opened or is already streamed by somebody else
//     throws out of the constructor, and that becomes exit 1 with one clear
//     line. `usb_cam` 0.8.1 logs an ERROR and idles forever instead, which
//     produces a process that is up, discoverable, subscribed to, and
//     publishing nothing — indistinguishable from a working camera pointed at a
//     dark room until you look at the topic.
//  2. A device that dies *mid-stream* also exits non-zero, via the node's own
//     exit_code(). rclcpp::spin() returns when the capture thread shuts the
//     context down, and the status follows the node's verdict rather than
//     spin()'s.

#include <exception>
#include <memory>

#include "pimesh_camera/camera_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int status = 0;
  try {
    auto node = std::make_shared<pimesh_camera::CameraNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    status = node->exit_code();
  } catch (const std::exception & e) {
    // CaptureError lands here, and so does a parameter that violates its
    // descriptor's range. Turning it into one FATAL line and a status is the
    // difference between a node that refuses to start and a node that crashed.
    RCLCPP_FATAL(rclcpp::get_logger("camera_node"), "refusing to start: %s", e.what());
    status = 1;
  }

  rclcpp::shutdown();
  return status;
}
