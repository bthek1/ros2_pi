#ifndef PIMESH_CAMERA__CAMERA_NODE_HPP_
#define PIMESH_CAMERA__CAMERA_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <pimesh_msgs/msg/pipeline_stats.hpp>

#include "pimesh_camera/v4l2_capture.hpp"

namespace pimesh_camera
{

/// The Pi's only node: capture, stamp, publish. Nothing else runs here.
///
/// The capture loop is a dedicated thread blocking in poll(), NOT a ROS timer.
/// A timer asks the camera for a frame at a rate the node picked; a blocking
/// read takes frames at the rate the camera produces them. The difference is
/// measurable — the predecessor's timer-driven driver delivered 24 fps steady
/// while raw V4L2 capture on the same camera delivered 30, because the timer
/// beat against the sensor's own cadence.
class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options);
  ~CameraNode() override;

  /// True if the capture loop died. Standalone `main` turns this into a
  /// non-zero exit code: a camera node that idles after losing its device
  /// looks exactly like one that is working and publishing nothing.
  bool failed() const {return failed_.load();}

private:
  void capture_loop();
  void publish_stats(double rate_hz, double latency_ms);

  std::string frame_id_;
  int poll_timeout_ms_{0};
  double stats_period_s_{0.0};

  std::unique_ptr<V4l2Capture> capture_;
  sensor_msgs::msg::CameraInfo camera_info_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<pimesh_msgs::msg::PipelineStats>::SharedPtr stats_pub_;

  std::thread capture_thread_;
  std::atomic<bool> running_{true};
  std::atomic<bool> failed_{false};

  uint64_t frames_{0};
  uint64_t dropped_{0};
  uint32_t last_sequence_{0};
  bool have_sequence_{false};
};

}  // namespace pimesh_camera

#endif  // PIMESH_CAMERA__CAMERA_NODE_HPP_
