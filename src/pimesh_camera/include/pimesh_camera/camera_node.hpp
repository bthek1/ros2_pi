#ifndef PIMESH_CAMERA__CAMERA_NODE_HPP_
#define PIMESH_CAMERA__CAMERA_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include "pimesh_camera/v4l2_capture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace pimesh_camera
{

/// The Pi's only job: frames off the sensor, stamped honestly, onto the wire.
///
/// No decode, no re-encode, no processing. The MJPEG bytes the camera produced
/// are the bytes that get published. Everything expensive happens on the dev
/// box, where there is a GPU to pay for it.
class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options);
  ~CameraNode() override;

  /// What the process should exit with. Non-zero once capture has failed in a
  /// way that will not recover.
  ///
  /// A node that cannot produce frames must not sit there being a node. This is
  /// how that reaches `main`: the capture thread sets it and shuts the context
  /// down, `spin` returns, and the exit status says what happened. The
  /// alternative — log an ERROR and keep spinning — is what `usb_cam` does, and
  /// it produces a process that looks alive to every tool in ROS while
  /// publishing nothing at all.
  int exit_code() const {return exit_code_.load();}

private:
  void capture_loop(int timeout_ms);
  void publish(const Frame & frame);
  rclcpp::Time stamp_for(const Frame & frame);
  sensor_msgs::msg::CameraInfo build_camera_info();
  void fail(const std::string & why);

  std::unique_ptr<V4l2Capture> capture_;
  std::thread worker_;
  std::atomic<bool> running_ {true};
  std::atomic<int> exit_code_ {0};

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;

  std::string frame_id_;
  sensor_msgs::msg::CameraInfo camera_info_;

  /// Whether `camera_info_` came from a real checkerboard run.
  ///
  /// **Derived, never declared.** It was a `calibrated` parameter until P9, and
  /// a parameter is the wrong shape for this: it let a human assert the claim
  /// the startup warning exists to police, so `calibrated:=true` with no
  /// calibration silenced the warning and changed nothing else. Now it is set by
  /// build_camera_info() from whether a file loaded *and* carries non-zero
  /// distortion — a thing that cannot be true without somebody having run the
  /// board.
  bool calibrated_ {false};

  /// Whether `camera_info_`'s numbers came out of a file at all.
  ///
  /// Distinct from `calibrated_` because there are three states, not two: a real
  /// calibration, the nominal fallback, and a file that loaded cleanly and
  /// carries all-zero distortion. The last is not calibrated but it is also not
  /// nominal, and the startup warning has to say which — printing "camera_info
  /// carries NOMINAL intrinsics (fx=905.1 ...)" with the file's own focal length
  /// in the parentheses is a claim about the wrong numbers.
  bool info_from_file_ {false};

  /// Set once, the first time a frame arrives without a monotonic timestamp, so
  /// the warning is loud rather than 47 times a second.
  bool warned_no_monotonic_ {false};
  bool warned_implausible_age_ {false};

  std::uint64_t frames_ {0};
  std::uint32_t last_sequence_ {0};
  bool have_sequence_ {false};
  std::uint64_t kernel_drops_ {0};
};

}  // namespace pimesh_camera

#endif  // PIMESH_CAMERA__CAMERA_NODE_HPP_
