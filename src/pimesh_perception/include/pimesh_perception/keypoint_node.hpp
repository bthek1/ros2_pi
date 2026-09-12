#ifndef PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_
#define PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "opencv2/core.hpp"
#include "pimesh_msgs/msg/keypoints.hpp"
#include "pimesh_perception/mailbox.hpp"
#include "pimesh_perception/orb_tracker.hpp"
#include "pimesh_perception/rotation_fit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
// **`.hpp`, not `.h`, and it is the cross-distro rule again.** tf2 renamed every
// header to `.hpp`; Jazzy ships both spellings with the `.h` forms emitting
// `#warning …_DEPRECATION`, and Lyrical has **deleted** the `.h` forms under
// `tf2/LinearMath/` outright. So `tf2/LinearMath/Matrix3x3.h` compiles on the Pi
// and stops the dev box dead — the same shape of failure as
// `ament_target_dependencies()`, from the other direction. Where two spellings
// exist, take the one that exists at both ends.
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace pimesh_perception
{

/// ORB on every decoded frame: corners, tracks, and a rotation-only pose.
///
/// It reads `/image_raw` from inside the container, so the 2.7 MB frame arrives as
/// a pointer — the same buffer decode_node wrote, which is what P2 proved. The
/// work then runs on its own thread behind a one-slot mailbox, because ORB at 500
/// features costs milliseconds and a subscription callback is not the place to
/// spend them.
///
/// **Two outputs and they are for different audiences.** `/keypoints` is the data:
/// positions, descriptors and track ids, struct-of-arrays, for the stages that
/// come later. `/keypoints/image/compressed` is a JPEG with the corners drawn on
/// it, for a person — throttled to ~10 Hz, because encoding it costs more than
/// detecting the features does and nobody can watch 59 fps of it anyway.
///
/// **The pose is rotation only, and says so.** Bearing rays through `K` from
/// consecutive matched pairs, one SVD, composed into a running orientation on
/// `odom -> base_link` with translation identically zero. That is the honest scope
/// of what these pairs can support: translation is not recoverable from rays
/// without depth, and a hand pan carries ~0.9 m of arm arc that this will not see.
/// The frame in RViz rotating and not translating is correct, not a bug, and is
/// the whole motivation for P7.
///
/// When the gates fail — too few pairs, or a residual too large — it **holds the
/// last pose** and logs the change of regime. Publishing a guess instead would put
/// a wrong orientation into the TF tree, and a wrong pose is worse than a stale
/// one: the mesh folds around it and nothing downstream can tell which frames were
/// guesses.
class KeypointNode : public rclcpp::Node
{
public:
  explicit KeypointNode(const rclcpp::NodeOptions & options);
  ~KeypointNode() override;

private:
  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void work();
  void process_frame(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void publish_keypoints(const TrackedFrame & frame, const sensor_msgs::msg::Image & source);
  void publish_preview(const TrackedFrame & frame, const sensor_msgs::msg::Image & source);
  void publish_pose(const rclcpp::Time & stamp);
  void log_stats();

  /// Estimate this frame's rotation from the tracker's consecutive pairs, and
  /// fold it into the running orientation if the gates allow.
  void update_pose(const TrackedFrame & frame, const std::string & optical_frame);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<pimesh_msgs::msg::Keypoints>::SharedPtr keypoints_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /// **A shared-pointer slot, and this is the one design decision in the package
  /// that was decided by a measurement rather than by reasoning.**
  ///
  /// The obvious signature for a zero-copy consumer is `std::unique_ptr`, and it is
  /// the right one when a topic has exactly one consumer — it is what
  /// `pimesh_hello` demonstrates and what decode_node uses above. It is the wrong
  /// one here, because `/image_raw` has two consumers in the container and will
  /// have four. rclcpp's intra-process manager serves *ownership-taking*
  /// subscriptions by moving the buffer into the last one and **copying it for
  /// every other** (`add_owned_msg_to_buffers` in
  /// rclcpp/experimental/intra_process_manager.hpp: "Copy the message since we have
  /// additional subscriptions to serve"). Subscriptions that take a shared const
  /// pointer are served by `add_shared_msg_to_buffers` instead, which hands *one*
  /// buffer to all of them, however many there are.
  ///
  /// Measured 2026-09-12 with tools/gates/ipc.sh: with keypoint_node and the probe
  /// both taking `unique_ptr`, **0 of 574** frames reached the probe at the address
  /// decode_node published — one consumer got the original and the other got a
  /// 2.7 MB copy, at 59 Hz, silently. With both taking `ConstSharedPtr` it is every
  /// frame. Nothing in any log, topic tool or rate measurement distinguishes the
  /// two; only the address comparison does.
  Mailbox<sensor_msgs::msg::Image::ConstSharedPtr> mailbox_;
  std::thread worker_;
  std::unique_ptr<OrbTracker> tracker_;

  // --- Configuration --------------------------------------------------------
  std::size_t min_pairs_ {8};
  double max_residual_rad_ {0.03};
  double reject_fraction_ {0.2};
  int preview_quality_ {80};
  double preview_period_s_ {0.1};
  bool publish_tf_ {true};
  std::string odom_frame_ {"odom"};
  std::string base_frame_ {"base_link"};

  // --- Intrinsics -----------------------------------------------------------
  //
  // Written by the CameraInfo callback on an executor thread and read by the
  // worker, hence the mutex. Nominal-looking intrinsics are not substituted if it
  // never arrives: without K there are no rays, and inventing them would produce a
  // pose that is confidently wrong rather than absent.
  std::mutex k_mutex_;
  cv::Matx33d k_ {cv::Matx33d::zeros()};
  bool have_k_ {false};

  // --- Pose -----------------------------------------------------------------
  cv::Matx33d orientation_ {cv::Matx33d::eye()};
  /// base_link's rotation from camera_optical_frame, from TF, looked up once.
  cv::Matx33d base_from_optical_ {cv::Matx33d::eye()};
  bool have_basis_ {false};
  bool holding_ {false};

  // --- Reused buffers, so the hot path does not allocate --------------------
  cv::Mat gray_;
  cv::Mat preview_;
  std::vector<std::uint8_t> jpeg_;

  // --- Counters -------------------------------------------------------------
  std::atomic<std::uint64_t> frames_ {0};
  std::atomic<std::uint64_t> pose_ok_ {0};
  std::atomic<std::uint64_t> pose_held_ {0};
  std::atomic<std::uint64_t> previews_ {0};
  std::atomic<double> cost_sum_ms_ {0.0};
  std::atomic<double> cost_max_ms_ {0.0};
  std::atomic<double> detect_sum_ms_ {0.0};
  std::atomic<double> match_sum_ms_ {0.0};
  std::atomic<double> preview_cost_sum_ms_ {0.0};
  std::atomic<double> keypoints_sum_ {0.0};
  std::atomic<double> matched_sum_ {0.0};
  std::atomic<double> residual_sum_ {0.0};

  std::uint64_t last_frames_ {0};
  std::size_t last_dropped_ {0};
  rclcpp::Time last_log_;
  rclcpp::Time last_preview_;
  bool have_preview_time_ {false};
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_
