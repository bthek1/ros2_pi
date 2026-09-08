// The pipeline's first inference stage: ORB features off the decoded frame,
// and a rotation-only odometer built out of them.
//
// It sits inside the same container as `decode_node` and subscribes that
// node's output, so the 2.7 MB frame it works on is the same buffer decode
// published — never a copy, never a serialisation. That is the whole reason
// the container exists (CLAUDE.md, "One Wi-Fi reader, not five").
//
// Shape, and it is the house pattern from here on:
//
//   executor thread  ──▶ on_frame(): move a shared_ptr into a 1-deep mailbox
//   worker thread    ──▶ detect, match, estimate, publish
//
// ORB at 500 features is cheap enough for the full camera rate, which makes
// this stage the contrast that gives the pipeline its shape: keypoints keep up
// at 30-60 Hz, depth will not exceed ~13 Hz, and the dashboard exists partly to
// make that gap visible. The mailbox is what lets the two coexist without a
// queue growing between them.
//
// **The odometer is deliberately rotation-only at this phase.** With no
// baseline the essential matrix is degenerate and with no depth translation
// has no scale, so a hand-held pan is all this can honestly measure. P7
// back-fills 6-DoF once `depth_node` exists. Until then `odom -> base_link`
// carries orientation and zero translation, and the node says which regime it
// is in on every stats message rather than letting a consumer assume.

#ifndef PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_
#define PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <pimesh_msgs/msg/keypoints.hpp>
#include <pimesh_msgs/msg/pipeline_stats.hpp>

#include "pimesh_perception/mailbox.hpp"
#include "pimesh_perception/orb_tracker.hpp"
#include "pimesh_perception/rotation.hpp"

namespace pimesh_perception
{

class KeypointNode : public rclcpp::Node
{
public:
  explicit KeypointNode(const rclcpp::NodeOptions & options);
  ~KeypointNode() override;

private:
  using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;

  void on_frame(ImageConstPtr msg);
  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void on_reset(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void work_loop();
  void preview_loop();
  void publish_stats();

  /// Advance the composed orientation by one frame, or hold it. Returns the
  /// estimate so the caller can count why it was rejected.
  RotationEstimate update_orientation(const TrackedFrame & tracked);
  void publish_pose(const std_msgs::msg::Header & header);

  /// Everything the preview thread needs, detached from the tracker's state.
  ///
  /// The frame is a shared_ptr, so handing it over is a refcount bump and the
  /// 2.7 MB is not touched. The keypoints ARE copied — ~14 kB for 500 features
  /// — because the tracker's next frame will overwrite them, and a bounded
  /// copy is exactly what a handoff is allowed to cost.
  struct PreviewJob
  {
    ImageConstPtr frame;
    std::vector<cv::KeyPoint> fresh;
    std::vector<cv::KeyPoint> matched;
  };
  void enqueue_preview(const ImageConstPtr & frame, const TrackedFrame & tracked);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<pimesh_msgs::msg::Keypoints>::SharedPtr keypoints_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<pimesh_msgs::msg::PipelineStats>::SharedPtr stats_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  Mailbox<ImageConstPtr> mailbox_;
  std::thread worker_;

  // The preview gets its OWN thread and its own one-deep mailbox, for the same
  // reason the tracker has one: drawing 500 rich keypoints and JPEG-encoding a
  // 1280x720 image costs ~9 ms, and on the tracking thread that showed up as a
  // p95 of 20.6 ms against a 10.9 ms mean (measured 2026-09-07) plus a stream
  // of dropped frames. A picture for humans must never be able to slow the
  // estimator down. Newest-wins here too: a preview frame that has been
  // overtaken has nothing left to show.
  Mailbox<PreviewJob> preview_mailbox_;
  std::thread preview_worker_;
  std::atomic<bool> running_{true};

  std::unique_ptr<OrbTracker> tracker_;      // worker-owned after construction
  RotationGates gates_;
  cv::Mat gray_;                             // reused across frames
  std::vector<int> jpeg_params_;

  // Parameters, read once. All fixed at startup: changing the feature cap or
  // the window mid-run would invalidate the state the tracker is holding.
  double stale_after_s_{2.0};
  double preview_period_s_{0.1};
  bool publish_tf_{true};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};

  /// K, written by the executor thread on /camera_info and read by the worker.
  std::mutex intrinsics_mutex_;
  CameraMatrix k_{};
  bool calibrated_{false};
  bool warned_uncalibrated_{false};

  /// The composed orientation, in OPTICAL axes. Worker-only.
  Eigen::Matrix3d orientation_{Eigen::Matrix3d::Identity()};
  std::atomic<bool> reset_requested_{false};
  std::chrono::steady_clock::time_point last_preview_{};

  std::atomic<uint64_t> processed_{0};
  std::atomic<uint64_t> previews_{0};
  std::atomic<uint64_t> undecodable_{0};
  std::atomic<uint64_t> pose_ok_{0};
  std::atomic<uint64_t> pose_rejected_{0};
  std::atomic<uint64_t> reject_few_{0};
  std::atomic<uint64_t> reject_residual_{0};
  std::atomic<uint64_t> reject_no_intrinsics_{0};

  // Written by the worker, drained by the stats timer on the executor thread.
  std::mutex window_mutex_;
  std::vector<double> window_ms_;
  std::vector<double> matched_fractions_;
  std::chrono::steady_clock::time_point window_start_;
  std::chrono::steady_clock::time_point last_frame_;
  bool ever_received_{false};
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_
