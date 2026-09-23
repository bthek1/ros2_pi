#ifndef PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_
#define PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "opencv2/core.hpp"
#include "pimesh_msgs/msg/keypoints.hpp"
#include "pimesh_msgs/msg/pipeline_stats.hpp"
#include "pimesh_core/mailbox.hpp"
#include "pimesh_frontend/orb_tracker.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace pimesh_frontend
{

/// ORB on every decoded frame: corners, and who each one is.
///
/// It reads `/image_raw` from inside the container, so the 2.7 MB frame arrives as
/// a pointer — the same buffer decode_node wrote, which is what P2 proved. The
/// work then runs on its own thread behind a one-slot mailbox, because ORB at 500
/// features costs milliseconds and a subscription callback is not the place to
/// spend them.
///
/// **The pose used to be here and is not, since 2026-09-23.** This node published
/// *two* `/pipeline/stats` rows — `keypoints` at the camera's rate on one thread
/// and `odometry` at the depth rate on another — because it was two stages wearing
/// one name, and the pipeline table had listed them as two since P7. They are two
/// nodes now: `odometry_node` subscribes to what this one publishes. Nothing about
/// the estimator changed in the move; what changed is that the node boundary and
/// the stage boundary are the same line.
///
/// **Two outputs and they are for different audiences.** `/keypoints` is the data:
/// positions, descriptors, track ids and each corner's position one frame ago,
/// struct-of-arrays, for the stages that come later.
/// `/keypoints/image/compressed` is a JPEG with the corners drawn on it, for a
/// person — throttled to ~10 Hz, because encoding it costs more than detecting the
/// features does and nobody can watch 59 fps of it anyway.
///
/// **Two matchings go out on that topic, and conflating them is the trap.** Track
/// ids come from a pooled pass over a ten-frame window, which forgives detection
/// churn; `prev_x`/`prev_y` come from a mutual-best pass against the previous frame
/// alone, which is what the geometry needs. See OrbTracker, and the field comments
/// in pimesh_msgs/Keypoints.msg.
class KeypointNode : public rclcpp::Node
{
public:
  explicit KeypointNode(const rclcpp::NodeOptions & options);
  ~KeypointNode() override;

private:
  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void work();
  void process_frame(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void publish_keypoints(const TrackedFrame & frame, const sensor_msgs::msg::Image & source);
  void publish_preview(const TrackedFrame & frame, const sensor_msgs::msg::Image & source);
  void log_stats();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<pimesh_msgs::msg::Keypoints>::SharedPtr keypoints_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_pub_;
  /// `/pipeline/stats`, the contract P8's dashboard reads. One row: this node is
  /// one stage now.
  rclcpp::Publisher<pimesh_msgs::msg::PipelineStats>::SharedPtr stats_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  /// **A shared-pointer slot, and this is the one design decision in the package
  /// that was decided by a measurement rather than by reasoning.**
  ///
  /// The obvious signature for a zero-copy consumer is `std::unique_ptr`, and it is
  /// the right one when a topic has exactly one consumer — it is what decode_node
  /// uses on its own inter-process subscription, where the middleware allocates a
  /// fresh message anyway. It is the wrong one here, because `/image_raw` has more
  /// than one consumer in the container. rclcpp's intra-process manager serves
  /// *ownership-taking* subscriptions by moving the buffer into the last one and
  /// **copying it for every other** (`add_owned_msg_to_buffers` in
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
  pimesh_core::Mailbox<sensor_msgs::msg::Image::ConstSharedPtr> mailbox_;
  std::thread worker_;
  std::unique_ptr<OrbTracker> tracker_;

  // --- Configuration --------------------------------------------------------
  int preview_quality_ {80};
  double preview_period_s_ {0.1};

  // --- Reused buffers, so the hot path does not allocate --------------------
  cv::Mat gray_;
  cv::Mat preview_;
  std::vector<std::uint8_t> jpeg_;

  // --- Counters -------------------------------------------------------------
  std::atomic<std::uint64_t> frames_ {0};
  /// Frames with at least one feature, which is the denominator of the matched
  /// fraction — and `empty_frames_` is the rest. See process_frame() for why the
  /// two are counted apart.
  std::atomic<std::uint64_t> measured_frames_ {0};
  std::atomic<std::uint64_t> empty_frames_ {0};
  std::atomic<std::uint64_t> previews_ {0};
  std::atomic<double> cost_sum_ms_ {0.0};
  std::atomic<double> cost_max_ms_ {0.0};
  std::atomic<double> detect_sum_ms_ {0.0};
  std::atomic<double> match_sum_ms_ {0.0};
  std::atomic<double> preview_cost_sum_ms_ {0.0};
  std::atomic<double> keypoints_sum_ {0.0};
  std::atomic<double> matched_sum_ {0.0};

  std::uint64_t last_frames_ {0};
  /// A rate is always a delta over a span here, never a total over an uptime: an
  /// average since startup only ever moves slowly, so a stage that stopped an
  /// instant ago still reads healthy — which is precisely what a dashboard is for
  /// noticing.
  std::size_t last_dropped_ {0};
  rclcpp::Time last_log_;
  rclcpp::Time last_preview_;
  bool have_preview_time_ {false};
};

}  // namespace pimesh_frontend

#endif  // PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_
