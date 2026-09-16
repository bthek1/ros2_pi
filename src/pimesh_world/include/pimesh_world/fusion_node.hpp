#ifndef PIMESH_WORLD__FUSION_NODE_HPP_
#define PIMESH_WORLD__FUSION_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"
#include "pimesh_msgs/msg/pipeline_stats.hpp"
#include "pimesh_perception/mailbox.hpp"
#include "pimesh_world/scale_aligner.hpp"
#include "pimesh_world/shared_volume.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
// **`.hpp`, not `.h`.** tf2 renamed every one of these; the `.h` forms are
// deprecated on Jazzy and warn on Lyrical, and some of `tf2/LinearMath/` is gone
// there outright. Where two spellings exist, this workspace takes the one that
// exists at both ends.
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace pimesh_world
{

/// Posed depth maps in, one consistent volume out.
///
/// **This is where the pipeline stops being a stream and starts being a map.**
/// Everything upstream is per-frame: a frame is decoded, its corners found, its
/// depth inferred, and then it is gone. This node is the first stage that
/// *remembers* — and the memory is a weighted average per voxel rather than a
/// list of frames, which is what turns a depth stream that wobbles by ±4% into a
/// wall that sits in one place.
///
/// Four things happen per frame and the order matters:
///
///  1. **Pair.** `/depth` and `/depth/rgb` carry the same stamp by construction —
///     `depth_node` publishes the colour twin from the frame it actually inferred
///     on, precisely so this pairing is exact rather than approximate. The match
///     is on the stamp and nothing else; there is no time tolerance, because a
///     tolerance here would silently colour a wall with the picture of a different
///     instant and nothing downstream could tell.
///  2. **Pose.** `map -> camera_optical_frame` at the frame's own stamp, from TF.
///     A frame whose pose cannot be looked up is **dropped, not integrated at the
///     last known pose**: `keypoint_node` already holds its pose when its gates
///     fail, so a missing transform means something further wrong, and folding a
///     frame in at a guessed orientation smears the map in a way no later frame
///     undoes.
///  3. **Align.** Ray-cast the volume from that pose, take the median ratio
///     against the incoming depth, and correct the frame's *deviation* from a
///     rolling baseline. See `scale_aligner.hpp` for why it is a high-pass and
///     what happened to the predecessor when it was not.
///  4. **Integrate.** Fold the scaled depth and its colour into the TSDF.
///
/// **The mailbox is the same one every expensive stage here uses**, and the drop
/// count is published rather than hidden. Depth runs at ~17 Hz and this node's
/// budget is 20 ms of integration, so in normal operation it keeps up and drops
/// nothing — which makes a rising drop count a real signal rather than background
/// noise, unlike `depth_node` where dropping two frames in three is the design.
///
/// **`align` is a parameter because `tools/gates/fusion.sh` needs the control.**
/// The gate replays the same clip twice, once each way, and compares how far the
/// next observation sits from the surface already built. A knob whose off
/// position nobody has measured is a knob nobody knows the value of.
class FusionNode : public rclcpp::Node
{
public:
  explicit FusionNode(const rclcpp::NodeOptions & options);
  ~FusionNode() override;

private:
  /// One depth map and the exact colour frame it was inferred on.
  ///
  /// `arrived` is a steady-clock stamp taken in the subscription callback, and it
  /// is what the integration *lag* is measured from. Deliberately not a ROS time:
  /// the frame's `header.stamp` is the Pi's clock and this machine's relationship
  /// to it is NTP's business, so a lag computed against it would be measuring two
  /// machines rather than this node's backlog.
  struct Frame
  {
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    sensor_msgs::msg::Image::ConstSharedPtr rgb;
    std::chrono::steady_clock::time_point arrived;
  };

  void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_rgb(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void work();
  void process(Frame & frame);
  sensor_msgs::msg::Image::ConstSharedPtr colour_for(const builtin_interfaces::msg::Time & stamp);
  bool pose_at(const builtin_interfaces::msg::Time & stamp, cv::Affine3d & world_from_camera);
  void log_stats();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<pimesh_msgs::msg::PipelineStats>::SharedPtr stats_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<SharedVolume> volume_;
  ScaleAligner aligner_;
  pimesh_perception::Mailbox<std::shared_ptr<Frame>> mailbox_;
  std::thread worker_;

  /// The last few colour frames, waiting for the depth map with their stamp.
  ///
  /// `depth_node` publishes the colour twin *first* and the depth map immediately
  /// after, so in practice the match is against the newest entry and this holds
  /// eight only so that a reordering under load is not a dropped frame. Keyed by
  /// the stamp in nanoseconds; a `deque` rather than a map because eight linear
  /// comparisons are cheaper than a tree and the eviction is from the front.
  std::mutex rgb_mutex_;
  std::deque<sensor_msgs::msg::Image::ConstSharedPtr> recent_rgb_;
  std::size_t rgb_history_ {8};

  std::mutex k_mutex_;
  cv::Matx33d k_ {cv::Matx33d::eye()};
  bool have_k_ {false};
  int k_width_ {0};
  int k_height_ {0};

  std::string world_frame_ {"map"};
  std::string optical_frame_ {"camera_optical_frame"};
  bool align_ {true};
  int raycast_stride_ {16};
  double tf_timeout_s_ {0.05};
  double agree_tolerance_ {0.05};

  /// Reused across frames so the hot path allocates nothing: the scaled depth,
  /// the downsampled copy the aligner compares, and the ray-cast.
  cv::Mat scaled_depth_;
  cv::Mat small_depth_;
  cv::Mat expected_;

  // --- Counters -------------------------------------------------------------
  std::atomic<std::uint64_t> depth_in_ {0};
  std::atomic<std::uint64_t> unpaired_ {0};
  std::atomic<std::uint64_t> no_pose_ {0};
  std::atomic<std::uint64_t> no_k_ {0};
  std::atomic<std::uint64_t> integrated_ {0};
  std::atomic<std::uint64_t> aligned_ {0};
  std::atomic<std::uint64_t> clamped_ {0};
  std::atomic<std::uint64_t> refused_ {0};

  /// Per-window samples, taken under one mutex and swapped out by the stats
  /// timer. Vectors rather than running sums because every one of these is
  /// reported as a p95 as well as a mean, and a p95 cannot be accumulated.
  std::mutex sample_mutex_;
  std::vector<double> integrate_ms_;
  std::vector<double> align_ms_;
  std::vector<double> total_ms_;
  std::vector<double> lag_ms_;
  /// The wall-clock interval between one integration finishing and the next, on
  /// the worker's own steady clock.
  ///
  /// **This is the number P6's gate reads, and it is not the same as the rate.** A
  /// rate averaged over five seconds is blind to a single 400 ms stall — which is
  /// exactly what meshing under the volume's lock would produce, and the whole
  /// reason `mesh_node` snapshots in chunks. A median and a maximum over the same
  /// window make the stall the thing being measured rather than something
  /// averaged away.
  std::vector<double> interval_ms_;
  bool have_last_integration_ {false};
  std::chrono::steady_clock::time_point last_integration_;
  std::vector<double> gap_m_;
  std::vector<double> overlap_;
  std::vector<double> scale_;
  std::vector<double> agree_;

  std::uint64_t last_integrated_ {0};
  std::size_t last_dropped_ {0};
  rclcpp::Time last_log_;
};

}  // namespace pimesh_world

#endif  // PIMESH_WORLD__FUSION_NODE_HPP_
