#ifndef PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_
#define PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/core.hpp"
#include "pimesh_msgs/msg/keypoints.hpp"
#include "pimesh_msgs/msg/pipeline_stats.hpp"
#include "pimesh_perception/keyframe_store.hpp"
#include "pimesh_perception/mailbox.hpp"
#include "pimesh_perception/orb_tracker.hpp"
#include "pimesh_perception/rgbd_odometry.hpp"
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

/// Which estimator is driving the pose, and it is a choice rather than a fallback
/// chain.
///
/// P3 could only do `RotationOnly`, because bearing rays cannot see translation.
/// P7 added `SixDof`, which pairs each frame's corners with its own depth map and
/// fits a rigid transform to the resulting landmarks. `RotationOnly` stays
/// selectable because it is the **control run** `tools/gates/odom.sh` measures
/// against: a claim that translation improves the surface needs a run without it.
enum class OdometryRegime
{
  RotationOnly,
  SixDof,
};

/// ORB on every decoded frame: corners, tracks, and a pose.
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
/// **Two threads and two clocks, which is the shape P7 gave this node.** The ORB
/// worker runs at the camera's rate on every decoded frame. The *pose* worker runs
/// at the depth rate — one frame in three, ~17 Hz — because a 6-DoF estimate needs
/// a depth map and `depth_node` is the pipeline's clock. It finds the ORB output
/// belonging to each depth map's own stamp in a short history, so the two are
/// exactly the same instant rather than the nearest one: a tolerance here would
/// read a corner's distance off a picture taken while the camera was somewhere
/// else, and a 15 mm voxel does not forgive that.
///
/// **In `SixDof` the pose worker is the only thing that publishes a pose**, and
/// the ORB worker publishes none at all. Two publishers of one TF edge at two
/// rates would interleave stamps carrying poses estimated at different instants,
/// and `tf2` would interpolate between them as though they were a trajectory. The
/// cost is that `odom -> base_link` exists at the depth stamps and not between
/// them, which is exactly the set of stamps `fusion_node` looks up — it waits up
/// to its `tf_timeout_ms` for each one, so the handover is a wait and not a race.
///
/// **When the gates fail it holds the last pose** — too few pairs, too few
/// landmarks, or a residual too large — and logs the change of regime. Publishing
/// a guess instead would put a wrong pose into the TF tree, and a wrong pose is
/// worse than a stale one: the TSDF bakes it into every voxel it touches and
/// nothing downstream can tell which frames were guesses.
class KeypointNode : public rclcpp::Node
{
public:
  explicit KeypointNode(const rclcpp::NodeOptions & options);
  ~KeypointNode() override;

private:
  /// One frame's ORB output, kept just long enough for its depth map to catch up.
  ///
  /// Pixels and ids rather than the whole TrackedFrame: the history is ~90 frames
  /// deep and the parts that are not needed here — the preview, the consecutive
  /// pairs — are the bulk of it. `descriptors` is a cv::Mat, so keeping it is a
  /// reference count and not a copy.
  struct FrameRecord
  {
    std::int64_t stamp_ns {0};
    std::string optical_frame;
    std::vector<cv::Point2f> pixels;
    std::vector<std::int32_t> ids;
    cv::Mat descriptors;
  };

  /// The landmarks one depth-backed frame contributed, indexed by track id.
  ///
  /// `ids` is sorted ascending and `points` and `bearings` are parallel to it, so
  /// two views are intersected by a linear merge rather than a hash lookup per
  /// feature. Sorted because a track id is assigned in detection order and
  /// detection order is not stable between frames.
  ///
  /// **Both a 3D point and a unit ray for every entry**, which is not redundant:
  /// the rotation is fitted from the rays and the translation from the points.
  /// See TranslationFit in rgbd_odometry.hpp — a pixel is exact and a depth
  /// reading is not, and solving the two together lets the depth noise into the
  /// rotation.
  struct RgbdView
  {
    std::int64_t stamp_ns {0};
    std::vector<std::int32_t> ids;
    std::vector<cv::Vec3d> points;
    std::vector<cv::Vec3d> bearings;
    /// Where each of these features sat on the sensor. The *current* frame's
    /// pixels are what PnP poses against, and a feature needs no depth of its own
    /// to supply one — so this vector is longer than `points` on the frame being
    /// posed and parallel to it on a reference.
    std::vector<cv::Point2f> pixels;
    /// `odom <- camera_optical_frame` when this view was taken. Only meaningful on
    /// the reference view, which is the one the pose is measured *from*.
    cv::Affine3d odom_from_camera {cv::Affine3d::Identity()};
    bool valid() const {return !ids.empty();}
  };

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);
  void work();
  void pose_work();
  void process_frame(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void process_depth(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void publish_keypoints(const TrackedFrame & frame, const sensor_msgs::msg::Image & source);
  void publish_preview(const TrackedFrame & frame, const sensor_msgs::msg::Image & source);
  void publish_pose(const rclcpp::Time & stamp);
  void log_stats();

  /// How far the pose is from where it started, which is not the path length.
  double net_displacement();

  /// Estimate this frame's rotation from the tracker's consecutive pairs, and
  /// fold it into the running orientation if the gates allow. `RotationOnly` only.
  void update_pose(const TrackedFrame & frame, const std::string & optical_frame);

  /// Read `base_link <- camera_optical_frame` from TF, once. Both workers need it,
  /// so it is behind its own mutex and its own guard rather than duplicated.
  bool ensure_basis(const std::string & optical_frame);

  /// The ORB output captured at exactly `stamp_ns`, or nullptr. **Exact**, never
  /// nearest: `depth_node` publishes each depth map under the stamp of the frame
  /// it inferred on, so an exact match exists whenever the history is deep enough,
  /// and a nearest match would silently pair a corner with a different instant.
  std::shared_ptr<const FrameRecord> record_at(std::int64_t stamp_ns);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<pimesh_msgs::msg::Keypoints>::SharedPtr keypoints_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  /// `/pipeline/stats`, the contract P8's dashboard reads. **Two rows, not one**:
  /// this node runs two estimators on two threads at two rates, and averaging
  /// them into a single row would report the ORB path's 58 Hz beside the pose
  /// solve's cost — a line in which no two numbers describe the same thing.
  rclcpp::Publisher<pimesh_msgs::msg::PipelineStats>::SharedPtr stats_pub_;
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
  ///
  /// The same reasoning covers `depth_mailbox_`: `/depth` already has
  /// `fusion_node` on it, so this node is the second consumer of a 3.7 MB message.
  Mailbox<sensor_msgs::msg::Image::ConstSharedPtr> mailbox_;
  Mailbox<sensor_msgs::msg::Image::ConstSharedPtr> depth_mailbox_;
  std::thread worker_;
  std::thread pose_worker_;
  std::unique_ptr<OrbTracker> tracker_;

  // --- Configuration --------------------------------------------------------
  OdometryRegime regime_ {OdometryRegime::SixDof};
  std::size_t min_pairs_ {8};
  double max_residual_rad_ {0.03};
  double reject_fraction_ {0.2};
  std::size_t min_landmark_pairs_ {12};
  /// Below this many landmarks shared with the reference view, the reference has
  /// stopped describing what is in front of the lens and a new one is taken —
  /// whatever the angle and distance thresholds say. A keyframe the camera has
  /// turned away from is a keyframe whose few surviving correspondences are all
  /// in one corner of the frame, which is the geometry TranslationFit is worst on.
  std::size_t keyframe_min_shared_ {60};
  /// "pnp" | "points". See PnpFit in rgbd_odometry.hpp for the four measurements
  /// that made pnp the default and left points as the control.
  bool solve_pnp_ {true};
  double reprojection_px_ {3.0};
  double max_reprojection_px_ {2.0};
  /// **The gate a low reprojection error cannot supply.** A PnP solve reports how
  /// well its pose explains the pixels it was given; it has no opinion at all
  /// about whether the 3D points those pixels were matched against are where the
  /// depth network said. Measured on `bags/desk1`: a single step of **9.4 m**
  /// between two depth frames, at a mean inlier reprojection of 1.23 px, which
  /// then defined the reference every later frame was measured from. The
  /// trajectory before and after it was ordinary.
  ///
  /// A residual gate structurally cannot catch that — the pose was the best
  /// explanation of the pixels — so the plausibility of the *motion* is a separate
  /// question and gets a separate refusal.
  double max_speed_m_s_ {2.0};
  double max_turn_rad_s_ {6.0};
  /// Consecutive depth frames without a usable fit before the reference view is
  /// replaced at the held pose. Without it, losing track once loses it forever.
  std::size_t max_hold_frames_ {5};
  /// Time constant of the low-pass on the measured position, in seconds. 0
  /// disables it.
  ///
  /// **The one filter in this pipeline, and it is here because the signal and the
  /// noise are in different bands.** A hand sweeping a room moves at a few
  /// centimetres a second and turns over seconds; the translation estimate's error
  /// is white at the depth rate and, measured on bags/desk1, about 0.09 m per
  /// sample against a real motion of ~0.003 m per sample. Nothing about a better
  /// estimator changes that ratio — a monocular relative-depth network's error is
  /// structured, it does not average down over landmarks, and 300 of them share
  /// most of it. What *does* change it is refusing to believe that the camera
  /// moved 9 cm and back in 57 ms.
  ///
  /// It is a low-pass and not a smoother of a *chain*: the position it filters is
  /// measured absolutely against a keyframe, so lag is the only cost and there is
  /// no integrated noise to suppress. At 0.5 s the lag is ~2.5 cm of hand motion.
  double translation_tau_s_ {0.5};
  double max_point_residual_m_ {0.05};
  double point_reject_fraction_ {0.3};
  ScaleHandling scale_handling_ {ScaleHandling::DivideOut};
  int depth_patch_ {3};
  double depth_patch_spread_ {0.1};
  double min_depth_m_ {0.2};
  double max_depth_m_ {6.0};
  std::size_t history_frames_ {90};
  int preview_quality_ {80};
  double preview_period_s_ {0.1};
  bool publish_tf_ {true};
  std::string odom_frame_ {"odom"};
  std::string base_frame_ {"base_link"};

  // --- Intrinsics -----------------------------------------------------------
  //
  // Written by the CameraInfo callback on an executor thread and read by both
  // workers, hence the mutex. Nominal-looking intrinsics are not substituted if it
  // never arrives: without K there are no rays, and inventing them would produce a
  // pose that is confidently wrong rather than absent.
  std::mutex k_mutex_;
  cv::Matx33d k_ {cv::Matx33d::zeros()};
  bool have_k_ {false};

  // --- Pose -----------------------------------------------------------------
  //
  // `odom <- base_link`, as one transform rather than a rotation and a vector: a
  // 6-DoF increment mixes the two and keeping them apart invites composing them
  // separately, which is wrong the moment the basis has a lever arm.
  std::mutex pose_mutex_;
  cv::Affine3d pose_ {cv::Affine3d::Identity()};

  std::mutex basis_mutex_;
  /// base_link's transform from camera_optical_frame, from TF, looked up once.
  cv::Affine3d base_from_optical_ {cv::Affine3d::Identity()};
  bool have_basis_ {false};
  bool holding_ {false};

  // --- The RGB-D rendezvous --------------------------------------------------
  //
  // Written by the ORB worker, read by the pose worker. A deque under a mutex
  // rather than a lock-free ring: the critical section is a push and a pop of a
  // shared_ptr, the contention is two threads at 59 and 17 Hz, and a lock-free
  // structure here would be complexity bought with nothing.
  std::mutex history_mutex_;
  std::deque<std::shared_ptr<const FrameRecord>> history_;

  /// **The view the current pose is measured *from*, and the one design decision
  /// in P7 that the plan did not call for.**
  ///
  /// The phase describes frame-to-frame estimation with the keyframe store built
  /// but unused. Measured on `bags/desk1` that does not work: each step's
  /// translation error is about the size of one step's real motion, so chaining
  /// them is a random walk. It reported a **117 m** path and **6.9 m** of net
  /// displacement over a 45 s desk sweep, and made the surface worse than
  /// rotation-only odometry rather than better.
  ///
  /// Measuring each frame against a *keyframe* instead fixes it, and the reason is
  /// arithmetic rather than cleverness: over the ~0.5 s a keyframe lasts the real
  /// motion grows by an order of magnitude while the measurement error does not,
  /// and the pose is set **absolutely** from the keyframe's rather than
  /// accumulated onto the last one, so nothing chains. Simulated over a 35 s
  /// sweep: final position error **0.106 m** against **2.70 m** frame-to-frame,
  /// same noise, same estimator.
  ///
  /// So the store is consumed after all. It is still built exactly as P7
  /// specifies — descriptors, bearing rays and 3D landmarks, admitted at ~18
  /// degrees or 0.3 m — and loop closure still needs it for what it is; this only
  /// adds a reader.
  RgbdView reference_;
  KeyframeStore keyframes_;

  // --- Reused buffers, so the hot path does not allocate --------------------
  cv::Mat gray_;
  cv::Mat preview_;
  std::vector<std::uint8_t> jpeg_;

  // --- Counters -------------------------------------------------------------
  std::atomic<std::uint64_t> frames_ {0};
  /// Frames with at least one feature, which is the denominator of the matched
  /// fraction — and `empty_frames_` is the rest. See process_frame() for why the two
  /// are counted apart.
  std::atomic<std::uint64_t> measured_frames_ {0};
  std::atomic<std::uint64_t> empty_frames_ {0};
  /// The **rotation** chain's accounting, on the ORB worker's clock. Kept apart
  /// from the translation's below: the two estimators run at 59 and 17 Hz on
  /// different threads, and one pair of counters for both makes every average in
  /// the stats line a mean over the wrong denominator. Measured once, that turned
  /// a depth scale of 1.0 into a reported 0.18.
  std::atomic<std::uint64_t> pose_ok_ {0};
  std::atomic<std::uint64_t> pose_held_ {0};
  std::atomic<std::uint64_t> shift_ok_ {0};
  std::atomic<std::uint64_t> shift_held_ {0};
  std::atomic<std::uint64_t> previews_ {0};
  std::atomic<double> cost_sum_ms_ {0.0};
  std::atomic<double> cost_max_ms_ {0.0};
  std::atomic<double> detect_sum_ms_ {0.0};
  std::atomic<double> match_sum_ms_ {0.0};
  std::atomic<double> preview_cost_sum_ms_ {0.0};
  std::atomic<double> keypoints_sum_ {0.0};
  std::atomic<double> matched_sum_ {0.0};
  std::atomic<double> residual_sum_ {0.0};

  // P7's own, all on the depth cadence.
  std::atomic<std::uint64_t> depth_frames_ {0};
  /// Depth maps whose ORB output had already fallen out of the history. Counted
  /// rather than warned about once: it is the number that says whether
  /// `history_frames` is deep enough, and a pipeline whose depth stage slows down
  /// would show it here first.
  std::atomic<std::uint64_t> depth_unmatched_ {0};
  std::atomic<double> landmark_sum_ {0.0};
  std::atomic<double> pair_sum_ {0.0};
  /// Keyframes taken because the shared-landmark count collapsed rather than
  /// because the camera moved far enough. A high share means the reference is
  /// being lost to tracking rather than retired on view change.
  std::atomic<std::uint64_t> keyframes_on_loss_ {0};
  /// Keyframes taken because nothing could be fitted for a run of frames. Every
  /// one of these is a stretch of motion that was not estimated at all.
  std::atomic<std::uint64_t> keyframes_on_stall_ {0};
  /// Poses refused for implausible motion rather than for a bad fit. Counted
  /// separately because they mean something different: a bad fit is a camera
  /// looking at nothing, and this is a fit that was confident and wrong.
  std::atomic<std::uint64_t> implausible_ {0};
  /// Consecutive holds, pose-worker-local.
  std::size_t holds_ {0};
  /// Filter state: the last accepted measurement's stamp, and whether there is one.
  std::int64_t last_shift_ns_ {0};
  bool have_shift_ {false};
  std::atomic<double> point_residual_sum_ {0.0};
  /// Mean reprojection error of the PnP inliers, in **pixels** — a unit this
  /// project has a calibration for, unlike the metres everything else is in.
  std::atomic<double> reprojection_sum_px_ {0.0};
  std::atomic<double> inlier_sum_ {0.0};
  /// The depth network's per-step scale, averaged. It is the number that says
  /// whether dividing it out is doing anything at all — 1.000 would mean the
  /// network is steady and the choice is moot.
  std::atomic<double> scale_sum_ {0.0};
  std::atomic<double> pose_cost_sum_ms_ {0.0};
  /// Path length, in metres, summed over every accepted increment. The headline
  /// number of P7: rotation-only reports exactly 0.0 here whatever the camera did.
  std::atomic<double> trajectory_m_ {0.0};

  std::uint64_t last_frames_ {0};
  /// Windowed denominators for the two `/pipeline/stats` rows. A rate is always a
  /// delta over a span here, never a total over an uptime: an average since
  /// startup only ever moves slowly, so a stage that stopped an instant ago still
  /// reads healthy — which is precisely what a dashboard is for noticing.
  std::uint64_t last_depth_frames_ {0};
  std::size_t last_dropped_ {0};
  rclcpp::Time last_log_;
  rclcpp::Time last_preview_;
  bool have_preview_time_ {false};
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__KEYPOINT_NODE_HPP_
