#ifndef PIMESH_PERCEPTION__KEYFRAME_STORE_HPP_
#define PIMESH_PERCEPTION__KEYFRAME_STORE_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"

namespace pimesh_perception
{

/// One remembered view: what was seen, where it was in space, and from where.
///
/// Three descriptions of the same features, and each is here because a different
/// consumer needs a different one:
///
///   - `descriptors` are what a *relocaliser* matches against. They are the only
///     part that survives the camera going away and coming back.
///   - `landmarks` are the 3D points in this keyframe's own optical frame, which
///     is what a 3D-3D fit against a rediscovered view needs.
///   - `bearings` are the unit rays, which is what a bearing-only fit needs — and
///     they are *not* derivable from the landmarks, because a landmark only exists
///     where the depth map had a usable reading. Roughly half of a frame's corners
///     have a bearing and no landmark, and throwing those away would discard the
///     features on distant geometry, which are exactly the ones that constrain
///     rotation best.
struct Keyframe
{
  std::int64_t stamp_ns {0};
  /// Where the camera was when this was taken — `odom <- camera_optical_frame`.
  cv::Affine3d odom_from_camera {cv::Affine3d::Identity()};

  /// CV_8U, N rows of 32 bytes. Row i belongs to track_ids[i].
  cv::Mat descriptors;
  std::vector<cv::Vec3d> bearings;
  std::vector<std::int32_t> track_ids;

  /// Landmarks, in this keyframe's own optical frame. `landmark_row[i]` is the row
  /// of `descriptors` that `landmarks[i]` came from — a parallel index rather than
  /// a sentinel value, because "this feature has no depth" and "this feature is at
  /// the origin" must not have the same spelling.
  std::vector<cv::Vec3d> landmarks;
  std::vector<std::int32_t> landmark_row;

  /// Roughly what this costs to keep. Reported rather than assumed: the budget in
  /// P7 is ~16 kB each, and a store that quietly grew to ten times that would be
  /// a memory leak with a plan behind it.
  std::size_t bytes() const;
};

/// A bounded history of keyframes, admitted on how far the camera has moved.
///
/// **Nothing consumes this yet, and that is deliberate rather than an oversight.**
/// P7's odometry is frame-to-frame; this store is what the deferred loop-closure
/// work needs to exist before it can start, and it is built alongside the 6-DoF
/// estimator because that is where the landmarks already are. Building it later
/// would mean either re-deriving the same landmarks or carrying a second copy of
/// the tracker.
///
/// **The admission test is on *view change*, not on time**, and the two
/// thresholds are one condition each rather than a combined score: a camera
/// panning on the spot travels no distance and still sees an entirely different
/// room, and a camera sliding along a wall turns through no angle and still does.
/// Either alone is enough to make the last keyframe a poor description of what is
/// in front of the lens now.
class KeyframeStore
{
public:
  struct Config
  {
    /// ~18 degrees of view change, in radians.
    double min_angle_rad {0.314};
    double min_translation_m {0.3};
    /// The ceiling. At ~16 kB each, 500 keyframes is ~8 MB — a bound chosen so
    /// that a session left running overnight cannot become a memory question,
    /// which is the failure a store with no ceiling eventually has.
    std::size_t max_keyframes {500};
  };

  explicit KeyframeStore(const Config & config)
  : config_(config) {}

  /// Whether a camera at `pose` is far enough from the newest keyframe to warrant
  /// another. Always true when the store is empty.
  bool would_insert(const cv::Affine3d & pose) const;

  /// Insert if would_insert() says so. Returns whether it did.
  ///
  /// The two halves are separate so a caller can skip *building* a keyframe it is
  /// about to be told to throw away — the descriptors are a copy of a 16 kB matrix
  /// and this is asked of every frame.
  bool maybe_insert(Keyframe frame);

  std::size_t size() const {return frames_.size();}
  std::size_t bytes() const;
  std::size_t dropped() const {return dropped_;}
  bool empty() const {return frames_.empty();}
  /// The newest. Undefined if empty() — callers check.
  const Keyframe & latest() const {return frames_.back();}
  const std::deque<Keyframe> & frames() const {return frames_;}
  const Config & config() const {return config_;}

  void clear();

private:
  Config config_;
  std::deque<Keyframe> frames_;
  std::size_t dropped_ {0};
};

/// The angle, in radians, between two orientations.
///
/// `acos((trace(A^T B) - 1) / 2)`, clamped before the acos — a trace of
/// 3.0000000002 is ordinary floating point and acos of the value it produces is
/// NaN, which then compares false against every threshold and silently stops the
/// store from ever admitting anything.
double angle_between(const cv::Matx33d & a, const cv::Matx33d & b);

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__KEYFRAME_STORE_HPP_
