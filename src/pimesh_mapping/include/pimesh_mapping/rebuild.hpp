#ifndef PIMESH_MAPPING__REBUILD_HPP_
#define PIMESH_MAPPING__REBUILD_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"
#include "pimesh_mapping/tsdf_volume.hpp"

namespace pimesh_mapping
{

/// #12's P18: the volume rebuilt at the poses the pose graph corrected.
///
/// **Why this is not optional.** A TSDF bakes the pose a frame was integrated at into
/// every voxel it touched. A loop closure corrects the trajectory and leaves the
/// surface where the drifted poses put it — the room seen on the first pass and the
/// same room seen on the return, drifted, both in the volume, a *ghost* one
/// truncation band apart (test_tsdf_volume pins that a jump further than the band
/// leaves one). So the frames are remembered, and when the correction moves them far
/// enough the volume is integrated again from scratch at the corrected poses.

/// One frame, as it was integrated: the depth **after** scale alignment, so a rebuild
/// integrates the same numbers the live volume did and differs only in the pose.
struct RememberedFrame
{
  std::int64_t stamp_ns {0};
  /// CV_16UC1, millimetres, 0 for no reading. At the memory's stored resolution.
  cv::Mat depth_mm;
  /// CV_8UC3 at the same size, or empty for a frame integrated without colour.
  cv::Mat bgr;
  /// The intrinsics scaled to the stored resolution.
  cv::Matx33d k {cv::Matx33d::eye()};
  /// `odom <- camera_optical_frame` at the frame's stamp: the pose a correction is
  /// applied to.
  cv::Affine3d odom_from_camera {cv::Affine3d::Identity()};
  /// `map <- odom` as TF had it when the frame was integrated live. The rebuild's
  /// trigger is how far the pose graph has moved a frame *from here*.
  cv::Affine3d map_from_odom_used {cv::Affine3d::Identity()};
};

/// A bounded memory of integrated frames, thinned rather than truncated.
///
/// **Thinned, not a ring buffer.** A ring buffer that drops the oldest frame keeps
/// the end of a session and forgets its start — and a loop closure is precisely the
/// moment the start matters, because the return is being joined to it. So when full
/// it keeps every other frame and from then on admits one frame in two: the density
/// halves across the whole session instead of the history being cut. Each halving
/// is counted, because a rebuild from a memory thinned three times is a volume with
/// an eighth of the observations and a surface that looks cleaner for it — the plan's
/// first false green.
class FrameMemory
{
public:
  struct Config
  {
    /// ~0.29 MB a frame at 320x180: 600 frames is ~170 MB.
    std::size_t max_frames {600};
    /// The stored resolution is the integrated one divided by this.
    int downsample {4};
  };

  explicit FrameMemory(const Config & config);

  /// Offer one integrated frame. `depth_m` is CV_32FC1 in metres, `bgr` CV_8UC3 of
  /// the same size or empty, `k` for that size. Returns whether it was kept.
  bool offer(
    std::int64_t stamp_ns, const cv::Mat & depth_m, const cv::Mat & bgr, const cv::Matx33d & k,
    const cv::Affine3d & odom_from_camera, const cv::Affine3d & map_from_odom_used);

  const std::vector<RememberedFrame> & frames() const {return frames_;}
  std::size_t size() const {return frames_.size();}
  /// How many times the memory has halved, and the stride it admits at now.
  std::size_t halvings() const {return halvings_;}
  std::size_t stride() const {return stride_;}
  std::size_t bytes() const;
  void clear();

private:
  Config config_;
  std::vector<RememberedFrame> frames_;
  std::size_t stride_ {1};
  std::size_t offered_ {0};
  std::size_t halvings_ {0};
};

/// Keyframe stamps and each keyframe's `map <- odom` correction, in stamp order —
/// what `/pose_graph/corrections` carries.
using Corrections = std::vector<std::pair<std::int64_t, cv::Affine3d>>;

/// The correction a frame at `stamp_ns` takes: the one of the **latest keyframe at or
/// before it**, not an interpolation. The tracker poses every frame against its
/// reference keyframe, so a frame is rigidly attached to that keyframe and moves with
/// it; interpolating would invent a pose the graph never produced. Identity before
/// the first keyframe and with no corrections at all.
cv::Affine3d correction_at(const Corrections & corrections, std::int64_t stamp_ns);

/// How far the corrections would move the remembered frames from where they were
/// integrated: the largest translation and rotation between each frame's live pose
/// and its corrected one. The rebuild's trigger.
struct Shift
{
  double max_m {0.0};
  double max_deg {0.0};
};
Shift pose_shift(const std::vector<RememberedFrame> & frames, const Corrections & corrections);

struct RebuildResult
{
  std::unique_ptr<TsdfVolume> volume;
  /// Frames integrated. **Asserted equal to the memory's size by gates/rebuild.sh** —
  /// a rebuild that skipped frames produces a cleaner surface for the wrong reason.
  std::size_t integrated {0};
  std::size_t blocks_refused {0};
};

/// Integrate every remembered frame, in order, into a new volume with `options`, each
/// at `correction_at(stamp) * odom_from_camera`. With empty or all-identity
/// corrections this is the memory re-integrated at the poses it was first integrated
/// at — gates/rebuild.sh's control, the same code path one input apart.
///
/// The allocation stride is divided by the memory's downsample, because it is in
/// pixels and the frames are smaller: kept as it was it would sample the image four
/// times more sparsely than the live volume did and miss thin surfaces.
RebuildResult rebuild_volume(
  const std::vector<RememberedFrame> & frames, const Corrections & corrections,
  TsdfVolume::Options options, int downsample);

/// The same, at poses given outright — one per frame, `map <- optical`. What the
/// ground-truth reference of tools/gates/rebuild.sh is built with: the remembered
/// frames at motion-capture poses. Frames whose pose is not finite are skipped and
/// not counted, so `integrated` says how many really went in.
RebuildResult rebuild_volume_at(
  const std::vector<RememberedFrame> & frames, const std::vector<cv::Affine3d> & poses,
  TsdfVolume::Options options, int downsample);

/// Write the memory and the corrections it was rebuilt with to `dir`, for the
/// offline ground-truth comparison: `index.txt` (one line per frame: stamp, K, the
/// odometry pose, the correction used live, and file names), 16-bit PNG depth, PNG
/// colour, and `corrections.txt`. **Written to `dir.partial` and renamed over `dir`**,
/// so a session killed mid-write leaves the previous complete dump rather than half
/// of this one — a shorter memory is a plausible one. Empty string on success,
/// otherwise the reason.
/// `meta.txt` carries the volume options and the downsample the memory was taken
/// with, so the offline rebuild uses the live volume's, not a second copy typed on a
/// command line — two copies of one value agree only until somebody edits one.
std::string write_memory(
  const std::string & dir, const std::vector<RememberedFrame> & frames,
  const Corrections & corrections, const TsdfVolume::Options & options, int downsample);
/// Read one back. Refuses rather than guessing: a missing image, a short line, or an
/// index that names no frames is an error, not an empty memory.
std::string read_memory(
  const std::string & dir, std::vector<RememberedFrame> & frames, Corrections & corrections,
  TsdfVolume::Options & options, int & downsample);

}  // namespace pimesh_mapping

#endif  // PIMESH_MAPPING__REBUILD_HPP_
