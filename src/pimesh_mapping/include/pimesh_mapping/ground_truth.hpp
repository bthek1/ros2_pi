#ifndef PIMESH_MAPPING__GROUND_TRUTH_HPP_
#define PIMESH_MAPPING__GROUND_TRUTH_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"
#include "pimesh_mapping/rebuild.hpp"
#include "pimesh_mapping/tsdf_volume.hpp"

namespace pimesh_mapping
{

/// #12's P18 instrument: where should the surface be, according to something this
/// project did not produce?
///
/// **Why it exists.** The rebuild's first measure was a volume ray-cast against the
/// frames it was built from, and it answered "better" twice and "worse" twice
/// (2026-09-30), because a drifted volume agrees with itself as well as a correct one
/// does. So the reference here is the *same remembered frames* integrated at
/// **motion-capture** poses: the depth readings — and the depth network's errors —
/// identical in every volume compared, and only the poses differing. What is left
/// between an arm and the reference is what the poses did to the surface.

/// A similarity: `to = s * R * from + t`.
struct Sim3
{
  double s {1.0};
  cv::Matx33d r {cv::Matx33d::eye()};
  cv::Vec3d t {0.0, 0.0, 0.0};
  bool ok {false};
};

/// Umeyama's least-squares similarity taking `from` onto `to`. `ok` false with fewer
/// than three pairs or a degenerate (collinear) `from`: an alignment from a line has a
/// free rotation about it, and any answer would be a guess.
Sim3 umeyama(const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to);

/// A pose carried through a similarity: positions transform as points, orientations
/// are only rotated — a camera does not change shape when the world is scaled.
cv::Affine3d apply(const Sim3 & sim, const cv::Affine3d & pose);

/// TUM `groundtruth.txt`: stamps in nanoseconds, poses `world <- optical`, in order.
std::vector<std::pair<std::int64_t, cv::Affine3d>> load_tum_groundtruth(const std::string & path);
/// The ground-truth pose nearest `stamp_ns`, if one is within `tolerance_ns`
/// (default 20 ms, two samples of a 100 Hz capture). False otherwise.
bool groundtruth_at(
  const std::vector<std::pair<std::int64_t, cv::Affine3d>> & gt, std::int64_t stamp_ns,
  cv::Affine3d & pose, std::int64_t tolerance_ns = 20000000);

/// Two volumes ray-cast from the same poses and compared: the median paired depth gap
/// (metres) and the median fraction of `reference`'s pixels the other agrees with
/// within `tolerance` of depth. -1 for either if no view overlapped.
struct SurfaceComparison
{
  double gap_m {-1.0};
  double agree {-1.0};
  std::size_t views {0};
};
SurfaceComparison compare_surfaces(
  const TsdfVolume & volume, const TsdfVolume & reference, const std::vector<cv::Affine3d> & poses,
  const cv::Matx33d & k, cv::Size size, double min_overlap, double tolerance);

/// One arm scored against ground truth.
struct ArmScore
{
  std::size_t frames {0};
  /// Frames with a ground-truth pose within tolerance — the only ones used anywhere
  /// below. A low count is reported, never quietly accepted.
  std::size_t judged {0};
  /// The similarity aligning ground truth onto this arm, and the arm's position error
  /// after it — an ATE over the remembered frames, the sanity check that this arm is
  /// the trajectory it claims to be.
  Sim3 alignment;
  double ate_m {-1.0};
  /// The arm's volume against the reference volume, from the reference's poses.
  SurfaceComparison surface;
};

/// Score the frames at `poses` (one per frame, the arm's `map <- optical`) against the
/// same frames rebuilt at ground truth aligned onto them. **Each arm is aligned to its
/// own best fit**, so neither is compared in a frame chosen for the other.
ArmScore score_arm(
  const std::vector<RememberedFrame> & frames, const std::vector<cv::Affine3d> & poses,
  const std::vector<std::pair<std::int64_t, cv::Affine3d>> & groundtruth,
  const TsdfVolume::Options & options, int downsample, double min_overlap, double tolerance);

}  // namespace pimesh_mapping

#endif  // PIMESH_MAPPING__GROUND_TRUTH_HPP_
