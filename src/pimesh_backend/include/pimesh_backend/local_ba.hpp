#ifndef PIMESH_BACKEND__LOCAL_BA_HPP_
#define PIMESH_BACKEND__LOCAL_BA_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"
#include "pimesh_backend/map.hpp"

namespace pimesh_backend
{

/// One keyframe in a bundle-adjustment window.
struct BaKeyframe
{
  KeyframeId id {kNoKeyframe};
  cv::Affine3d map_from_camera {cv::Affine3d::Identity()};
  cv::Matx33d k {cv::Matx33d::eye()};
  /// Held where it is. The window's own keyframes move; the keyframes outside it
  /// that also see the window's points are fixed, and they are what stops the
  /// whole window sliding and rotating freely — BA's gauge.
  bool fixed {false};
};

struct BaPoint
{
  PointId id {kNoPoint};
  cv::Vec3d position {0.0, 0.0, 0.0};
};

/// One sighting: keyframe `keyframe` saw point `point` at `pixel`, and — if `depth`
/// is positive — the depth network put it `depth` metres down that keyframe's
/// optical axis. Indices are into the problem's own vectors, not map ids.
struct BaObservation
{
  std::size_t keyframe {0};
  std::size_t point {0};
  std::int32_t row {-1};
  cv::Point2f pixel;
  double depth {0.0};
};

/// A snapshot of the window, taken under the map's lock and solved outside it.
struct BaProblem
{
  std::vector<BaKeyframe> keyframes;
  std::vector<BaPoint> points;
  std::vector<BaObservation> observations;
};

struct BaConfig
{
  /// Levenberg-Marquardt iterations in each of the two rounds — before and after
  /// outliers are switched off. ORB-SLAM's 5 + 10.
  int first_iterations {5};
  int second_iterations {10};
  /// **The depth prior's standard deviation, relative to the depth**, and the one
  /// number here that came out of this project rather than out of ORB-SLAM. P14
  /// measured Depth Anything's scale differing by ~15% between consecutive
  /// keyframes (`align_dev` on the `stats map` line, fr1/desk, 2026-09-29), so a
  /// depth reading is worth about that much. 0 switches the prior off and makes
  /// this monocular BA, whose only scale anchor is the fixed keyframes — and P14
  /// showed what a map does with its scale when the network stops anchoring it.
  double depth_sigma_rel {0.15};
  /// **One sigma of each keyframe's depth-map scale, as a log.** 0 is the model
  /// above: every reading an independent prior of `depth_sigma_rel`. Positive gives
  /// each keyframe a scale `s_k` that every one of its readings shares, so a reading
  /// says the point is `s_k * d` down the axis, and `log s_k` carries a prior of this
  /// width about 0.
  ///
  /// **Why that is the right model and the per-reading prior is not**: what P14
  /// measured is a depth map breathing *as a whole* — `align_dev` is a median ratio
  /// over a keyframe's associations. Error shared by three hundred readings does not
  /// average down over them, so an independent 15% on each is at once too loose for
  /// the shape within one depth map and blind to the correlation across it. With the
  /// shared factor explained, what is left per reading is the within-frame warp,
  /// `depth_point_sigma_rel`.
  ///
  /// The prior is about 0 every solve and the solved scale is **not** written back
  /// into the readings: re-centring each solve on the last one's answer is
  /// inheritance, and P14 measured what inheritance of a scale does (a fitted Sim(3)
  /// scale of 0.699). The network's reading stays the anchor.
  double depth_scale_sigma {0.0};
  /// One sigma of a reading **once its keyframe's scale is modelled**, relative to
  /// the depth. Used only when `depth_scale_sigma` is positive. P12 measured a
  /// within-frame spread of 5.3% over a patch of wall — the warp that survives
  /// dividing out one scale.
  double depth_point_sigma_rel {0.05};
  /// Pixel noise of a corner, one sigma. The information matrix is its inverse
  /// square; P9's calibration reprojects at 0.4955 px, and ORB's corners at scale are
  /// coarser than a chessboard's.
  double pixel_sigma {1.0};
};

struct BaResult
{
  /// False with a `refusal` when the problem was not handed to g2o at all. **It is
  /// never handed a window with no free keyframe**: this g2o is built with asserts
  /// on, and BlockSolver's resize asserts a pose count above zero — measured
  /// 2026-09-29, the process aborts. In a component that is the container, and the
  /// TSDF in it.
  bool ran {false};
  const char * refusal {""};
  /// Iterations g2o actually took, both rounds together. **Asserted > 0 by
  /// gates/ba.sh**: a solver that returns at iteration 0 produces a run identical to
  /// no BA at all.
  int iterations {0};
  /// Robust cost before and after, summed over the edges that were active.
  double chi2_before {0.0};
  double chi2_after {0.0};
  std::size_t free_keyframes {0};
  std::size_t fixed_keyframes {0};
  std::size_t edges {0};
  std::size_t depth_edges {0};
  /// Keyframes that got a scale vertex: those with at least one depth reading, and
  /// only when `depth_scale_sigma` is positive. 0 otherwise.
  std::size_t scale_vertices {0};
  /// Parallel to the problem's keyframes: each depth map's solved scale, and **0.0
  /// where it had no scale vertex** — not 1.0, which is what a perfectly scaled depth
  /// map solves to; an unmeasured value and a good one must not share a spelling.
  /// **Reported, never applied** — see BaConfig::depth_scale_sigma. What it
  /// is for is `ba_scale_dev` on the backend's stats line: if the solved scales sit
  /// at 1.0 the vertices are doing nothing, and if they spread by P14's 15% they are
  /// absorbing exactly what P14 said was there.
  std::vector<double> depth_scale;
  /// Parallel to the problem's vectors: the solved poses and positions.
  std::vector<cv::Affine3d> map_from_camera;
  std::vector<cv::Vec3d> positions;
  /// Parallel to the problem's observations: outliers after the second round, which
  /// the map drops as observations.
  std::vector<std::uint8_t> outlier;
};

/// Local bundle adjustment: refine the window's keyframe poses and the positions of
/// every point they see, together, against every observation of those points.
///
/// **What it minimises is internal, and that is the plan's first false green for
/// this phase.** A lower chi-squared says the poses and points agree with the
/// pixels better; it says nothing about whether they agree with the room — the
/// residual-versus-truth distinction P7 paid for, one level up. gates/ba.sh judges
/// the ATE, not the cost.
///
/// Pure: no map, no lock, no thread. The caller snapshots, solves, writes back.
BaResult solve_local_ba(const BaProblem & problem, const BaConfig & config);

}  // namespace pimesh_backend

#endif  // PIMESH_BACKEND__LOCAL_BA_HPP_
