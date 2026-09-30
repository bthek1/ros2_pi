#ifndef PIMESH_BACKEND__POSE_GRAPH_HPP_
#define PIMESH_BACKEND__POSE_GRAPH_HPP_

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"

namespace pimesh_backend
{

/// The pose graph's noise model, and every number in it was measured on TUM fr1/desk
/// against motion capture on 2026-09-30 rather than chosen.
///
/// **Odometry edges get less certain with the time they span**, because the tracker's
/// error is drift: `evo_rpe --pose_relation angle_deg` put its rotation error at a
/// median 6.7 degrees over ~1 s and 26 over ~5 s, and P11's RPE its translation error
/// at ~0.15 m over 1 s. **Loop edges carry a fixed uncertainty**: the median error of
/// P16's closures against the same ground truth, 2.3-3.7 degrees and ~0.05 m.
///
/// The ratio between the two is what decides how far a closure is allowed to bend
/// the chain, so getting either wrong by a constant factor does not fail loudly — it
/// makes the graph trust the wrong half.
struct PoseGraphConfig
{
  double odom_rot_deg_per_s {7.0};
  double odom_trans_m_per_s {0.15};
  /// Floors, so an edge between two keyframes 10 ms apart is not treated as exact.
  double odom_rot_floor_deg {0.5};
  double odom_trans_floor_m {0.01};
  double loop_rot_deg {3.0};
  double loop_trans_m {0.06};
  int iterations {20};
};

struct PoseGraphResult
{
  /// False with a `refusal` when g2o was not run: nothing to correct without a loop,
  /// and nothing to optimise with fewer than two keyframes.
  bool ran {false};
  const char * refusal {""};
  int iterations {0};
  double chi2_before {0.0};
  double chi2_after {0.0};
  std::size_t keyframes {0};
  std::size_t loops {0};
  /// Loop edges whose residual after the solve is past chi-squared at 95% for six
  /// degrees of freedom: a closure the rest of the graph disagrees with. Reported,
  /// because a closure the graph could not reconcile is either a false one P16 let
  /// through or odometry worse than its model says.
  std::size_t inconsistent_loops {0};
  /// How far the solve moved the newest keyframe from where odometry put it.
  double correction_m {0.0};
  double correction_deg {0.0};
};

/// Keyframe poses joined by odometry and by loop closures, and the least-squares
/// compromise between them (#12's P17).
///
/// Each keyframe is a vertex. Consecutive keyframes are joined by the relative pose
/// odometry measured between them; a closure (P16) joins two keyframes far apart in
/// time by the relative pose place recognition measured. With no closure the
/// optimum *is* the odometry chain — every edge satisfied exactly — which is what
/// makes `loop_closure:=false` a control one parameter apart. With one, the drift
/// accumulated around the loop is spread back along the chain in proportion to each
/// edge's uncertainty.
///
/// Pure: no ROS, no thread. SE(3), not Sim(3): every keyframe's landmarks carry a
/// depth reading, so scale is observed at every keyframe, not only at the start.
class PoseGraph
{
public:
  explicit PoseGraph(const PoseGraphConfig & config);

  /// Keyframes arrive in time order. Its corrected pose starts at the newest
  /// correction applied to its odometry pose, so a keyframe added after a closure is
  /// in the corrected frame at once rather than jumping back to raw odometry.
  void add_keyframe(std::int64_t stamp_ns, const cv::Affine3d & odom_from_camera);
  /// A closure between two keyframes already added. False if either is unknown.
  /// `query_from_candidate` maps points from the candidate's optical frame into the
  /// query's — P16's PlaceMatch, unchanged.
  bool add_loop(
    std::int64_t query_stamp_ns, std::int64_t candidate_stamp_ns,
    const cv::Affine3d & query_from_candidate);

  PoseGraphResult optimize();

  /// `map <- odom`: the correction at the newest keyframe, which is what the TF tree
  /// publishes between the two frames. Identity until a closure has been optimised.
  cv::Affine3d map_from_odom() const;
  /// Every keyframe's stamp and corrected pose, in order.
  std::vector<std::pair<std::int64_t, cv::Affine3d>> trajectory() const;
  std::size_t size() const {return keyframes_.size();}
  std::size_t loops() const {return loops_.size();}

private:
  struct Node
  {
    std::int64_t stamp_ns {0};
    cv::Affine3d odom {cv::Affine3d::Identity()};
    cv::Affine3d corrected {cv::Affine3d::Identity()};
  };
  struct Loop
  {
    std::size_t query {0};
    std::size_t candidate {0};
    cv::Affine3d query_from_candidate {cv::Affine3d::Identity()};
  };
  std::ptrdiff_t index_of(std::int64_t stamp_ns) const;

  PoseGraphConfig config_;
  /// `map <- odom` as of the last solve. Stored, not recomputed from the newest
  /// keyframe as `corrected * odom^-1`: that product is identity only to round-off,
  /// and applied to every new keyframe it compounds — measured by the first run of
  /// test_pose_graph at 1e-14 per keyframe, on a control that must be exact.
  cv::Affine3d correction_ {cv::Affine3d::Identity()};
  std::vector<Node> keyframes_;
  std::vector<Loop> loops_;
};

}  // namespace pimesh_backend

#endif  // PIMESH_BACKEND__POSE_GRAPH_HPP_
