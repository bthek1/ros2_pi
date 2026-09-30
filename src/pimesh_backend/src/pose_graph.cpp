#include "pimesh_backend/pose_graph.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

#include "Eigen/Core"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/edge_se3_expmap.h"
#include "g2o/types/sba/vertex_se3_expmap.h"

namespace pimesh_backend
{
namespace
{

/// chi-squared at 95% for six degrees of freedom: a whole relative pose.
constexpr double kChi2Pose = 12.592;

g2o::SE3Quat to_se3(const cv::Affine3d & a)
{
  const cv::Matx33d r = a.rotation();
  const cv::Vec3d t = a.translation();
  Eigen::Matrix3d rotation;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {rotation(i, j) = r(i, j);}
  }
  return g2o::SE3Quat(rotation, Eigen::Vector3d(t[0], t[1], t[2]));
}

cv::Affine3d to_affine(const g2o::SE3Quat & s)
{
  const Eigen::Matrix3d r = s.rotation().toRotationMatrix();
  const Eigen::Vector3d t = s.translation();
  return cv::Affine3d(
    cv::Matx33d(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2)),
    cv::Vec3d(t.x(), t.y(), t.z()));
}

/// Information for a relative pose: g2o's SE3Quat::log orders its error `[omega,
/// upsilon]` — rotation first, in radians — so the diagonal is rotation's inverse
/// variance three times and then translation's. Pinned by test_pose_graph: the two
/// halves swapped is a graph that trusts metres as if they were radians.
Eigen::Matrix<double, 6, 6> information(double rot_sigma_rad, double trans_sigma_m)
{
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
  for (int i = 0; i < 3; ++i) {
    info(i, i) = 1.0 / (rot_sigma_rad * rot_sigma_rad);
    info(i + 3, i + 3) = 1.0 / (trans_sigma_m * trans_sigma_m);
  }
  return info;
}

double rotation_deg(const cv::Matx33d & r)
{
  const double c = std::clamp((cv::trace(r) - 1.0) / 2.0, -1.0, 1.0);
  return std::acos(c) * 180.0 / CV_PI;
}

}  // namespace

PoseGraph::PoseGraph(const PoseGraphConfig & config)
: config_(config) {}

void PoseGraph::add_keyframe(std::int64_t stamp_ns, const cv::Affine3d & odom_from_camera)
{
  Node node;
  node.stamp_ns = stamp_ns;
  node.odom = odom_from_camera;
  node.corrected = correction_ * odom_from_camera;
  keyframes_.push_back(node);
}

std::ptrdiff_t PoseGraph::index_of(std::int64_t stamp_ns) const
{
  const auto it = std::lower_bound(
    keyframes_.begin(), keyframes_.end(), stamp_ns,
    [](const Node & n, std::int64_t s) {return n.stamp_ns < s;});
  if (it == keyframes_.end() || it->stamp_ns != stamp_ns) {return -1;}
  return it - keyframes_.begin();
}

bool PoseGraph::add_loop(
  std::int64_t query_stamp_ns, std::int64_t candidate_stamp_ns,
  const cv::Affine3d & query_from_candidate)
{
  const std::ptrdiff_t q = index_of(query_stamp_ns);
  const std::ptrdiff_t c = index_of(candidate_stamp_ns);
  if (q < 0 || c < 0 || q == c) {return false;}
  loops_.push_back(Loop{static_cast<std::size_t>(q), static_cast<std::size_t>(c), query_from_candidate});
  return true;
}

cv::Affine3d PoseGraph::map_from_odom() const
{
  return correction_;
}

std::vector<std::pair<std::int64_t, cv::Affine3d>> PoseGraph::trajectory() const
{
  std::vector<std::pair<std::int64_t, cv::Affine3d>> out;
  out.reserve(keyframes_.size());
  for (const Node & n : keyframes_) {out.emplace_back(n.stamp_ns, n.corrected);}
  return out;
}

PoseGraphResult PoseGraph::optimize()
{
  PoseGraphResult result;
  result.keyframes = keyframes_.size();
  result.loops = loops_.size();
  if (keyframes_.size() < 2) {
    result.refusal = "fewer than two keyframes";
    return result;
  }
  // Without a loop the optimum is the odometry chain exactly, and running the solver
  // to confirm it would only add its round-off to every pose.
  if (loops_.empty()) {
    result.refusal = "no loop";
    return result;
  }

  // Six-dimensional poses and no landmarks: the 6_3 block solver with nothing
  // marginalised, so its landmark half is simply empty. One fixed vertex — the first keyframe — is the gauge, and
  // the graph always has it, so the free-pose abort test_local_ba pins cannot happen
  // here: two keyframes minimum means one free.
  using Block = g2o::BlockSolver_6_3;
  auto linear = std::make_unique<g2o::LinearSolverEigen<Block::PoseMatrixType>>();
  auto * algorithm = new g2o::OptimizationAlgorithmLevenberg(std::make_unique<Block>(std::move(linear)));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algorithm);

  // Vertices hold camera_from_map, g2o's SBA convention.
  for (std::size_t i = 0; i < keyframes_.size(); ++i) {
    auto * v = new g2o::VertexSE3Expmap();
    v->setId(static_cast<int>(i));
    v->setEstimate(to_se3(keyframes_[i].corrected.inv()));
    v->setFixed(i == 0);
    optimizer.addVertex(v);
  }

  // EdgeSE3Expmap's error is log(v2^-1 * C * v1), zero when C = v2 * v1^-1. With
  // vertices camera_from_map, that is cam2_from_map * map_from_cam1 = cam2_from_cam1:
  // the measurement maps points from vertex 1's frame into vertex 2's.
  for (std::size_t i = 1; i < keyframes_.size(); ++i) {
    const Node & a = keyframes_[i - 1];
    const Node & b = keyframes_[i];
    const double dt = std::abs(static_cast<double>(b.stamp_ns - a.stamp_ns)) * 1e-9;
    const double rot = std::max(config_.odom_rot_floor_deg, config_.odom_rot_deg_per_s * dt);
    const double trans = std::max(config_.odom_trans_floor_m, config_.odom_trans_m_per_s * dt);
    auto * e = new g2o::EdgeSE3Expmap();
    e->setVertex(0, optimizer.vertex(static_cast<int>(i - 1)));
    e->setVertex(1, optimizer.vertex(static_cast<int>(i)));
    e->setMeasurement(to_se3(b.odom.inv() * a.odom));
    e->setInformation(information(rot * CV_PI / 180.0, trans));
    optimizer.addEdge(e);
  }
  std::vector<g2o::EdgeSE3Expmap *> loop_edges;
  for (const Loop & l : loops_) {
    auto * e = new g2o::EdgeSE3Expmap();
    e->setVertex(0, optimizer.vertex(static_cast<int>(l.candidate)));
    e->setVertex(1, optimizer.vertex(static_cast<int>(l.query)));
    e->setMeasurement(to_se3(l.query_from_candidate));
    e->setInformation(information(config_.loop_rot_deg * CV_PI / 180.0, config_.loop_trans_m));
    // Huber on closures only: odometry edges are the chain, and a closure is the one
    // kind of edge that can be wrong by a whole room. The kernel bounds how hard one
    // bad closure can pull.
    auto * kernel = new g2o::RobustKernelHuber();
    kernel->setDelta(std::sqrt(kChi2Pose));
    e->setRobustKernel(kernel);
    optimizer.addEdge(e);
    loop_edges.push_back(e);
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  result.chi2_before = optimizer.activeRobustChi2();
  result.iterations = optimizer.optimize(config_.iterations);
  optimizer.computeActiveErrors();
  result.chi2_after = optimizer.activeRobustChi2();
  for (g2o::EdgeSE3Expmap * e : loop_edges) {
    e->computeError();
    if (e->chi2() > kChi2Pose) {++result.inconsistent_loops;}
  }

  for (std::size_t i = 0; i < keyframes_.size(); ++i) {
    const auto * v = static_cast<const g2o::VertexSE3Expmap *>(optimizer.vertex(static_cast<int>(i)));
    keyframes_[i].corrected = to_affine(v->estimate()).inv();
  }
  const cv::Affine3d odom_newest = keyframes_.back().odom;
  const cv::Affine3d after = keyframes_.back().corrected;
  correction_ = after * odom_newest.inv();
  result.correction_m = cv::norm(after.translation() - odom_newest.translation());
  result.correction_deg = rotation_deg(after.rotation().t() * odom_newest.rotation());
  result.ran = true;
  return result;
}

}  // namespace pimesh_backend
