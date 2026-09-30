#include "pimesh_backend/local_ba.hpp"

#include <cmath>
#include <istream>
#include <memory>
#include <ostream>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "g2o/core/base_fixed_sized_edge.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/sba/edge_project_stereo_xyz.h"
#include "g2o/types/sba/edge_project_xyz.h"
#include "g2o/types/sba/vertex_se3_expmap.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"

namespace pimesh_backend
{
namespace
{

/// chi-squared at 95% for 2 and 3 degrees of freedom: a monocular and a stereo
/// residual. ORB-SLAM's thresholds, and the Huber kernel's width is their root —
/// past it an edge's cost grows linearly, so one bad correspondence pulls with a
/// bounded force instead of a quadratic one.
constexpr double kChi2Mono = 5.991;
constexpr double kChi2Depth = 7.815;
/// And for 1 degree of freedom: a depth reading on its own, when it is an edge of its
/// own beside the pixel's rather than the third row of a stereo residual.
constexpr double kChi2Scalar = 3.841;

/// A depth map's scale, as its log so that it stays positive and a prior on it is
/// symmetric in ratio — 15% too near and 15% too far cost the same.
class VertexLogScale : public g2o::BaseVertex<1, double>
{
public:
  void setToOriginImpl() override {_estimate = 0.0;}
  void oplusImpl(const double * update) override {_estimate += update[0];}
  bool read(std::istream &) override {return false;}
  bool write(std::ostream &) const override {return false;}
};

/// One depth reading: the network put this point `d` down the keyframe's optical
/// axis, and the keyframe's depth map is off by a factor `s` that all its readings
/// share. The residual is relative, `s - z / d`, so its sigma is a fraction of the
/// depth — the unit the network's error comes in. Jacobians are g2o's numeric
/// default: three vertices and a one-row residual, and a hand-written Jacobian is
/// one more thing that can be wrong with a plausible answer.
class EdgeDepthReading
  : public g2o::BaseFixedSizedEdge<1, double, g2o::VertexPointXYZ, g2o::VertexSE3Expmap,
    VertexLogScale>
{
public:
  void computeError() override
  {
    const auto * point = vertexXn<0>();
    const auto * pose = vertexXn<1>();
    const auto * scale = vertexXn<2>();
    const double z = pose->estimate().map(point->estimate()).z();
    _error[0] = std::exp(scale->estimate()) - z / _measurement;
  }
  bool depth_positive() const
  {
    return vertexXn<1>()->estimate().map(vertexXn<0>()->estimate()).z() > 0.0;
  }
  bool read(std::istream &) override {return false;}
  bool write(std::ostream &) const override {return false;}
};

/// The prior on a depth map's scale: `log s` about 0, which is the network's own
/// reading. Every solve starts from here again rather than from the last answer.
class EdgeLogScalePrior : public g2o::BaseUnaryEdge<1, double, VertexLogScale>
{
public:
  void computeError() override {_error[0] = vertexXn<0>()->estimate() - _measurement;}
  bool read(std::istream &) override {return false;}
  bool write(std::ostream &) const override {return false;}
};

g2o::SE3Quat camera_from_map(const cv::Affine3d & map_from_camera)
{
  const cv::Affine3d inv = map_from_camera.inv();
  const cv::Matx33d r = inv.rotation();
  const cv::Vec3d t = inv.translation();
  Eigen::Matrix3d rotation;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {rotation(i, j) = r(i, j);}
  }
  return g2o::SE3Quat(rotation, Eigen::Vector3d(t[0], t[1], t[2]));
}

cv::Affine3d map_from_camera(const g2o::SE3Quat & camera_from_map)
{
  const Eigen::Matrix3d r = camera_from_map.rotation().toRotationMatrix();
  const Eigen::Vector3d t = camera_from_map.translation();
  const cv::Affine3d cfm(
    cv::Matx33d(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2)),
    cv::Vec3d(t.x(), t.y(), t.z()));
  return cfm.inv();
}

}  // namespace

BaResult solve_local_ba(const BaProblem & problem, const BaConfig & config)
{
  BaResult result;
  result.map_from_camera.reserve(problem.keyframes.size());
  for (const BaKeyframe & kf : problem.keyframes) {
    result.map_from_camera.push_back(kf.map_from_camera);
    (kf.fixed ? result.fixed_keyframes : result.free_keyframes) += 1;
  }
  for (const BaPoint & p : problem.points) {result.positions.push_back(p.position);}
  result.outlier.assign(problem.observations.size(), 0);
  result.depth_scale.assign(problem.keyframes.size(), 0.0);

  // --- Refusals, before g2o sees anything ------------------------------------
  //
  // **Every one of these is a process abort otherwise, not an error return.** This
  // g2o is built with asserts on; a problem with no free pose trips BlockSolver's
  // `_sizePoses > 0` assertion (measured on both machines, 2026-09-29), and in a
  // component that is the whole container.
  if (result.free_keyframes == 0) {
    result.refusal = "no free keyframe";
    return result;
  }
  if (problem.points.empty() || problem.observations.empty()) {
    result.refusal = "nothing observed";
    return result;
  }
  // A free keyframe nothing observes is not in g2o's active set at all — so a window
  // whose free keyframes are all unobserved is the same abort by a different route.
  std::vector<std::uint8_t> free_observed(problem.keyframes.size(), 0);
  for (const BaObservation & o : problem.observations) {
    if (o.keyframe >= problem.keyframes.size() || o.point >= problem.points.size()) {
      result.refusal = "observation out of range";
      return result;
    }
    if (!problem.keyframes[o.keyframe].fixed) {free_observed[o.keyframe] = 1;}
  }
  bool any_free_observed = false;
  for (std::uint8_t f : free_observed) {any_free_observed = any_free_observed || f;}
  if (!any_free_observed) {
    result.refusal = "no free keyframe observes anything";
    return result;
  }

  // --- The graph ----------------------------------------------------------------
  //
  // With scale vertices the problem is no longer poses of six and points of three:
  // a scale is a one-dimensional non-marginalised block beside the poses, so the
  // fixed-size 6_3 block solver cannot hold it and the dynamic one is used. Without
  // them nothing changes, down to the solver, so `depth_scale_sigma = 0` is exactly
  // the BA gates/ba.sh measured on 2026-09-30.
  const bool model_scale = config.depth_scale_sigma > 0.0 && config.depth_sigma_rel > 0.0;
  std::unique_ptr<g2o::Solver> block;
  if (model_scale) {
    block = std::make_unique<g2o::BlockSolverX>(
      std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>());
  } else {
    block = std::make_unique<g2o::BlockSolver_6_3>(
      std::make_unique<g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>>());
  }
  auto * algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algorithm);  // owned by the optimizer from here

  const int point_base = static_cast<int>(problem.keyframes.size());
  const int scale_base = point_base + static_cast<int>(problem.points.size());
  for (std::size_t i = 0; i < problem.keyframes.size(); ++i) {
    auto * v = new g2o::VertexSE3Expmap();
    v->setId(static_cast<int>(i));
    v->setEstimate(camera_from_map(problem.keyframes[i].map_from_camera));
    v->setFixed(problem.keyframes[i].fixed);
    optimizer.addVertex(v);
  }
  for (std::size_t i = 0; i < problem.points.size(); ++i) {
    auto * v = new g2o::VertexPointXYZ();
    v->setId(point_base + static_cast<int>(i));
    const cv::Vec3d & p = problem.points[i].position;
    v->setEstimate(Eigen::Vector3d(p[0], p[1], p[2]));
    // Marginalised: the Schur complement eliminates the points and leaves a system
    // the size of the poses, which is what makes BA over thousands of points
    // tractable at all.
    v->setMarginalized(true);
    optimizer.addVertex(v);
  }
  // One scale per keyframe that has a reading to scale — **fixed keyframes
  // included**. A fixed keyframe's pose is held for the gauge; its depth map is as
  // wrong as anybody's, and holding its scale at 1 would press its error onto every
  // point it shares with the window.
  std::vector<VertexLogScale *> scale_of(problem.keyframes.size(), nullptr);
  if (model_scale) {
    const double prior_info = 1.0 / (config.depth_scale_sigma * config.depth_scale_sigma);
    for (const BaObservation & o : problem.observations) {
      if (o.depth <= 0.0 || scale_of[o.keyframe] != nullptr) {continue;}
      auto * v = new VertexLogScale();
      v->setId(scale_base + static_cast<int>(o.keyframe));
      v->setEstimate(0.0);
      optimizer.addVertex(v);
      scale_of[o.keyframe] = v;
      auto * prior = new EdgeLogScalePrior();
      prior->setVertex(0, v);
      prior->setMeasurement(0.0);
      prior->setInformation(Eigen::Matrix<double, 1, 1>::Constant(prior_info));
      optimizer.addEdge(prior);
      ++result.scale_vertices;
    }
  }

  const double pixel_info = 1.0 / (config.pixel_sigma * config.pixel_sigma);
  // Per observation: the edge carrying its pixel (a stereo edge also carries its
  // depth), and — with scale vertices — a second edge carrying its depth.
  std::vector<g2o::OptimizableGraph::Edge *> edges(problem.observations.size(), nullptr);
  std::vector<EdgeDepthReading *> readings(problem.observations.size(), nullptr);
  std::vector<std::uint8_t> is_depth(problem.observations.size(), 0);
  for (std::size_t i = 0; i < problem.observations.size(); ++i) {
    const BaObservation & o = problem.observations[i];
    const cv::Matx33d & k = problem.keyframes[o.keyframe].k;
    auto * point = optimizer.vertex(point_base + static_cast<int>(o.point));
    auto * pose = optimizer.vertex(static_cast<int>(o.keyframe));

    if (config.depth_sigma_rel > 0.0 && o.depth > 0.0 && !model_scale) {
      // **The depth reading as a third residual**, ORB-SLAM's RGB-D formulation: the
      // depth becomes a virtual right-image coordinate `u - bf / d`, so a depth error
      // is a pixel error on an imaginary second camera `bf / fx` to the right. The
      // baseline is arbitrary — it scales the residual and its sigma together — and
      // what matters is the sigma: the pixel noise plus the depth's own, carried
      // through the reciprocal.
      auto * e = new g2o::EdgeStereoSE3ProjectXYZ();
      e->setVertex(0, point);
      e->setVertex(1, pose);
      e->fx = k(0, 0);
      e->fy = k(1, 1);
      e->cx = k(0, 2);
      e->cy = k(1, 2);
      e->bf = k(0, 0) * 0.1;
      const double ur = o.pixel.x - e->bf / o.depth;
      e->setMeasurement(Eigen::Vector3d(o.pixel.x, o.pixel.y, ur));
      const double sigma_ur_depth = e->bf * config.depth_sigma_rel / o.depth;
      const double sigma_ur2 = config.pixel_sigma * config.pixel_sigma + sigma_ur_depth * sigma_ur_depth;
      Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
      info(0, 0) = pixel_info;
      info(1, 1) = pixel_info;
      info(2, 2) = 1.0 / sigma_ur2;
      e->setInformation(info);
      auto * kernel = new g2o::RobustKernelHuber();
      kernel->setDelta(std::sqrt(kChi2Depth));
      e->setRobustKernel(kernel);
      optimizer.addEdge(e);
      edges[i] = e;
      is_depth[i] = 1;
      ++result.depth_edges;
      continue;
    }

    auto * e = new g2o::EdgeSE3ProjectXYZ();
    e->setVertex(0, point);
    e->setVertex(1, pose);
    e->fx = k(0, 0);
    e->fy = k(1, 1);
    e->cx = k(0, 2);
    e->cy = k(1, 2);
    e->setMeasurement(Eigen::Vector2d(o.pixel.x, o.pixel.y));
    e->setInformation(Eigen::Matrix2d::Identity() * pixel_info);
    auto * kernel = new g2o::RobustKernelHuber();
    kernel->setDelta(std::sqrt(kChi2Mono));
    e->setRobustKernel(kernel);
    optimizer.addEdge(e);
    edges[i] = e;

    if (model_scale && o.depth > 0.0) {
      auto * r = new EdgeDepthReading();
      r->setVertex(0, point);
      r->setVertex(1, pose);
      r->setVertex(2, scale_of[o.keyframe]);
      r->setMeasurement(o.depth);
      const double sigma = config.depth_point_sigma_rel;
      r->setInformation(Eigen::Matrix<double, 1, 1>::Constant(1.0 / (sigma * sigma)));
      auto * rk = new g2o::RobustKernelHuber();
      rk->setDelta(std::sqrt(kChi2Scalar));
      r->setRobustKernel(rk);
      optimizer.addEdge(r);
      readings[i] = r;
      ++result.depth_edges;
    }
  }
  result.edges = problem.observations.size();

  // --- Two rounds, outliers off between them ---------------------------------------
  optimizer.initializeOptimization(0);
  optimizer.computeActiveErrors();
  result.chi2_before = optimizer.activeRobustChi2();
  result.iterations += optimizer.optimize(config.first_iterations);

  // An observation is an outlier when its pixel disagrees, when its depth reading
  // disagrees even after its depth map's scale is allowed for, or when the point has
  // gone behind the camera. Either half is enough: a feature matched to the wrong
  // point can agree in one and not the other.
  auto is_outlier = [&](std::size_t i) {
      auto * e = edges[i];
      const double limit = is_depth[i] ? kChi2Depth : kChi2Mono;
      e->computeError();
      const bool behind = is_depth[i] ?
        !static_cast<g2o::EdgeStereoSE3ProjectXYZ *>(e)->isDepthPositive() :
        !static_cast<g2o::EdgeSE3ProjectXYZ *>(e)->isDepthPositive();
      bool reading_bad = false;
      if (readings[i] != nullptr) {
        readings[i]->computeError();
        reading_bad = readings[i]->chi2() > kChi2Scalar || !readings[i]->depth_positive();
      }
      return behind || reading_bad || e->chi2() > limit;
    };
  for (std::size_t i = 0; i < edges.size(); ++i) {
    const bool outlier = is_outlier(i);
    for (g2o::OptimizableGraph::Edge * e : {edges[i],
        static_cast<g2o::OptimizableGraph::Edge *>(readings[i])})
    {
      if (e == nullptr) {continue;}
      // Level 1 is outside the optimisation, which runs at level 0. The edge stays in
      // the graph so its error can still be read after the second round.
      if (outlier) {e->setLevel(1);}
      // The second round without the kernel: the outliers are gone, and what is left
      // is where a quadratic cost is the right model.
      e->setRobustKernel(nullptr);
    }
  }
  // **And the same abort once more, after outlier rejection**: if every edge of every
  // free keyframe was switched off, re-initialising at level 0 leaves no pose in the
  // active set. The first round's answer stands in that case.
  bool free_still_active = false;
  for (std::size_t i = 0; i < edges.size() && !free_still_active; ++i) {
    const BaObservation & o = problem.observations[i];
    free_still_active = edges[i]->level() == 0 && !problem.keyframes[o.keyframe].fixed;
  }
  if (free_still_active) {
    optimizer.initializeOptimization(0);
    result.iterations += optimizer.optimize(config.second_iterations);
  }
  optimizer.computeActiveErrors();
  result.chi2_after = optimizer.activeRobustChi2();

  for (std::size_t i = 0; i < edges.size(); ++i) {
    result.outlier[i] = is_outlier(i) ? 1 : 0;
  }

  // --- Out -------------------------------------------------------------------------
  for (std::size_t i = 0; i < problem.keyframes.size(); ++i) {
    if (problem.keyframes[i].fixed) {continue;}
    const auto * v = static_cast<const g2o::VertexSE3Expmap *>(optimizer.vertex(static_cast<int>(i)));
    result.map_from_camera[i] = map_from_camera(v->estimate());
  }
  for (std::size_t i = 0; i < problem.points.size(); ++i) {
    const auto * v = static_cast<const g2o::VertexPointXYZ *>(
      optimizer.vertex(point_base + static_cast<int>(i)));
    const Eigen::Vector3d p = v->estimate();
    result.positions[i] = cv::Vec3d(p.x(), p.y(), p.z());
  }
  for (std::size_t i = 0; i < problem.keyframes.size(); ++i) {
    if (scale_of[i] != nullptr) {result.depth_scale[i] = std::exp(scale_of[i]->estimate());}
  }
  result.ran = true;
  return result;
}

}  // namespace pimesh_backend
