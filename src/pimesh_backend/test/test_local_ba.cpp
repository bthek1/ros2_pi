// Local bundle adjustment on synthetic windows whose answer is known.
//
// The three ways this goes wrong that a running pipeline would not report as
// wrong: a solve that lowers its cost and moves the geometry further from the
// truth, a solve that never moves anything (identical to no BA, and a gate whose
// control then passes for it), and a window g2o cannot take — which with this
// build's asserts is a process abort, in a component that is the whole container.

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "pimesh_backend/local_ba.hpp"
#include "pimesh_backend/triangulation.hpp"

using pimesh_backend::BaConfig;
using pimesh_backend::BaKeyframe;
using pimesh_backend::BaObservation;
using pimesh_backend::BaPoint;
using pimesh_backend::BaProblem;
using pimesh_backend::BaResult;
using pimesh_backend::solve_local_ba;

namespace
{

cv::Matx33d camera_k()
{
  return cv::Matx33d(517.3, 0.0, 318.6, 0.0, 516.5, 255.3, 0.0, 0.0, 1.0);
}

cv::Matx33d yaw(double a)
{
  return cv::Matx33d(std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a));
}

/// Five cameras on a gentle arc looking into a box of points, every point seen by
/// every camera it projects into, with exact pixels and exact depths.
struct Scene
{
  std::vector<cv::Affine3d> poses;
  std::vector<cv::Vec3d> points;
  BaProblem problem;

  explicit Scene(std::size_t n_points = 150, unsigned seed = 21)
  {
    for (int i = 0; i < 5; ++i) {
      poses.emplace_back(yaw(0.04 * (i - 2)), cv::Vec3d(0.15 * i, 0.02 * i, 0.0));
    }
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> x(-1.0, 1.6);
    std::uniform_real_distribution<double> y(-0.6, 0.6);
    std::uniform_real_distribution<double> z(2.0, 4.0);
    for (std::size_t i = 0; i < n_points; ++i) {points.emplace_back(x(gen), y(gen), z(gen));}

    for (std::size_t k = 0; k < poses.size(); ++k) {
      problem.keyframes.push_back(
        BaKeyframe{static_cast<pimesh_backend::KeyframeId>(k), poses[k], camera_k(), k == 0});
    }
    for (std::size_t p = 0; p < points.size(); ++p) {
      problem.points.push_back(BaPoint{static_cast<pimesh_backend::PointId>(p), points[p]});
      for (std::size_t k = 0; k < poses.size(); ++k) {
        cv::Point2d pixel;
        if (!pimesh_backend::project(poses[k], camera_k(), points[p], pixel)) {continue;}
        if (pixel.x < 0 || pixel.x >= 640 || pixel.y < 0 || pixel.y >= 480) {continue;}
        BaObservation o;
        o.keyframe = k;
        o.point = p;
        o.pixel = cv::Point2f(static_cast<float>(pixel.x), static_cast<float>(pixel.y));
        o.depth = (poses[k].inv() * points[p])[2];
        problem.observations.push_back(o);
      }
    }
  }
};

double pose_error_m(const std::vector<cv::Affine3d> & a, const std::vector<cv::Affine3d> & b)
{
  double worst = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    worst = std::max(worst, cv::norm(cv::Vec3d(a[i].translation()) - cv::Vec3d(b[i].translation())));
  }
  return worst;
}

double point_error_m(const std::vector<cv::Vec3d> & a, const std::vector<cv::Vec3d> & b)
{
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {sum += cv::norm(a[i] - b[i]);}
  return sum / static_cast<double>(a.size());
}

}  // namespace

// --- The refusals: each of these is an abort without them ---------------------------

TEST(LocalBa, RefusesAWindowWithNoFreeKeyframeInsteadOfAborting)
{
  // **The reason this test exists at all**: handed to g2o, this problem trips
  // BlockSolver's `_sizePoses > 0` assertion and the process dies — measured on both
  // machines while probing the library. If the refusal is ever removed, this test
  // does not fail; the test *binary* aborts, which is the louder of the two.
  Scene s;
  for (auto & kf : s.problem.keyframes) {kf.fixed = true;}
  const BaResult r = solve_local_ba(s.problem, BaConfig{});
  EXPECT_FALSE(r.ran);
  EXPECT_STREQ(r.refusal, "no free keyframe");
  EXPECT_EQ(r.iterations, 0);
}

TEST(LocalBa, RefusesWhenNoFreeKeyframeObservesAnything)
{
  // The same abort by a different route: a free keyframe with no edges is not in
  // g2o's active set, so a window whose only free keyframe sees nothing has no pose.
  Scene s;
  for (auto & kf : s.problem.keyframes) {kf.fixed = true;}
  s.problem.keyframes.push_back(BaKeyframe{99, cv::Affine3d::Identity(), camera_k(), false});
  const BaResult r = solve_local_ba(s.problem, BaConfig{});
  EXPECT_FALSE(r.ran);
  EXPECT_STREQ(r.refusal, "no free keyframe observes anything");
}

TEST(LocalBa, RefusesAnObservationThatPointsOutsideTheProblem)
{
  Scene s;
  s.problem.observations.front().point = s.problem.points.size();
  EXPECT_FALSE(solve_local_ba(s.problem, BaConfig{}).ran);
}

// --- What it does -------------------------------------------------------------------

TEST(LocalBa, PullsPerturbedPosesAndPointsBackTowardsTheTruth)
{
  Scene s;
  std::mt19937 gen(4);
  std::normal_distribution<double> jitter(0.0, 0.03);
  for (std::size_t k = 1; k < s.problem.keyframes.size(); ++k) {
    auto & kf = s.problem.keyframes[k];
    kf.map_from_camera = cv::Affine3d(
      yaw(jitter(gen) * 0.3) * kf.map_from_camera.rotation(),
      cv::Vec3d(kf.map_from_camera.translation()) + cv::Vec3d(jitter(gen), jitter(gen), jitter(gen)));
  }
  for (auto & p : s.problem.points) {p.position += cv::Vec3d(jitter(gen), jitter(gen), jitter(gen));}

  std::vector<cv::Affine3d> before_poses;
  std::vector<cv::Vec3d> before_points;
  for (const auto & kf : s.problem.keyframes) {before_poses.push_back(kf.map_from_camera);}
  for (const auto & p : s.problem.points) {before_points.push_back(p.position);}

  const BaResult r = solve_local_ba(s.problem, BaConfig{});
  ASSERT_TRUE(r.ran) << r.refusal;
  // **Not merely a lower cost** — the plan's first false green is a BA whose cost
  // falls while the geometry gets worse. Both are asserted against the truth.
  EXPECT_GT(r.iterations, 0);
  EXPECT_LT(r.chi2_after, r.chi2_before);
  EXPECT_LT(pose_error_m(r.map_from_camera, s.poses), 0.25 * pose_error_m(before_poses, s.poses));
  EXPECT_LT(point_error_m(r.positions, s.points), 0.25 * point_error_m(before_points, s.points));
  EXPECT_EQ(r.free_keyframes, 4u);
  EXPECT_EQ(r.fixed_keyframes, 1u);
}

TEST(LocalBa, AFixedKeyframeDoesNotMove)
{
  Scene s;
  s.problem.keyframes[0].map_from_camera =
    cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(0.05, 0.0, 0.0));
  const BaResult r = solve_local_ba(s.problem, BaConfig{});
  ASSERT_TRUE(r.ran);
  EXPECT_EQ(
    cv::norm(cv::Matx44d(r.map_from_camera[0].matrix) - cv::Matx44d(s.problem.keyframes[0].map_from_camera.matrix)),
    0.0);
}

TEST(LocalBa, TheDepthPriorIsWhatAnchorsScale)
{
  // **P14's finding, as a property of the solver.** Reprojection error is blind to
  // scale: a window 30% too large, poses and points together, reprojects exactly as
  // well as the true one. With one keyframe fixed and nothing else to say what a
  // metre is, monocular BA has no reason to shrink it — and P14 measured what a map
  // does with its scale when nothing anchors it. The depth readings are what say
  // how big the room is, and with the prior on, the window comes back to size.
  auto scaled = [](double factor) {
      Scene s;
      for (std::size_t k = 1; k < s.problem.keyframes.size(); ++k) {
        auto & kf = s.problem.keyframes[k];
        kf.map_from_camera = cv::Affine3d(
          kf.map_from_camera.rotation(), cv::Vec3d(kf.map_from_camera.translation()) * factor);
      }
      for (auto & p : s.problem.points) {p.position *= factor;}
      return s;
    };
  auto span = [](const std::vector<cv::Affine3d> & poses) {
      return cv::norm(cv::Vec3d(poses.back().translation()) - cv::Vec3d(poses.front().translation()));
    };

  Scene mono = scaled(1.3);
  BaConfig no_depth;
  no_depth.depth_sigma_rel = 0.0;
  const BaResult m = solve_local_ba(mono.problem, no_depth);
  ASSERT_TRUE(m.ran);
  EXPECT_EQ(m.depth_edges, 0u);
  EXPECT_NEAR(span(m.map_from_camera) / span(mono.poses), 1.3, 0.05);

  Scene rgbd = scaled(1.3);
  const BaResult d = solve_local_ba(rgbd.problem, BaConfig{});
  ASSERT_TRUE(d.ran);
  EXPECT_GT(d.depth_edges, 0u);
  EXPECT_NEAR(span(d.map_from_camera) / span(rgbd.poses), 1.0, 0.05);
}

TEST(LocalBa, AMismatchedObservationIsCalledAnOutlierAndDoesNotMoveThePose)
{
  Scene s;
  // One observation, 40 px off: a correspondence that is wrong.
  const std::size_t bad = 17;
  s.problem.observations[bad].pixel.x += 40.0F;
  const BaResult r = solve_local_ba(s.problem, BaConfig{});
  ASSERT_TRUE(r.ran);
  EXPECT_EQ(r.outlier[bad], 1);
  std::size_t flagged = 0;
  for (auto o : r.outlier) {flagged += o;}
  EXPECT_EQ(flagged, 1u);
  EXPECT_LT(pose_error_m(r.map_from_camera, s.poses), 0.005);
}
