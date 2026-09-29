// Triangulation — the first route to a landmark's depth in this project that does
// not go through the depth network.
//
// Every failure here produces a *point*, and a point is exactly what the caller
// was expecting: a mirror image behind the cameras reprojects perfectly, a
// triangulation at one degree of parallax lands metres along the ray with a
// sub-pixel residual, and a mean over four views hides the one that is a mismatch.
// None of them is an error anywhere else in the pipeline.

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "pimesh_backend/triangulation.hpp"

using pimesh_backend::project;
using pimesh_backend::triangulate;
using pimesh_backend::Triangulation;
using pimesh_backend::View;

namespace
{

/// TUM fr1's intrinsics, 640x480 — the sequence gates/map.sh measures on.
cv::Matx33d camera_k()
{
  return cv::Matx33d(517.3, 0.0, 318.6, 0.0, 516.5, 255.3, 0.0, 0.0, 1.0);
}

/// A camera at `centre` looking down +z of the map, which is what an identity
/// rotation means for an optical frame.
cv::Affine3d camera_at(const cv::Vec3d & centre)
{
  return cv::Affine3d(cv::Matx33d::eye(), centre);
}

View seen(const cv::Affine3d & pose, const cv::Vec3d & point, double noise_x = 0.0,
  double noise_y = 0.0)
{
  cv::Point2d pixel;
  EXPECT_TRUE(project(pose, camera_k(), point, pixel));
  return View{pose, camera_k(),
    cv::Point2f(static_cast<float>(pixel.x + noise_x), static_cast<float>(pixel.y + noise_y))};
}

constexpr double kDeg = CV_PI / 180.0;

}  // namespace

TEST(Project, RefusesAPointBehindTheCamera)
{
  cv::Point2d pixel;
  EXPECT_TRUE(project(camera_at({0, 0, 0}), camera_k(), {0.1, 0.0, 2.0}, pixel));
  EXPECT_FALSE(project(camera_at({0, 0, 0}), camera_k(), {0.1, 0.0, -2.0}, pixel));
  // And on the image plane itself, where the division is by nothing.
  EXPECT_FALSE(project(camera_at({0, 0, 0}), camera_k(), {0.1, 0.0, 0.0}, pixel));
}

TEST(Project, PutsThePrincipalPointOnTheOpticalAxis)
{
  cv::Point2d pixel;
  ASSERT_TRUE(project(camera_at({0.5, -0.2, 0}), camera_k(), {0.5, -0.2, 3.0}, pixel));
  EXPECT_NEAR(pixel.x, 318.6, 1e-9);
  EXPECT_NEAR(pixel.y, 255.3, 1e-9);
}

TEST(Triangulate, RecoversAPointFromTwoExactViews)
{
  const cv::Vec3d truth(0.3, -0.2, 2.5);
  const Triangulation t = triangulate(
    {seen(camera_at({0, 0, 0}), truth), seen(camera_at({0.5, 0, 0}), truth)},
    2.0 * kDeg, 1.0);
  ASSERT_TRUE(t.ok) << t.refusal;
  EXPECT_LT(cv::norm(t.point - truth), 1e-4);
  EXPECT_LT(t.max_error_px, 0.01);
  EXPECT_GT(t.parallax_rad, 10.0 * kDeg);
}

TEST(Triangulate, UsesEveryViewAndNotTheFirstTwo)
{
  // Five cameras along a line, with noise. The first two are 1 cm apart — far too
  // close to triangulate from on their own — and the widest pair is 40 cm, so a
  // solver that used only the first two would refuse on parallax.
  const cv::Vec3d truth(-0.4, 0.1, 3.0);
  std::mt19937 gen(3);
  std::normal_distribution<double> noise(0.0, 0.4);
  std::vector<View> views;
  for (double x : {0.0, 0.01, 0.1, 0.25, 0.4}) {
    views.push_back(seen(camera_at({x, 0, 0}), truth, noise(gen), noise(gen)));
  }
  const Triangulation t = triangulate(views, 3.0 * kDeg, 2.0);
  ASSERT_TRUE(t.ok) << t.refusal;
  EXPECT_LT(cv::norm(t.point - truth), 0.05 * 3.0);
  EXPECT_LT(t.mean_error_px, 1.0);
}

TEST(Triangulate, RefusesLowParallaxEvenWhenTheGeometryIsExact)
{
  const cv::Vec3d truth(0.0, 0.0, 3.0);
  // 3 cm of baseline at 3 m is ~0.6 degrees.
  const Triangulation t = triangulate(
    {seen(camera_at({0, 0, 0}), truth), seen(camera_at({0.03, 0, 0}), truth)},
    3.0 * kDeg, 2.0);
  EXPECT_FALSE(t.ok);
  EXPECT_STREQ(t.refusal, "parallax");
  EXPECT_NEAR(t.parallax_rad, std::atan2(0.03, 3.0), 1e-3);
}

TEST(Triangulate, TheParallaxFloorIsWhyALowResidualIsNotEvidence)
{
  // **The reason for the floor, as a measurement.** One degree of parallax, half a
  // pixel of noise in one view: solved anyway (floor set to zero), the point comes
  // out tens of percent wrong in depth while reprojecting *better* than the budget
  // an accepted point is held to. The residual is at its most reassuring exactly
  // where the answer is least trustworthy — which is why the refusal is on the
  // geometry and not on the fit.
  const cv::Vec3d truth(0.0, 0.0, 3.0);
  const double baseline = 3.0 * std::tan(1.0 * kDeg);
  const Triangulation t = triangulate(
    {seen(camera_at({0, 0, 0}), truth), seen(camera_at({baseline, 0, 0}), truth, 0.5, 0.0)},
    0.0, 2.0);
  ASSERT_TRUE(t.ok) << t.refusal;
  EXPECT_LT(t.max_error_px, 0.5);
  EXPECT_GT(std::abs(t.point[2] - truth[2]) / truth[2], 0.05);

  // And at ten degrees the same half pixel is a fraction of a percent.
  const double wide = 3.0 * std::tan(10.0 * kDeg);
  const Triangulation w = triangulate(
    {seen(camera_at({0, 0, 0}), truth), seen(camera_at({wide, 0, 0}), truth, 0.5, 0.0)},
    0.0, 2.0);
  ASSERT_TRUE(w.ok);
  EXPECT_LT(std::abs(w.point[2] - truth[2]) / truth[2], 0.01);
}

TEST(Triangulate, RefusesAPointBehindTheCameras)
{
  // Two rays that diverge: the left camera sees the feature to its left, the right
  // camera to its right. They are closest *behind* both cameras, where DLT happily
  // solves them, and a point there reprojects onto both pixels exactly.
  const cv::Matx33d k = camera_k();
  const View left{camera_at({0, 0, 0}), k, cv::Point2f(100.0F, 255.3F)};
  const View right{camera_at({0.5, 0, 0}), k, cv::Point2f(540.0F, 255.3F)};
  const Triangulation t = triangulate({left, right}, 1.0 * kDeg, 2.0);
  EXPECT_FALSE(t.ok);
  EXPECT_STREQ(t.refusal, "behind a camera");
}

TEST(Triangulate, TheWorstViewDecidesAndNotTheMean)
{
  // Seven views, one of them a mismatch 8 px off. The mean over seven is inside a
  // 3 px budget; the worst view is not. A mean-based check would accept a point
  // built on a correspondence that is wrong.
  const cv::Vec3d truth(0.2, 0.1, 2.0);
  std::vector<View> views;
  for (double x : {0.0, 0.1, 0.2, 0.3, 0.4, 0.5}) {
    views.push_back(seen(camera_at({x, 0, 0}), truth));
  }
  views.push_back(seen(camera_at({0.6, 0, 0}), truth, 0.0, 8.0));
  const Triangulation t = triangulate(views, 2.0 * kDeg, 3.0);
  EXPECT_FALSE(t.ok);
  EXPECT_STREQ(t.refusal, "reprojection");
  EXPECT_LT(t.mean_error_px, 3.0);
  EXPECT_GT(t.max_error_px, 3.0);
}

TEST(Triangulate, OneViewIsNotATriangulation)
{
  const Triangulation t = triangulate({seen(camera_at({0, 0, 0}), {0, 0, 2})}, 0.0, 2.0);
  EXPECT_FALSE(t.ok);
  EXPECT_STREQ(t.refusal, "one view");
}

TEST(Triangulate, WorksWithRotatedCameras)
{
  // Two cameras converging on the point from either side, each turned 15 degrees
  // towards it. An implementation that used the pose's rotation the wrong way
  // round (camera-from-map where map-from-camera was meant) passes every test
  // above, where every rotation is the identity.
  const cv::Vec3d truth(0.0, 0.0, 2.0);
  auto yaw = [](double a) {
      return cv::Matx33d(std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a));
    };
  const cv::Affine3d left(yaw(15.0 * kDeg), cv::Vec3d(-0.5, 0, 0.1));
  const cv::Affine3d right(yaw(-15.0 * kDeg), cv::Vec3d(0.5, 0, 0.1));
  const Triangulation t = triangulate({seen(left, truth), seen(right, truth)}, 2.0 * kDeg, 1.0);
  ASSERT_TRUE(t.ok) << t.refusal;
  EXPECT_LT(cv::norm(t.point - truth), 1e-4);
}
