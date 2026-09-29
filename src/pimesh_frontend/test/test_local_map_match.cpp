// The projection search: finding a map point in this frame by where it should be
// and what it should look like, after the tracker has lost its track id.
//
// **Every failure here is a correspondence, and PnP uses correspondences.** A corner
// claimed by two map points, an ambiguous match taken on a coin flip, a point
// behind the camera "found" on the sensor — none of them is an error. They are
// matches, a RANSAC may or may not throw them out, and the ones it keeps move the
// pose with a residual that looks fine.

#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <vector>

#include "pimesh_frontend/local_map_match.hpp"

using pimesh_backend::PointView;
using pimesh_frontend::ProjectionConfig;
using pimesh_frontend::search_by_projection;

namespace
{

cv::Matx33d camera_k()
{
  return cv::Matx33d(517.3, 0.0, 318.6, 0.0, 516.5, 255.3, 0.0, 0.0, 1.0);
}

const cv::Size kImage(640, 480);

cv::Mat random_descriptor(std::mt19937 & gen)
{
  std::uniform_int_distribution<int> byte(0, 255);
  cv::Mat d(1, 32, CV_8U);
  for (int b = 0; b < 32; ++b) {d.at<std::uint8_t>(0, b) = static_cast<std::uint8_t>(byte(gen));}
  return d;
}

/// `bits` bits of `d` flipped, from the front.
cv::Mat flipped(const cv::Mat & d, int bits)
{
  cv::Mat out = d.clone();
  for (int i = 0; i < bits; ++i) {out.at<std::uint8_t>(0, i / 8) ^= static_cast<std::uint8_t>(1u << (i % 8));}
  return out;
}

cv::Point2f projected(const cv::Vec3d & p)
{
  const cv::Matx33d k = camera_k();
  return cv::Point2f(
    static_cast<float>(k(0, 0) * p[0] / p[2] + k(0, 2)),
    static_cast<float>(k(1, 1) * p[1] / p[2] + k(1, 2)));
}

struct Scene
{
  std::vector<PointView> points;
  std::vector<cv::Point2f> corners;
  cv::Mat descriptors;
};

/// `n` map points in front of an identity camera, each seen as a corner where it
/// projects, with its own descriptor.
Scene scene(std::size_t n)
{
  std::mt19937 gen(5);
  std::uniform_real_distribution<double> x(-0.8, 0.8);
  std::uniform_real_distribution<double> y(-0.6, 0.6);
  std::uniform_real_distribution<double> z(2.0, 4.0);
  Scene s;
  std::vector<cv::Mat> rows;
  for (std::size_t i = 0; i < n; ++i) {
    const cv::Vec3d p(x(gen), y(gen), z(gen));
    const cv::Mat d = random_descriptor(gen);
    s.points.push_back(PointView{static_cast<pimesh_backend::PointId>(i), p, d});
    s.corners.push_back(projected(p));
    rows.push_back(d);
  }
  cv::vconcat(rows, s.descriptors);
  return s;
}

std::vector<std::uint8_t> none(std::size_t n) {return std::vector<std::uint8_t>(n, 0);}

}  // namespace

TEST(SearchByProjection, FindsEveryPointWhereItProjects)
{
  const Scene s = scene(60);
  const auto r = search_by_projection(
    s.points, none(60), cv::Affine3d::Identity(), camera_k(), kImage, s.corners,
    s.descriptors, none(60), ProjectionConfig{});
  ASSERT_EQ(r.matches.size(), 60u);
  EXPECT_EQ(r.in_view.size(), 60u);
  for (const auto & m : r.matches) {
    EXPECT_EQ(m.point, m.corner);
    EXPECT_EQ(m.distance, 0);
  }
}

TEST(SearchByProjection, UsesThePredictedPoseAndNotTheIdentity)
{
  // The corners were seen by a camera 0.2 m to the right. Projected from the
  // identity they land ~30-50 px off and nothing is found; projected from the right
  // pose every one is.
  const Scene s = scene(40);
  const cv::Affine3d moved(cv::Matx33d::eye(), cv::Vec3d(0.2, 0.0, 0.0));
  std::vector<cv::Point2f> corners;
  for (const auto & p : s.points) {corners.push_back(projected(p.position - cv::Vec3d(0.2, 0, 0)));}
  const auto wrong = search_by_projection(
    s.points, none(40), cv::Affine3d::Identity(), camera_k(), kImage, corners,
    s.descriptors, none(40), ProjectionConfig{});
  const auto right = search_by_projection(
    s.points, none(40), moved, camera_k(), kImage, corners, s.descriptors, none(40),
    ProjectionConfig{});
  EXPECT_LT(wrong.matches.size(), 5u);
  EXPECT_EQ(right.matches.size(), 40u);
}

TEST(SearchByProjection, ACornerIsClaimedOnceAndByTheCloserDescriptor)
{
  // Two map points projecting within a few pixels of one corner. Both are close in
  // descriptor; the one that is closer wins it and the other goes without — rather
  // than the corner entering the PnP twice.
  std::mt19937 gen(9);
  const cv::Mat d = random_descriptor(gen);
  std::vector<PointView> points{
    PointView{0, cv::Vec3d(0.0, 0.0, 3.0), flipped(d, 10)},
    PointView{1, cv::Vec3d(0.01, 0.0, 3.0), flipped(d, 3)}};
  const std::vector<cv::Point2f> corners{projected({0.005, 0.0, 3.0})};
  const auto r = search_by_projection(
    points, none(2), cv::Affine3d::Identity(), camera_k(), kImage, corners, d, none(1),
    ProjectionConfig{});
  ASSERT_EQ(r.matches.size(), 1u);
  EXPECT_EQ(r.matches[0].point, 1u);
  EXPECT_EQ(r.matches[0].distance, 3);
}

TEST(SearchByProjection, AnAmbiguousMatchIsRefused)
{
  // Two corners inside the window, one 20 bits away and one 22: a repeated texture.
  // Picking the 20 is a coin flip with a margin of two bits.
  std::mt19937 gen(4);
  const cv::Mat d = random_descriptor(gen);
  const cv::Vec3d p(0.0, 0.0, 3.0);
  const std::vector<PointView> points{PointView{0, p, d}};
  const cv::Point2f at = projected(p);
  const std::vector<cv::Point2f> corners{at + cv::Point2f(3, 0), at + cv::Point2f(-3, 0)};
  cv::Mat rows;
  cv::vconcat(flipped(d, 20), flipped(d, 22), rows);
  const auto r = search_by_projection(
    points, none(1), cv::Affine3d::Identity(), camera_k(), kImage, corners, rows, none(2),
    ProjectionConfig{});
  EXPECT_TRUE(r.matches.empty());
  // And with the runner-up far enough behind, the same best match is accepted.
  cv::vconcat(flipped(d, 20), flipped(d, 60), rows);
  const auto clear = search_by_projection(
    points, none(1), cv::Affine3d::Identity(), camera_k(), kImage, corners, rows, none(2),
    ProjectionConfig{});
  EXPECT_EQ(clear.matches.size(), 1u);
}

TEST(SearchByProjection, RefusesPastTheHammingThresholdAndTheRadius)
{
  std::mt19937 gen(8);
  const cv::Mat d = random_descriptor(gen);
  const cv::Vec3d p(0.0, 0.0, 3.0);
  const std::vector<PointView> points{PointView{0, p, d}};
  const cv::Point2f at = projected(p);

  // The right corner, but 60 bits different: not the same corner.
  auto far_in_bits = search_by_projection(
    points, none(1), cv::Affine3d::Identity(), camera_k(), kImage, {at}, flipped(d, 60),
    none(1), ProjectionConfig{});
  EXPECT_TRUE(far_in_bits.matches.empty());

  // The identical descriptor, but 20 px away with a 12 px window.
  auto far_in_pixels = search_by_projection(
    points, none(1), cv::Affine3d::Identity(), camera_k(), kImage, {at + cv::Point2f(20, 0)},
    d, none(1), ProjectionConfig{});
  EXPECT_TRUE(far_in_pixels.matches.empty());
}

TEST(SearchByProjection, APointBehindTheCameraOrOffTheSensorIsNotInView)
{
  // A point behind the camera projects, with the sign flipped, to a perfectly good
  // pixel — and if there is a corner there with a similar descriptor, a search that
  // does not check will find it. Counting it as "in view" would also charge the map
  // point a miss it could never have avoided.
  std::mt19937 gen(2);
  const cv::Mat d = random_descriptor(gen);
  const std::vector<PointView> points{
    PointView{0, cv::Vec3d(0.0, 0.0, -3.0), d},
    PointView{1, cv::Vec3d(5.0, 0.0, 2.0), d}};
  const std::vector<cv::Point2f> corners{cv::Point2f(318.6F, 255.3F)};
  const auto r = search_by_projection(
    points, none(2), cv::Affine3d::Identity(), camera_k(), kImage, corners, d, none(1),
    ProjectionConfig{});
  EXPECT_TRUE(r.matches.empty());
  EXPECT_TRUE(r.in_view.empty());
}

TEST(SearchByProjection, IgnoresWhatStageOneAlreadyMatched)
{
  const Scene s = scene(20);
  std::vector<std::uint8_t> skip = none(20);
  std::vector<std::uint8_t> taken = none(20);
  skip[3] = 1;
  taken[7] = 1;
  const auto r = search_by_projection(
    s.points, skip, cv::Affine3d::Identity(), camera_k(), kImage, s.corners, s.descriptors,
    taken, ProjectionConfig{});
  EXPECT_EQ(r.matches.size(), 18u);
  for (const auto & m : r.matches) {
    EXPECT_NE(m.point, 3u);
    EXPECT_NE(m.corner, 7u);
  }
  // A skipped point is not counted as predicted-visible either: stage one has
  // already reported on it.
  EXPECT_EQ(r.in_view.size(), 19u);
}

TEST(SearchByProjection, RaggedInputFindsNothing)
{
  const Scene s = scene(10);
  const auto r = search_by_projection(
    s.points, none(9), cv::Affine3d::Identity(), camera_k(), kImage, s.corners,
    s.descriptors, none(10), ProjectionConfig{});
  EXPECT_TRUE(r.matches.empty());
}
