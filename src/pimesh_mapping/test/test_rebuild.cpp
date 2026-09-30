// P18's rebuild (#12): the frame memory, the correction a frame takes, and the
// volume integrated again at corrected poses.
//
// Every way this goes wrong produces a surface. A memory that forgets the start of a
// session rebuilds a smaller room that looks tidier; a correction interpolated where
// the graph gave a step invents poses; a rebuild that ignored its corrections is the
// live volume again at a lower resolution. These pin each against a scene whose
// answer is known.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "pimesh_mapping/rebuild.hpp"

using pimesh_mapping::Corrections;
using pimesh_mapping::FrameMemory;
using pimesh_mapping::RememberedFrame;
using pimesh_mapping::TsdfVolume;
using pimesh_mapping::correction_at;
using pimesh_mapping::pose_shift;
using pimesh_mapping::rebuild_volume;

namespace
{

constexpr std::int64_t kSecond = 1000000000LL;

cv::Matx33d test_k()
{
  return cv::Matx33d(500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0);
}

cv::Mat plane_at(float z)
{
  return cv::Mat(480, 640, CV_32FC1, cv::Scalar(z));
}

cv::Affine3d along_z(double z)
{
  return cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(0.0, 0.0, z));
}

}  // namespace

// --- The memory ---------------------------------------------------------------------

TEST(FrameMemory, StoresTheIntegratedDepthInMillimetresAtTheReducedSize)
{
  FrameMemory memory(FrameMemory::Config{10, 4});
  cv::Mat depth = plane_at(2.3456F);
  depth.at<float>(0, 0) = 0.0F;                     // no reading
  depth.at<float>(0, 4) = std::nanf("");            // not a number
  cv::Mat bgr(480, 640, CV_8UC3, cv::Scalar(10, 20, 30));
  ASSERT_TRUE(memory.offer(7, depth, bgr, test_k(), along_z(1.0), cv::Affine3d::Identity()));
  const RememberedFrame & f = memory.frames().at(0);
  EXPECT_EQ(f.depth_mm.size(), cv::Size(160, 120));
  EXPECT_EQ(f.depth_mm.type(), CV_16UC1);
  EXPECT_EQ(f.depth_mm.at<std::uint16_t>(0, 0), 0);
  EXPECT_EQ(f.depth_mm.at<std::uint16_t>(0, 1), 0) << "NaN is no reading, not 0 mm of something";
  EXPECT_EQ(f.depth_mm.at<std::uint16_t>(5, 5), 2346);
  EXPECT_EQ(f.bgr.size(), cv::Size(160, 120));
  // K follows the image: focal and principal point both divided by four.
  EXPECT_DOUBLE_EQ(f.k(0, 0), 125.0);
  EXPECT_DOUBLE_EQ(f.k(0, 2), 80.0);
  EXPECT_DOUBLE_EQ(f.k(1, 2), 60.0);
  EXPECT_EQ(f.odom_from_camera.translation()[2], 1.0);
  // A colour frame of the wrong size is dropped, not stretched.
  ASSERT_TRUE(memory.offer(8, depth, cv::Mat(10, 10, CV_8UC3), test_k(), along_z(1.0),
    cv::Affine3d::Identity()));
  EXPECT_TRUE(memory.frames().at(1).bgr.empty());
}

TEST(FrameMemory, WhenFullItThinsTheWholeSessionRatherThanForgettingItsStart)
{
  // The loop-closure case: the start of the session is the half being joined to. A
  // ring buffer would have dropped it. 100 frames through a memory of 10: the first
  // survives, the spacing is uniform, the end is covered, and the halvings say how
  // thin it got.
  FrameMemory memory(FrameMemory::Config{10, 4});
  for (int i = 0; i < 100; ++i) {
    memory.offer(i * kSecond, plane_at(2.0F), cv::Mat(), test_k(), along_z(0.0), cv::Affine3d::Identity());
  }
  const auto & frames = memory.frames();
  ASSERT_GE(frames.size(), 5u);
  EXPECT_LE(frames.size(), 11u);
  EXPECT_EQ(frames.front().stamp_ns, 0);
  const std::int64_t step = frames[1].stamp_ns - frames[0].stamp_ns;
  for (std::size_t i = 1; i < frames.size(); ++i) {
    EXPECT_EQ(frames[i].stamp_ns - frames[i - 1].stamp_ns, step) << i;
  }
  EXPECT_GE(frames.back().stamp_ns, 99 * kSecond - step);
  EXPECT_EQ(static_cast<std::int64_t>(memory.stride()) * kSecond, step);
  EXPECT_GE(memory.halvings(), 3u);
  EXPECT_GT(memory.bytes(), 0u);
}

// --- The correction a frame takes ---------------------------------------------------

TEST(CorrectionAt, IsTheLatestKeyframeAtOrBeforeTheFrameNotAnInterpolation)
{
  const Corrections c{{10 * kSecond, along_z(1.0)}, {20 * kSecond, along_z(3.0)}};
  EXPECT_EQ(correction_at(c, 5 * kSecond).translation()[2], 0.0) << "before the first: identity";
  EXPECT_EQ(correction_at(c, 10 * kSecond).translation()[2], 1.0) << "at a keyframe: its own";
  // Half way between: the earlier keyframe's, exactly. An interpolation would say 2.0.
  EXPECT_EQ(correction_at(c, 15 * kSecond).translation()[2], 1.0);
  EXPECT_EQ(correction_at(c, 99 * kSecond).translation()[2], 3.0) << "after the last: the last";
  EXPECT_EQ(correction_at(Corrections{}, 5 * kSecond).translation()[2], 0.0);
}

TEST(PoseShift, MeasuresHowFarTheCorrectionsMoveEachFrameFromWhereItWasIntegrated)
{
  FrameMemory memory(FrameMemory::Config{10, 4});
  memory.offer(0, plane_at(2.0F), cv::Mat(), test_k(), along_z(0.0), cv::Affine3d::Identity());
  memory.offer(10 * kSecond, plane_at(2.0F), cv::Mat(), test_k(), along_z(-0.3), cv::Affine3d::Identity());
  EXPECT_EQ(pose_shift(memory.frames(), Corrections{}).max_m, 0.0);
  const Corrections c{{0, cv::Affine3d::Identity()}, {10 * kSecond, along_z(0.3)}};
  EXPECT_NEAR(pose_shift(memory.frames(), c).max_m, 0.3, 1e-12);
  // A frame integrated live *with* a correction already applied is measured from
  // there: the same correction again moves it nowhere.
  FrameMemory already(FrameMemory::Config{10, 4});
  already.offer(10 * kSecond, plane_at(2.0F), cv::Mat(), test_k(), along_z(-0.3), along_z(0.3));
  EXPECT_NEAR(pose_shift(already.frames(), c).max_m, 0.0, 1e-12);
}

// --- The rebuild ----------------------------------------------------------------------

namespace
{

/// A wall 2 m ahead, seen twice from the same place. The second pass's odometry has
/// drifted 0.3 m toward the wall, so live integration put that pass's wall at 1.7 m —
/// a ghost in front of the real one, five truncation bands away. The pose graph's
/// correction for the second pass undoes the drift.
struct GhostScene
{
  FrameMemory memory {FrameMemory::Config{50, 4}};
  Corrections corrections;

  GhostScene()
  {
    for (int i = 0; i < 5; ++i) {
      memory.offer(i * kSecond / 10, plane_at(2.0F), cv::Mat(), test_k(), along_z(0.0),
        cv::Affine3d::Identity());
    }
    for (int i = 0; i < 5; ++i) {
      memory.offer(10 * kSecond + i * kSecond / 10, plane_at(2.0F), cv::Mat(), test_k(),
        along_z(-0.3), cv::Affine3d::Identity());
    }
    corrections = {{0, cv::Affine3d::Identity()}, {10 * kSecond, along_z(0.3)}};
  }
};

float first_surface(const TsdfVolume & volume)
{
  return volume.raycast_ray(cv::Affine3d::Identity(), cv::Vec3f(0.0F, 0.0F, 1.0F));
}

}  // namespace

TEST(RebuildVolume, AtTheCorrectedPosesTheGhostIsGoneAndWithoutThemItIsNot)
{
  // Both halves, so this cannot pass by the rebuild doing nothing: the control is the
  // same memory through the same function with no corrections, and it has to show
  // the ghost the correction removes.
  const GhostScene scene;
  const TsdfVolume::Options options;
  const auto control = rebuild_volume(scene.memory.frames(), Corrections{}, options, 4);
  const auto corrected = rebuild_volume(scene.memory.frames(), scene.corrections, options, 4);
  ASSERT_TRUE(control.volume && corrected.volume);
  EXPECT_NEAR(first_surface(*control.volume), 1.7F, 0.03F) << "the control must show the ghost";
  EXPECT_NEAR(first_surface(*corrected.volume), 2.0F, 0.03F);
}

TEST(RebuildVolume, IntegratesEveryRememberedFrame)
{
  // The plan's first false green for P18: a rebuild that skipped frames has fewer
  // shingles and looks cleaner for it. gates/rebuild.sh asserts this count equals the
  // memory's size; this pins that the function reports what it did.
  const GhostScene scene;
  const auto r = rebuild_volume(scene.memory.frames(), scene.corrections, TsdfVolume::Options{}, 4);
  EXPECT_EQ(r.integrated, scene.memory.size());
  EXPECT_EQ(r.integrated, 10u);
}

TEST(RebuildVolume, TheAllocationStrideFollowsTheReducedImage)
{
  // In pixels, so on a quarter-size image the live stride of 8 would sample four
  // times more sparsely than the live volume did. A narrow surface — a pole 3 px
  // wide at the stored size — has to be allocated at all.
  FrameMemory memory(FrameMemory::Config{10, 4});
  cv::Mat depth(480, 640, CV_32FC1, cv::Scalar(0.0F));
  // Stored columns 81-83: strictly between the samples a stride of 8 takes (80, 88).
  // The first version put it at 79-82, on a sampled column, and passed whatever the
  // stride was — found by deleting the division and watching it stay green.
  depth(cv::Rect(324, 0, 12, 480)).setTo(2.0F);     // 12 px live, 3 px stored
  for (int i = 0; i < 5; ++i) {
    memory.offer(i, depth, cv::Mat(), test_k(), cv::Affine3d::Identity(), cv::Affine3d::Identity());
  }
  const auto r = rebuild_volume(memory.frames(), Corrections{}, TsdfVolume::Options{}, 4);
  // Aimed at stored column 82, through the pole — not down the optical axis, which
  // crosses column 80 and would ask about a column the pole is not in.
  const cv::Vec3f toward_pole = cv::normalize(cv::Vec3f((82.0F - 80.0F) / 125.0F, 0.0F, 1.0F));
  EXPECT_NEAR(r.volume->raycast_ray(cv::Affine3d::Identity(), toward_pole), 2.0F, 0.03F);
}
