// The centred-patch statistic P12's unit comes out of.
//
// **This is a gate's instrument, and a gate's instrument needs its own tests.**
// `tools/gates/scale.sh` divides the tape measure by the number this function
// returns and calls the result `depth_scale` — the constant under every distance
// this project will ever report. There is no second opinion on it anywhere, and
// every way of getting it wrong returns a plausible distance rather than an
// error:
//
//   - **a patch that is not centred** reads a different part of a wall, which on
//     a flat surface square-on is very nearly the same number and on anything
//     else is quietly not;
//   - **a NaN or an infinity included in the sort** gives a median that depends
//     on the sort's implementation;
//   - **far-clip values counted as distances** are the worst of the three,
//     because `max_range_m` is the model's "far away or no idea" written into the
//     map as a real-looking 6 m. A patch that is half clip reports a median
//     between the surface and the clip, and depth is linear in `depth_scale`
//     *only below the clip* — so an implied scale derived from it is wrong in the
//     direction that makes the scale look smaller, with nothing saying so;
//   - **an empty patch reported as a median of zero** is `cost_mean=0.00`
//     again: 0 m is a plausible distance and zero reads as "very close".
//
// No ROS, no GPU, no model: synthetic `cv::Mat`s with known contents, so this
// runs on the Pi like the rest of this package's suite.

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "opencv2/core.hpp"
#include "pimesh_depth/depth_patch.hpp"

using pimesh_depth::PatchStats;
using pimesh_depth::centred_patch_stats;

namespace
{

constexpr double kClip = 6.0;

/// A depth map of one value throughout.
cv::Mat flat(int w, int h, float metres)
{
  return cv::Mat(h, w, CV_32FC1, cv::Scalar(metres));
}

}  // namespace

TEST(DepthPatch, ReadsAFlatSurfaceBackAtItsOwnDistance)
{
  const cv::Mat depth = flat(640, 480, 2.5F);
  const PatchStats s = centred_patch_stats(depth, 0.25, kClip);

  // 0.25 of 640x480 is 160x120.
  EXPECT_EQ(s.total, 160u * 120u);
  EXPECT_EQ(s.usable, s.total);
  EXPECT_EQ(s.clipped, 0u);
  EXPECT_EQ(s.bad, 0u);
  EXPECT_DOUBLE_EQ(s.median, 2.5);
  EXPECT_DOUBLE_EQ(s.iqr(), 0.0);
}

TEST(DepthPatch, IsActuallyCentred)
{
  // A map that is 1 m everywhere except a centred 200x200 block at 3 m. A patch
  // of 0.25 (80x60 of a 320x320... see below) has to land entirely inside that
  // block, and would not if it started at the origin or were offset by half.
  cv::Mat depth = flat(320, 320, 1.0F);
  depth(cv::Rect(60, 60, 200, 200)).setTo(3.0F);

  const PatchStats s = centred_patch_stats(depth, 0.25, kClip);
  EXPECT_EQ(s.total, 80u * 80u);
  EXPECT_DOUBLE_EQ(s.median, 3.0) << "the patch is reading outside the centred block";
  EXPECT_DOUBLE_EQ(s.iqr(), 0.0) << "the patch straddles the block's edge";
}

TEST(DepthPatch, CentresAnOddLeftoverOnePixelLeftAndUp)
{
  // Pinned so that the convention is a decision rather than an accident, and so
  // that "fixing" it the other way is a visible change. 5 wide with a patch of 3
  // leaves 2 over; (5 - 3) / 2 = 1, so the patch is columns 1..3 of 0..4 — the
  // exact centre. With 4 wide and a patch of 3, (4 - 3) / 2 = 0, so columns 0..2:
  // one left of centre.
  cv::Mat odd(1, 5, CV_32FC1);
  for (int x = 0; x < 5; ++x) {odd.at<float>(0, x) = static_cast<float>(x + 1);}
  // 0.6 of 5 is 3.
  const PatchStats a = centred_patch_stats(odd, 0.6, kClip);
  ASSERT_EQ(a.usable, 3u);
  EXPECT_DOUBLE_EQ(a.median, 3.0) << "columns 1,2,3 hold 2,3,4";

  cv::Mat even(1, 4, CV_32FC1);
  for (int x = 0; x < 4; ++x) {even.at<float>(0, x) = static_cast<float>(x + 1);}
  // 0.75 of 4 is 3.
  const PatchStats b = centred_patch_stats(even, 0.75, kClip);
  ASSERT_EQ(b.usable, 3u);
  EXPECT_DOUBLE_EQ(b.median, 2.0) << "columns 0,1,2 hold 1,2,3";
}

TEST(DepthPatch, CountsTheFarClipRatherThanAveragingItIn)
{
  // **The one that matters most.** Half the patch is a surface at 2 m and half is
  // the model saying nothing, written as exactly the clip. Including the clip
  // values would give a median of 4 m — a plausible distance, and one that would
  // make the implied depth_scale come out half what it should be.
  cv::Mat depth = flat(400, 400, 2.0F);
  depth(cv::Rect(0, 0, 400, 200)).setTo(static_cast<float>(kClip));

  const PatchStats s = centred_patch_stats(depth, 0.5, kClip);
  EXPECT_EQ(s.total, 200u * 200u);
  EXPECT_EQ(s.clipped, 100u * 200u);
  EXPECT_EQ(s.usable, 100u * 200u);
  EXPECT_DOUBLE_EQ(s.median, 2.0);
  EXPECT_DOUBLE_EQ(s.clipped_fraction(), 0.5);
}

TEST(DepthPatch, CountsAValueJustPastTheClipAsClippedAndOneJustUnderAsUsable)
{
  // The boundary is `>= max_m`, not `> max_m`: depth_to_metres writes *exactly*
  // max_range for anything past its floor, so the clip value is the common case
  // and an exclusive comparison would let every one of them through.
  cv::Mat at = flat(8, 8, static_cast<float>(kClip));
  EXPECT_EQ(centred_patch_stats(at, 1.0, kClip).clipped, 64u);
  EXPECT_EQ(centred_patch_stats(at, 1.0, kClip).usable, 0u);

  cv::Mat under = flat(8, 8, static_cast<float>(kClip) - 0.001F);
  EXPECT_EQ(centred_patch_stats(under, 1.0, kClip).clipped, 0u);
  EXPECT_EQ(centred_patch_stats(under, 1.0, kClip).usable, 64u);
}

TEST(DepthPatch, ExcludesNonFiniteAndNonPositiveSamples)
{
  cv::Mat depth = flat(10, 10, 2.0F);
  depth.at<float>(0, 0) = std::numeric_limits<float>::quiet_NaN();
  depth.at<float>(0, 1) = std::numeric_limits<float>::infinity();
  depth.at<float>(0, 2) = -std::numeric_limits<float>::infinity();
  depth.at<float>(0, 3) = 0.0F;
  depth.at<float>(0, 4) = -1.0F;

  const PatchStats s = centred_patch_stats(depth, 1.0, kClip);
  EXPECT_EQ(s.total, 100u);
  EXPECT_EQ(s.bad, 5u) << "an infinity is not a large distance and 0 m is not a distance";
  EXPECT_EQ(s.usable, 95u);
  EXPECT_DOUBLE_EQ(s.median, 2.0);
}

TEST(DepthPatch, ReportsAnEmptyPatchAsEmptyAndNotAsZeroMetres)
{
  // Every pixel unusable. The median stays 0.0 — it has to be *something* — so
  // the contract is that a caller branches on `usable`, and this test is what
  // says so out loud.
  const cv::Mat all_clip = flat(16, 16, static_cast<float>(kClip));
  const PatchStats s = centred_patch_stats(all_clip, 1.0, kClip);
  EXPECT_EQ(s.usable, 0u);
  EXPECT_DOUBLE_EQ(s.median, 0.0);
  EXPECT_DOUBLE_EQ(s.iqr(), 0.0);
  EXPECT_DOUBLE_EQ(s.clipped_fraction(), 1.0);
}

TEST(DepthPatch, ReportsTheSpreadOfASlopedSurface)
{
  // A wall at an angle: 1 m on the left, 3 m on the right. The median is the
  // middle and the IQR says the patch is not looking at one distance — which is
  // what a person pointing the camera obliquely at a wall would produce, and what
  // the gate has to be able to complain about.
  cv::Mat depth(100, 100, CV_32FC1);
  for (int y = 0; y < 100; ++y) {
    for (int x = 0; x < 100; ++x) {
      depth.at<float>(y, x) = 1.0F + 2.0F * static_cast<float>(x) / 99.0F;
    }
  }
  const PatchStats s = centred_patch_stats(depth, 1.0, kClip);
  EXPECT_NEAR(s.median, 2.0, 0.05);
  EXPECT_NEAR(s.iqr(), 1.0, 0.05) << "quartiles of a linear ramp are a quarter and three quarters of its span";
}

TEST(DepthPatch, RefusesAMapThatIsNotThirtyTwoBitFloat)
{
  // The one shape error that would otherwise reinterpret bytes: a 16UC1 depth map
  // read as float gives finite, positive, plausible-looking numbers.
  const cv::Mat wrong(32, 32, CV_16UC1, cv::Scalar(2000));
  const PatchStats s = centred_patch_stats(wrong, 1.0, kClip);
  EXPECT_EQ(s.total, 0u);
  EXPECT_EQ(s.usable, 0u);

  EXPECT_EQ(centred_patch_stats(cv::Mat(), 1.0, kClip).total, 0u);
}

TEST(DepthPatch, NeverRoundsThePatchAwayToNothing)
{
  // A fraction small enough to round to zero pixels would report `usable == 0`,
  // which reads as "the surface was out of range" rather than as "you asked for
  // no pixels".
  const cv::Mat depth = flat(100, 100, 2.0F);
  const PatchStats s = centred_patch_stats(depth, 0.0001, kClip);
  EXPECT_EQ(s.total, 1u);
  EXPECT_EQ(s.usable, 1u);
  EXPECT_DOUBLE_EQ(s.median, 2.0);
}
