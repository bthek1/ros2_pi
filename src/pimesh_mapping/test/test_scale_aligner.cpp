// The aligner, against ratio streams whose right answer is known by construction.
//
// **The property this file exists to pin is the one that is invisible in a
// running system: no net push on the map.** An aligner that conforms each frame
// to the map instead of high-passing it looks *better* on every per-frame number
// — the disagreement it reports goes to zero — while the map walks away at about
// 1% a frame. The predecessor measured exactly that and it is why the correction
// is a deviation from a rolling baseline rather than a ratio. A test is the only
// place that distinction can be seen, because both versions produce a plausible
// room and one of them is 60% too big by the end of a sweep.

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "opencv2/core.hpp"
#include "pimesh_mapping/scale_aligner.hpp"

using pimesh_mapping::ScaleAligner;
using pimesh_mapping::depth_ratio;
using pimesh_mapping::surface_gap;

namespace
{

cv::Mat constant(float value, int width = 40, int height = 30)
{
  return cv::Mat(height, width, CV_32FC1, cv::Scalar(value));
}

/// An image that is `value` on `valid_fraction` of its pixels and 0 elsewhere.
cv::Mat partly_valid(float value, double valid_fraction)
{
  cv::Mat image = cv::Mat::zeros(30, 40, CV_32FC1);
  const int total = image.rows * image.cols;
  const int keep = static_cast<int>(valid_fraction * total);
  for (int i = 0; i < keep; ++i) {
    image.at<float>(i / image.cols, i % image.cols) = value;
  }
  return image;
}

}  // namespace

// --- depth_ratio -------------------------------------------------------------

TEST(DepthRatio, TwoConstantImagesGiveTheirExactRatio)
{
  double ratio = 0.0;
  double overlap = 0.0;
  ASSERT_TRUE(depth_ratio(constant(2.2F), constant(2.0F), 0.2, ratio, overlap));
  EXPECT_NEAR(ratio, 1.1, 1e-6);
  EXPECT_NEAR(overlap, 1.0, 1e-9);
}

TEST(DepthRatio, ZeroAndNonFinitePixelsAreNotDistances)
{
  // A 0 from the ray-caster means "nothing there" and a NaN from the depth model
  // means "no reading". Treating either as a distance puts a ratio of 0 or a NaN
  // into the median, and one NaN in a median is a NaN.
  cv::Mat expected = constant(2.2F);
  cv::Mat incoming = constant(2.0F);
  expected.at<float>(0, 0) = 0.0F;
  expected.at<float>(1, 0) = std::numeric_limits<float>::quiet_NaN();
  incoming.at<float>(2, 0) = -1.0F;
  incoming.at<float>(3, 0) = std::numeric_limits<float>::infinity();

  double ratio = 0.0;
  double overlap = 0.0;
  ASSERT_TRUE(depth_ratio(expected, incoming, 0.2, ratio, overlap));
  EXPECT_TRUE(std::isfinite(ratio));
  EXPECT_NEAR(ratio, 1.1, 1e-6);
  EXPECT_LT(overlap, 1.0);
}

TEST(DepthRatio, TooLittleOverlapIsRefusedRatherThanAnsweredFromAHandfulOfPixels)
{
  double ratio = 0.0;
  double overlap = 0.0;
  EXPECT_FALSE(depth_ratio(partly_valid(2.2F, 0.10), constant(2.0F), 0.2, ratio, overlap));
  EXPECT_NEAR(overlap, 0.10, 0.02);
  EXPECT_DOUBLE_EQ(ratio, 1.0) << "a refusal must return the identity, not a guess";

  EXPECT_TRUE(depth_ratio(partly_valid(2.2F, 0.35), constant(2.0F), 0.2, ratio, overlap));
  EXPECT_NEAR(ratio, 1.1, 1e-6);
}

TEST(DepthRatio, TheMedianIgnoresNewGeometryAndFailedPixels)
{
  // **This is why it is a median and not a mean.** A third of the frame here is
  // looking at something the map does not hold — an overlap that reads 10x — and
  // the answer must still be 1.1. A mean comes back at 4.1, and the aligner would
  // then scale the whole frame by the clamp.
  cv::Mat expected = constant(2.2F);
  cv::Mat incoming = constant(2.0F);
  for (int v = 0; v < 10; ++v) {
    for (int u = 0; u < expected.cols; ++u) {
      expected.at<float>(v, u) = 20.0F;
    }
  }

  double ratio = 0.0;
  double overlap = 0.0;
  ASSERT_TRUE(depth_ratio(expected, incoming, 0.2, ratio, overlap));
  EXPECT_NEAR(ratio, 1.1, 1e-6);
}

TEST(DepthRatio, MismatchedShapesAndTypesAreRefused)
{
  double ratio = 0.0;
  double overlap = 0.0;
  EXPECT_FALSE(depth_ratio(constant(2.0F, 40, 30), constant(2.0F, 20, 15), 0.2, ratio, overlap));
  cv::Mat wrong(30, 40, CV_64FC1, cv::Scalar(2.0));
  EXPECT_FALSE(depth_ratio(wrong, constant(2.0F), 0.2, ratio, overlap));
  EXPECT_FALSE(depth_ratio(cv::Mat(), constant(2.0F), 0.2, ratio, overlap));
}

// --- surface_gap -------------------------------------------------------------

TEST(SurfaceGap, TwoSurfacesAKnownDistanceApartReportThatDistance)
{
  // The paired-surface number tools/gates/fusion.sh prints, in metres, with its
  // sign taken off: how far apart do the volume and the new view say the wall is?
  double gap = 0.0;
  double overlap = 0.0;
  ASSERT_TRUE(surface_gap(constant(2.05F), constant(2.0F), 0.2, gap, overlap));
  EXPECT_NEAR(gap, 0.05, 1e-6);

  ASSERT_TRUE(surface_gap(constant(1.95F), constant(2.0F), 0.2, gap, overlap));
  EXPECT_NEAR(gap, 0.05, 1e-6) << "a surface in front and one behind are the same gap";
}

TEST(SurfaceGap, ScalingAWobblyFrameOntoTheMapShrinksIt)
{
  // The experiment the gate runs over a whole clip, in three lines: a frame 10%
  // short of the map disagrees by 20 cm, and the same frame scaled by the ratio
  // the aligner would compute disagrees by nothing.
  const cv::Mat map = constant(2.2F);
  const cv::Mat frame = constant(2.0F);

  double before = 0.0;
  double after = 0.0;
  double overlap = 0.0;
  ASSERT_TRUE(surface_gap(map, frame, 0.2, before, overlap));

  double ratio = 0.0;
  ASSERT_TRUE(depth_ratio(map, frame, 0.2, ratio, overlap));
  cv::Mat scaled;
  frame.convertTo(scaled, CV_32FC1, ratio);
  ASSERT_TRUE(surface_gap(map, scaled, 0.2, after, overlap));

  EXPECT_NEAR(before, 0.2, 1e-6);
  EXPECT_LT(after, before);
  EXPECT_NEAR(after, 0.0, 1e-5);
}

// --- ScaleAligner ------------------------------------------------------------

TEST(ScaleAligner, TheFirstFrameIsNotCorrectedAtAll)
{
  // "The first frame defines the map's scale", from P5. There is nothing to
  // conform to, so the honest answer is the identity and `aligned` false — not a
  // correction of 1.0 that a counter would report as an alignment having happened.
  ScaleAligner aligner;
  cv::Mat nothing = cv::Mat::zeros(30, 40, CV_32FC1);
  const auto result = aligner.scale_for(nothing, constant(2.0F));
  EXPECT_DOUBLE_EQ(result.scale, 1.0);
  EXPECT_FALSE(result.aligned);
  EXPECT_NEAR(result.overlap, 0.0, 1e-9);
  EXPECT_EQ(aligner.history(), 0U)
    << "a frame with no overlap has no opinion and must not enter the baseline";
}

TEST(ScaleAligner, AConstantBiasIsAbsorbedAndNeverFedBack)
{
  // **The property the whole design exists for.** The ray-caster reads
  // systematically far — the predecessor measured ~1.25 voxels, voxel-proportional
  // and drifting as the map fills — so every frame's ratio carries a constant
  // offset. An aligner that applied the ratio would multiply the map by 1.02 every
  // frame and the wall would walk away; this one must converge on exactly 1.0 and
  // push the map nowhere.
  ScaleAligner aligner;
  const cv::Mat map = constant(2.04F);     // 2% "far", every frame, forever
  const cv::Mat frame = constant(2.0F);

  double product = 1.0;
  for (int i = 0; i < 200; ++i) {
    const auto result = aligner.scale_for(map, frame);
    if (i >= 5) {product *= result.scale;}
  }
  EXPECT_NEAR(product, 1.0, 1e-6)
    << "the corrections have a net effect, so the map is walking";
}

TEST(ScaleAligner, PerFrameWobbleIsCorrected)
{
  // What it is *for*: the model's ±4% frame-to-frame scale wobble. The baseline
  // settles on the bias, and each frame's deviation from it is taken out — so a
  // frame that reads 4% short is scaled up by about 4%.
  ScaleAligner aligner;
  const cv::Mat map = constant(2.0F);
  for (int i = 0; i < 60; ++i) {
    aligner.scale_for(map, constant(2.0F));
  }

  const auto shy = aligner.scale_for(map, constant(1.96F));
  ASSERT_TRUE(shy.aligned);
  EXPECT_GT(shy.scale, 1.01) << "a frame reading 4% short was not corrected up";
  EXPECT_NEAR(shy.scale * 1.96, 2.0, 0.01);

  const auto keen = aligner.scale_for(map, constant(2.04F));
  ASSERT_TRUE(keen.aligned);
  EXPECT_LT(keen.scale, 0.995) << "a frame reading 4% long was not corrected down";
}

TEST(ScaleAligner, OneWildFrameIsClampedRatherThanApplied)
{
  // A wrong pose or a failed depth map produces a ratio of 2 with a perfectly
  // healthy-looking overlap. Applying it folds the map in a way no later frame
  // undoes, so the correction is cut at max_correction and the fact is reported.
  ScaleAligner::Options options;
  options.max_correction = 0.15;
  ScaleAligner aligner(options);
  const cv::Mat map = constant(2.0F);
  for (int i = 0; i < 60; ++i) {
    aligner.scale_for(map, constant(2.0F));
  }

  const auto wild = aligner.scale_for(map, constant(1.0F));
  ASSERT_TRUE(wild.aligned);
  EXPECT_TRUE(wild.clamped);
  EXPECT_DOUBLE_EQ(wild.scale, 1.15);

  const auto other_way = aligner.scale_for(map, constant(8.0F));
  EXPECT_TRUE(other_way.clamped);
  EXPECT_DOUBLE_EQ(other_way.scale, 0.85);
}

TEST(ScaleAligner, TheBaselineForgetsAtTheWindowLength)
{
  // The window is the high-pass's time constant. Too long and a real change in
  // the scene is treated as wobble for a minute; too short and the baseline
  // follows the wobble it is meant to be measuring against. This asserts it
  // forgets at all, which a deque with a missing eviction would not.
  ScaleAligner::Options options;
  options.window = 10;
  ScaleAligner aligner(options);
  const cv::Mat map = constant(2.0F);
  for (int i = 0; i < 50; ++i) {
    aligner.scale_for(map, constant(2.0F));
  }
  EXPECT_EQ(aligner.history(), 10U);
}

TEST(ScaleAligner, ResetForgetsEverything)
{
  ScaleAligner aligner;
  for (int i = 0; i < 20; ++i) {
    aligner.scale_for(constant(2.0F), constant(2.0F));
  }
  ASSERT_GT(aligner.history(), 0U);
  aligner.reset();
  EXPECT_EQ(aligner.history(), 0U);
}

TEST(ScaleAligner, ANonsenseRatioIsRefusedRatherThanApplied)
{
  // An all-zero incoming frame has no valid pixels at all, so there is no overlap
  // and no ratio. The identity, and `aligned` false.
  ScaleAligner aligner;
  const auto result = aligner.scale_for(constant(2.0F), cv::Mat::zeros(30, 40, CV_32FC1));
  EXPECT_DOUBLE_EQ(result.scale, 1.0);
  EXPECT_FALSE(result.aligned);
}
