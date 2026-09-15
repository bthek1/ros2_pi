// The arithmetic either side of the depth model, which is all of P4 that can be
// got wrong in silence.
//
// No GPU, no ONNX Runtime, no camera — so this suite runs identically on both
// machines, which is the point of depth_model.hpp being a separate header. Every
// assertion here is about a mistake whose symptom is a *plausible* depth map:
// nothing crashes, nothing logs, and the error shows up as a mesh that is subtly
// the wrong shape several stages later.

#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "opencv2/core.hpp"
#include "pimesh_perception/depth_model.hpp"

using pimesh_perception::kImagenetMean;
using pimesh_perception::kImagenetStd;
using pimesh_perception::kInputElements;
using pimesh_perception::kModelSize;
using pimesh_perception::preprocess;
using pimesh_perception::to_metres;

namespace
{

/// Where channel `c` of pixel (x, y) lands in an NCHW tensor.
std::size_t at(int c, int y, int x)
{
  const std::size_t plane = static_cast<std::size_t>(kModelSize) * kModelSize;
  return c * plane + static_cast<std::size_t>(y) * kModelSize + x;
}

}  // namespace

// --- preprocess --------------------------------------------------------------

TEST(DepthModel, NormalisesAConstantFrameToTheImagenetValue)
{
  // A solid mid-grey. Resizing cannot change a constant image, so every element
  // must be exactly the normalisation of 128/255 for its channel — which makes
  // this the one case where the expected value can be written down in full.
  cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(128, 128, 128));
  std::vector<float> input(kInputElements);
  preprocess(frame, input.data());

  for (int c = 0; c < 3; ++c) {
    const float want = (128.0F / 255.0F - kImagenetMean[c]) / kImagenetStd[c];
    EXPECT_NEAR(input[at(c, 0, 0)], want, 1e-5F) << "channel " << c << " at the origin";
    EXPECT_NEAR(input[at(c, kModelSize - 1, kModelSize - 1)], want, 1e-5F)
      << "channel " << c << " at the far corner";
    EXPECT_NEAR(input[at(c, 259, 259)], want, 1e-5F) << "channel " << c << " in the middle";
  }
}

TEST(DepthModel, PutsRedInPlaneZeroAndBlueInPlaneTwo)
{
  // **The channel-order test, and it is the one that catches the invisible bug.**
  // bgr8 and rgb8 differ by nothing a subscriber can detect, and a model fed
  // channel-swapped input produces a perfectly smooth, confident depth map that
  // is simply wrong. A pure red frame in BGR is (0, 0, 255).
  cv::Mat red(720, 1280, CV_8UC3, cv::Scalar(0, 0, 255));
  std::vector<float> input(kInputElements);
  preprocess(red, input.data());

  const float hot = (1.0F - kImagenetMean[0]) / kImagenetStd[0];
  const float cold_g = (0.0F - kImagenetMean[1]) / kImagenetStd[1];
  const float cold_b = (0.0F - kImagenetMean[2]) / kImagenetStd[2];

  EXPECT_NEAR(input[at(0, 100, 100)], hot, 1e-5F) << "R plane should be saturated";
  EXPECT_NEAR(input[at(1, 100, 100)], cold_g, 1e-5F) << "G plane should be empty";
  EXPECT_NEAR(input[at(2, 100, 100)], cold_b, 1e-5F) << "B plane should be empty";
}

TEST(DepthModel, KeepsThePlanesSeparateRatherThanInterleaving)
{
  // Interleaved-vs-planar has the same failure signature as the channel swap: the
  // model gets a third of each channel in the wrong place and still returns a
  // smooth surface. Distinct per-channel values make the layout readable.
  cv::Mat frame(720, 1280, CV_8UC3, cv::Scalar(30, 60, 90));  // B=30, G=60, R=90
  std::vector<float> input(kInputElements);
  preprocess(frame, input.data());

  const std::size_t plane = static_cast<std::size_t>(kModelSize) * kModelSize;
  // Every element within a plane is equal, and the three planes differ. If the
  // writer had interleaved, neighbouring elements inside plane 0 would cycle
  // through all three channel values instead.
  for (std::size_t i = 0; i < 16; ++i) {
    EXPECT_NEAR(input[i], input[0], 1e-6F) << "plane 0 is not uniform at " << i;
    EXPECT_NEAR(input[plane + i], input[plane], 1e-6F) << "plane 1 is not uniform at " << i;
    EXPECT_NEAR(input[2 * plane + i], input[2 * plane], 1e-6F) << "plane 2 is not uniform";
  }
  EXPECT_GT(input[0], input[plane]) << "R=90 should normalise above G=60";
  EXPECT_GT(input[plane], input[2 * plane]) << "G=60 should normalise above B=30";
}

TEST(DepthModel, ResizesAnyInputToTheModelSize)
{
  // The model takes dynamic spatial dims, so a wrong size here would *run* and
  // degrade quietly. Two aspect ratios, because the resize stretches rather than
  // letterboxing and a letterbox would show up as bars at one of them.
  for (const auto size : {cv::Size(1280, 720), cv::Size(640, 480)}) {
    cv::Mat frame(size, CV_8UC3, cv::Scalar(10, 20, 30));
    std::vector<float> input(kInputElements, std::numeric_limits<float>::quiet_NaN());
    preprocess(frame, input.data());
    for (std::size_t i = 0; i < kInputElements; i += 4099) {
      ASSERT_TRUE(std::isfinite(input[i]))
        << "element " << i << " untouched for " << size.width << "x" << size.height;
    }
  }
}

// --- to_metres ---------------------------------------------------------------

TEST(DepthModel, InvertsRelativeDepthWithTheScale)
{
  cv::Mat relative = (cv::Mat_<float>(1, 3) << 10.0F, 5.0F, 2.0F);
  cv::Mat metres;
  to_metres(relative, 10.0F, 6.0F, metres);

  EXPECT_FLOAT_EQ(metres.at<float>(0, 0), 1.0F);  // 10 / 10
  EXPECT_FLOAT_EQ(metres.at<float>(0, 1), 2.0F);  // 10 / 5
  EXPECT_FLOAT_EQ(metres.at<float>(0, 2), 5.0F);  // 10 / 2
}

TEST(DepthModel, NeverExceedsMaxRangeAndNeverProducesInfinity)
{
  // **The clamp-before-reciprocal test.** Zero is what the model emits for
  // "background, no idea", and 1/0 is not a large number, it is inf — which
  // propagates into a TSDF as NaN and poisons voxels that were fine. Negative and
  // denormal values are in here because nothing guarantees the model's output is
  // positive.
  cv::Mat relative = (cv::Mat_<float>(1, 5) <<
    0.0F, -3.0F, 1e-9F, 1e-30F, 0.5F);
  cv::Mat metres;
  to_metres(relative, 10.0F, 6.0F, metres);

  for (int i = 0; i < 5; ++i) {
    const float v = metres.at<float>(0, i);
    EXPECT_TRUE(std::isfinite(v)) << "element " << i << " is not finite: " << v;
    EXPECT_LE(v, 6.0F) << "element " << i << " exceeds max_range";
    EXPECT_GT(v, 0.0F) << "element " << i << " is not a positive distance";
  }
  // 0.5 with scale 10 would be 20 m, well past the clip, so it lands on the cap.
  EXPECT_FLOAT_EQ(metres.at<float>(0, 4), 6.0F);
}

TEST(DepthModel, MapsNonFiniteInputToMaxRangeRatherThanPropagatingIt)
{
  // Far-away and don't-know are the same answer in this pipeline, and neither of
  // them is NaN. A NaN here survives every downstream comparison silently.
  cv::Mat relative = (cv::Mat_<float>(1, 3) <<
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    -std::numeric_limits<float>::infinity());
  cv::Mat metres;
  to_metres(relative, 10.0F, 6.0F, metres);

  EXPECT_FLOAT_EQ(metres.at<float>(0, 0), 6.0F) << "NaN must not propagate";
  EXPECT_FLOAT_EQ(metres.at<float>(0, 1), 6.0F) << "+inf must not propagate";
  EXPECT_FLOAT_EQ(metres.at<float>(0, 2), 6.0F) << "-inf must not propagate";
}

TEST(DepthModel, ScaleIsALinearFactorOnEveryDistance)
{
  // Monocular depth is scale-ambiguous and depth_scale is the one knob that fixes
  // it. Doubling it must double every distance that is not clipped — which is
  // what makes P5's tape-measure calibration a single multiplication rather than
  // a re-fit.
  cv::Mat relative = (cv::Mat_<float>(1, 3) << 20.0F, 10.0F, 8.0F);
  cv::Mat a;
  cv::Mat b;
  to_metres(relative, 10.0F, 100.0F, a);
  to_metres(relative, 20.0F, 100.0F, b);

  for (int i = 0; i < 3; ++i) {
    EXPECT_FLOAT_EQ(b.at<float>(0, i), 2.0F * a.at<float>(0, i)) << "element " << i;
  }
}

TEST(DepthModel, ProducesA32FC1MatOfTheInputShape)
{
  // The encoding on the wire is 32FC1 and the arithmetic has to actually produce
  // it — a CV_64F here would publish a message whose step is twice what the
  // header says, which shears the image rather than failing.
  cv::Mat relative(kModelSize, kModelSize, CV_32FC1, cv::Scalar(4.0F));
  cv::Mat metres;
  to_metres(relative, 10.0F, 6.0F, metres);

  EXPECT_EQ(metres.type(), CV_32FC1);
  EXPECT_EQ(metres.rows, kModelSize);
  EXPECT_EQ(metres.cols, kModelSize);
  EXPECT_FLOAT_EQ(metres.at<float>(37, 42), 2.5F);
}
