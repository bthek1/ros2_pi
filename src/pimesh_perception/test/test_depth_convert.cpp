// The two pure steps either side of the depth model.
//
// Neither needs ONNX Runtime, a GPU, or the 99 MB model file: they were pulled
// out of `DepthModel` precisely so that the arithmetic could be handed a value
// whose answer is known. What is being tested is the class of bug this stage is
// most exposed to — the kind that produces a plausible depth map that is
// quietly wrong, and throws nothing on the way.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "pimesh_perception/depth_convert.hpp"

using pimesh_perception::kImagenetMean;
using pimesh_perception::kImagenetStd;
using pimesh_perception::preprocess_frame;
using pimesh_perception::relative_to_metres;

namespace
{
constexpr int kSide = 14;   // one patch: the smallest legal input
}

// --------------------------------------------------------- preprocessing ----

TEST(PreprocessFrame, ProducesOneContiguousPlanePerChannel)
{
  cv::Mat bgr(40, 60, CV_8UC3, cv::Scalar(10, 20, 30));
  std::vector<float> out;
  preprocess_frame(bgr, kSide, out);
  EXPECT_EQ(out.size(), static_cast<size_t>(3 * kSide * kSide));
}

TEST(PreprocessFrame, ConvertsBgrToRgb)
{
  // THE test in this file. OpenCV hands us BGR and the model was trained on
  // RGB; swapping them throws nothing, produces a depth map that looks
  // roughly right, and is wrong in a way no downstream stage can detect.
  //
  // A pure-blue BGR image (255, 0, 0) must land in the LAST plane, because
  // blue is channel 2 of RGB.
  cv::Mat blue(28, 28, CV_8UC3, cv::Scalar(255, 0, 0));
  std::vector<float> out;
  preprocess_frame(blue, kSide, out);

  const size_t plane = kSide * kSide;
  const float r = out[0];
  const float g = out[plane];
  const float b = out[2 * plane];
  EXPECT_LT(r, 0.0f) << "red plane should hold 0, i.e. below the ImageNet mean";
  EXPECT_LT(g, 0.0f) << "green plane should hold 0";
  EXPECT_GT(b, 0.0f) << "blue plane should hold 255 — if this is the red plane, "
                        "the BGR->RGB conversion is missing";
}

TEST(PreprocessFrame, AppliesImagenetNormalisation)
{
  // A known pixel, checked against the arithmetic rather than against itself:
  // (value/255 - mean) / std, per channel.
  cv::Mat grey(28, 28, CV_8UC3, cv::Scalar(128, 128, 128));
  std::vector<float> out;
  preprocess_frame(grey, kSide, out);

  const size_t plane = kSide * kSide;
  for (int c = 0; c < 3; ++c) {
    const float expected = (128.0f / 255.0f - kImagenetMean[c]) / kImagenetStd[c];
    EXPECT_NEAR(out[c * plane], expected, 1e-5) << "channel " << c;
  }
}

TEST(PreprocessFrame, IsPlanarNotInterleaved)
{
  // A left-half-black, right-half-white image. In CHW every plane must show
  // that split independently; in HWC the values would alternate every three
  // elements instead.
  cv::Mat split(28, 28, CV_8UC3, cv::Scalar(0, 0, 0));
  split(cv::Rect(14, 0, 14, 28)).setTo(cv::Scalar(255, 255, 255));
  std::vector<float> out;
  preprocess_frame(split, kSide, out);

  const size_t plane = kSide * kSide;
  for (int c = 0; c < 3; ++c) {
    const float left = out[c * plane + 0];                  // row 0, col 0
    const float right = out[c * plane + (kSide - 1)];        // row 0, last col
    EXPECT_LT(left, right) << "channel " << c << " lost the left/right split";
  }
}

TEST(PreprocessFrame, RejectsASideThatIsNotAMultipleOfFourteen)
{
  // The transformer works on 14x14 patches. A side that does not divide
  // evenly is not a subtle quality loss, it is a shape the model cannot
  // accept — so it fails here, loudly, rather than inside ONNX Runtime.
  cv::Mat bgr(40, 60, CV_8UC3, cv::Scalar(10, 20, 30));
  std::vector<float> out;
  EXPECT_THROW(preprocess_frame(bgr, 100, out), std::invalid_argument);
  EXPECT_THROW(preprocess_frame(bgr, 0, out), std::invalid_argument);
  EXPECT_NO_THROW(preprocess_frame(bgr, 518, out));
}

TEST(PreprocessFrame, RejectsAnEmptyOrWrongTypeImage)
{
  std::vector<float> out;
  cv::Mat empty;
  EXPECT_THROW(preprocess_frame(empty, kSide, out), std::invalid_argument);
  cv::Mat grey(28, 28, CV_8UC1, cv::Scalar(128));
  EXPECT_THROW(preprocess_frame(grey, kSide, out), std::invalid_argument);
}

// ------------------------------------------------------- inverse to metres ----

TEST(RelativeToMetres, InvertsTheModelOutput)
{
  // The model emits relative INVERSE depth: bigger means NEARER. A stage that
  // forgot the inversion would build a room turned inside out.
  cv::Mat relative = (cv::Mat_<float>(1, 3) << 10.0f, 5.0f, 2.5f);
  cv::Mat metres;
  relative_to_metres(relative, 10.0, 100.0, metres);

  EXPECT_NEAR(metres.at<float>(0, 0), 1.0f, 1e-5);
  EXPECT_NEAR(metres.at<float>(0, 1), 2.0f, 1e-5);
  EXPECT_NEAR(metres.at<float>(0, 2), 4.0f, 1e-5);
  EXPECT_LT(metres.at<float>(0, 0), metres.at<float>(0, 2))
    << "a LARGER model output must mean a NEARER surface";
}

TEST(RelativeToMetres, ScalesLinearlyWithDepthScale)
{
  // depth_scale is the one knob a tape measure will move at P5, so doubling it
  // must double every distance and nothing else.
  cv::Mat relative = (cv::Mat_<float>(1, 2) << 4.0f, 8.0f);
  cv::Mat a, b;
  relative_to_metres(relative, 2.0, 100.0, a);
  relative_to_metres(relative, 4.0, 100.0, b);
  for (int i = 0; i < 2; ++i) {
    EXPECT_NEAR(b.at<float>(0, i), 2.0f * a.at<float>(0, i), 1e-5);
  }
}

TEST(RelativeToMetres, BoundsTheResultAtMaxDepth)
{
  cv::Mat relative = (cv::Mat_<float>(1, 4) << 100.0f, 1.0f, 0.01f, 0.0f);
  cv::Mat metres;
  relative_to_metres(relative, 10.0, 6.0, metres);

  for (int i = 0; i < 4; ++i) {
    EXPECT_LE(metres.at<float>(0, i), 6.0f + 1e-4)
      << "index " << i << " exceeded max_depth_m";
  }
  // Near values are untouched by the bound.
  EXPECT_NEAR(metres.at<float>(0, 0), 0.1f, 1e-5);
  // Far ones saturate at exactly the bound rather than at something arbitrary.
  EXPECT_NEAR(metres.at<float>(0, 2), 6.0f, 1e-4);
  EXPECT_NEAR(metres.at<float>(0, 3), 6.0f, 1e-4);
}

TEST(RelativeToMetres, NeverProducesInfinityOrNaN)
{
  // The reason the bound is applied BEFORE the division. Zero and negative
  // values are what the model emits for "background, no idea"; dividing by
  // them first and clipping afterwards leaves inf and NaN to clean up, and a
  // single NaN propagates through a TSDF integration silently.
  cv::Mat relative = (cv::Mat_<float>(1, 5) << 0.0f, -1.0f, -1e-9f, 1e-12f, 3.0f);
  cv::Mat metres;
  relative_to_metres(relative, 10.0, 6.0, metres);

  for (int i = 0; i < 5; ++i) {
    const float v = metres.at<float>(0, i);
    EXPECT_TRUE(std::isfinite(v)) << "index " << i << " produced " << v;
    EXPECT_GT(v, 0.0f) << "index " << i;
    EXPECT_LE(v, 6.0f + 1e-4) << "index " << i;
  }
}

TEST(RelativeToMetres, PreservesShapeAndType)
{
  cv::Mat relative(37, 53, CV_32FC1, cv::Scalar(2.0f));
  cv::Mat metres;
  relative_to_metres(relative, 10.0, 6.0, metres);
  EXPECT_EQ(metres.rows, 37);
  EXPECT_EQ(metres.cols, 53);
  EXPECT_EQ(metres.type(), CV_32FC1) << "the /depth topic is 32FC1";
}

TEST(RelativeToMetres, RejectsNonPositiveParameters)
{
  cv::Mat relative(4, 4, CV_32FC1, cv::Scalar(1.0f));
  cv::Mat metres;
  EXPECT_THROW(relative_to_metres(relative, 0.0, 6.0, metres), std::invalid_argument);
  EXPECT_THROW(relative_to_metres(relative, 10.0, 0.0, metres), std::invalid_argument);
  EXPECT_THROW(relative_to_metres(relative, -1.0, 6.0, metres), std::invalid_argument);
}
