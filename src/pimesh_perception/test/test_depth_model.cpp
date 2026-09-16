// The arithmetic either side of the depth model, which is all of P4 that can be
// got wrong in silence.
//
// No GPU, no ONNX Runtime, no camera — so this suite runs identically on both
// machines, which is the point of depth_model.hpp being a separate header. Every
// assertion here is about a mistake whose symptom is a *plausible* depth map:
// nothing crashes, nothing logs, and the error shows up as a mesh that is subtly
// the wrong shape several stages later.

#include <cmath>
#include <cstdint>
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
using pimesh_perception::depth_to_preview_8u;
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

TEST(DepthModel, ClampsExactlyAtTheClipBoundaryAndNotBefore)
{
  // The clamp is written `!(r > floor_inverse)` rather than `r <= floor_inverse`,
  // because every comparison against NaN is false and the negated form catches it.
  // That spelling also decides the boundary itself, which nothing else pins: a
  // relative value of exactly `scale / max_range` is the last one that must land on
  // the cap, and anything above it must be a real distance strictly inside it.
  const float scale = 10.0F;
  const float max_range = 6.0F;
  const float floor_inverse = scale / max_range;           // 1.6667

  cv::Mat relative = (cv::Mat_<float>(1, 3) <<
    floor_inverse,                                          // exactly at the clip
    std::nextafter(floor_inverse, 100.0F),                  // the first value past it
    std::nextafter(floor_inverse, 0.0F));                   // the last value below it
  cv::Mat metres;
  to_metres(relative, scale, max_range, metres);

  EXPECT_FLOAT_EQ(metres.at<float>(0, 0), max_range) << "the boundary itself must clamp";
  EXPECT_LT(metres.at<float>(0, 1), max_range) << "just past the boundary is a real distance";
  EXPECT_GT(metres.at<float>(0, 1), max_range - 0.01F) << "...and only just inside it";
  EXPECT_FLOAT_EQ(metres.at<float>(0, 2), max_range) << "below the boundary must clamp";
}

// --- depth_to_preview_8u -----------------------------------------------------
//
// The colour preview is for a person, but its arithmetic fails silently in the
// most complete way anything here can: every mistake below still produces a
// smooth, room-shaped, entirely convincing picture of the wrong thing.

TEST(DepthPreview, MapsNearToBrightAndFarToBlack)
{
  // The sign of the gain *is* the meaning of the image. Inferno runs black at 0
  // and yellow at 255, so near has to map high and far has to map low — flipped,
  // the 6 m clip (this pipeline's "too far away or no idea") becomes the
  // brightest thing on screen, which is the region carrying the least
  // information.
  cv::Mat metres = (cv::Mat_<float>(1, 3) << 0.0F, 3.0F, 6.0F);
  cv::Mat preview;
  depth_to_preview_8u(metres, 6.0F, preview);

  EXPECT_EQ(preview.type(), CV_8UC1);
  EXPECT_EQ(preview.at<std::uint8_t>(0, 0), 255) << "0 m must be the brightest";
  EXPECT_EQ(preview.at<std::uint8_t>(0, 2), 0) << "the clip must be black";
  EXPECT_NEAR(preview.at<std::uint8_t>(0, 1), 128, 2) << "half range is mid grey";
}

TEST(DepthPreview, IsMonotonicSoAColourIsADistance)
{
  // Whatever else changes, nearer must never be darker than further. This is the
  // property that lets somebody read the picture at all.
  cv::Mat metres(1, 64, CV_32FC1);
  for (int i = 0; i < 64; ++i) {
    metres.at<float>(0, i) = 6.0F * static_cast<float>(i) / 63.0F;
  }
  cv::Mat preview;
  depth_to_preview_8u(metres, 6.0F, preview);

  for (int i = 1; i < 64; ++i) {
    EXPECT_LE(preview.at<std::uint8_t>(0, i), preview.at<std::uint8_t>(0, i - 1))
      << "brightness rose with distance at column " << i;
  }
}

TEST(DepthPreview, SaturatesRatherThanWrappingOutsideTheRange)
{
  // convertTo saturates; a hand-rolled cast would wrap, sending a distance just
  // past the clip back to full brightness — a bright ring around every far
  // surface that reads as an object.
  cv::Mat metres = (cv::Mat_<float>(1, 4) << -5.0F, -0.001F, 6.001F, 1000.0F);
  cv::Mat preview;
  depth_to_preview_8u(metres, 6.0F, preview);

  EXPECT_EQ(preview.at<std::uint8_t>(0, 0), 255) << "nearer than zero clamps bright";
  EXPECT_EQ(preview.at<std::uint8_t>(0, 1), 255);
  EXPECT_EQ(preview.at<std::uint8_t>(0, 2), 0) << "past the clip must stay black";
  EXPECT_EQ(preview.at<std::uint8_t>(0, 3), 0) << "far past the clip must not wrap";
}

TEST(DepthPreview, TheScaleIsFixedRatherThanPerFrame)
{
  // **The failure this guards is RViz's `Normalize Range`**, which rescales every
  // frame to its own min and max — so the same distance is a different shade from
  // one frame to the next and a hand passing the lens re-darkens the whole room.
  // A given distance must come out the same shade whatever else is in the frame.
  cv::Mat lonely = (cv::Mat_<float>(1, 2) << 2.0F, 2.5F);
  cv::Mat varied = (cv::Mat_<float>(1, 4) << 2.0F, 0.1F, 5.9F, 2.5F);

  cv::Mat a;
  cv::Mat b;
  depth_to_preview_8u(lonely, 6.0F, a);
  depth_to_preview_8u(varied, 6.0F, b);

  EXPECT_EQ(a.at<std::uint8_t>(0, 0), b.at<std::uint8_t>(0, 0))
    << "2.0 m rendered differently depending on its neighbours";
  EXPECT_EQ(a.at<std::uint8_t>(0, 1), b.at<std::uint8_t>(0, 3));
}

TEST(DepthPreview, MaxRangeChangesTheMappingAndNothingElseDoes)
{
  // max_range is the one input to this mapping besides the metres, and it has to
  // actually be used — a hard-coded 6.0 would look right for the default config
  // and silently ignore the parameter.
  cv::Mat metres = (cv::Mat_<float>(1, 1) << 3.0F);
  cv::Mat six;
  cv::Mat twelve;
  depth_to_preview_8u(metres, 6.0F, six);
  depth_to_preview_8u(metres, 12.0F, twelve);

  EXPECT_NEAR(six.at<std::uint8_t>(0, 0), 128, 2) << "half of a 6 m range";
  EXPECT_NEAR(twelve.at<std::uint8_t>(0, 0), 191, 2) << "a quarter of a 12 m range";
}
