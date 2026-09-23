// The bgr8 layout arithmetic, which is the kind of code that is either exactly
// right or quietly produces a plausible image.
//
// A wrong `step` shears the frame by a column per row, which looks like a camera
// fault. A wrong encoding string swaps red and blue, which looks like bad white
// balance and survives all the way to a blue-tinted mesh. A Mat built over a
// message that is too short reads past the end of a vector and usually gets away
// with it. None of these throw.

#include <cstdint>
#include <cstring>
#include <vector>

#include "gtest/gtest.h"
#include "opencv2/core.hpp"
#include "pimesh_core/image_buffer.hpp"
#include "sensor_msgs/msg/image.hpp"

using pimesh_core::depth_mat_over;
using pimesh_core::fill_bgr8;
using pimesh_core::mat_over;

namespace
{

/// A frame whose every pixel is a function of its position, so a shear, a
/// transpose or an off-by-one row shows up as a wrong value rather than as a
/// picture nobody is looking at.
cv::Mat ramp(int rows, int cols)
{
  cv::Mat mat(rows, cols, CV_8UC3);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      mat.at<cv::Vec3b>(r, c) = cv::Vec3b(
        static_cast<std::uint8_t>(r), static_cast<std::uint8_t>(c),
        static_cast<std::uint8_t>(r + c));
    }
  }
  return mat;
}

}  // namespace

TEST(ImageBuffer, RoundTripsPixelForPixel)
{
  const cv::Mat original = ramp(37, 53);   // deliberately not a round number
  sensor_msgs::msg::Image msg;
  fill_bgr8(msg, original);

  EXPECT_EQ(msg.encoding, "bgr8");
  EXPECT_EQ(msg.width, 53u);
  EXPECT_EQ(msg.height, 37u);
  EXPECT_EQ(msg.step, 53u * 3u);
  EXPECT_EQ(msg.data.size(), 37u * 53u * 3u);
  EXPECT_EQ(msg.is_bigendian, 0u);

  const cv::Mat back = mat_over(msg);
  ASSERT_FALSE(back.empty());
  EXPECT_EQ(cv::countNonZero(cv::Mat(back != original).reshape(1)), 0);
}

TEST(ImageBuffer, MatSharesTheMessagesMemory)
{
  // The claim the whole pipeline is built on: this conversion is a header, not a
  // copy. If it ever becomes a copy, every stage downstream silently pays 2.7 MB
  // per frame, and the only symptom is that the numbers get worse.
  sensor_msgs::msg::Image msg;
  fill_bgr8(msg, ramp(8, 8));

  cv::Mat view = mat_over(msg);
  ASSERT_FALSE(view.empty());
  EXPECT_EQ(view.data, msg.data.data());

  view.at<cv::Vec3b>(3, 4) = cv::Vec3b(1, 2, 3);
  const std::size_t offset = 3u * msg.step + 4u * 3u;
  EXPECT_EQ(msg.data[offset + 0], 1u);
  EXPECT_EQ(msg.data[offset + 1], 2u);
  EXPECT_EQ(msg.data[offset + 2], 3u);
}

TEST(ImageBuffer, CopiesRowByRowFromANonContiguousMat)
{
  // A cropped Mat has the parent's stride. The obvious one-memcpy version of
  // fill_bgr8 reads the wrong bytes for every row but the first, and the result
  // is a sheared image that still decodes, still detects corners, and is wrong.
  const cv::Mat parent = ramp(20, 20);
  const cv::Mat crop = parent(cv::Rect(2, 3, 9, 7));
  ASSERT_FALSE(crop.isContinuous());

  sensor_msgs::msg::Image msg;
  fill_bgr8(msg, crop);
  const cv::Mat back = mat_over(msg);
  ASSERT_FALSE(back.empty());
  EXPECT_EQ(cv::countNonZero(cv::Mat(back != crop).reshape(1)), 0);
}

TEST(ImageBuffer, RefusesAnythingItCannotVouchFor)
{
  sensor_msgs::msg::Image msg;
  fill_bgr8(msg, ramp(4, 4));

  // Wrong encoding. Returning a Mat anyway would reinterpret whatever bytes are
  // there — a 16-bit depth map read as BGR, for instance, which produces corners.
  auto wrong_encoding = msg;
  wrong_encoding.encoding = "rgb8";
  EXPECT_TRUE(mat_over(wrong_encoding).empty());

  // Truncated payload: the message says 4 rows and carries 2.
  auto truncated = msg;
  truncated.data.resize(truncated.data.size() / 2);
  EXPECT_TRUE(mat_over(truncated).empty());

  // A step narrower than a row of pixels is arithmetically impossible, and the
  // Mat it would produce overlaps its own rows.
  auto narrow = msg;
  narrow.step = narrow.width * 3 - 1;
  EXPECT_TRUE(mat_over(narrow).empty());

  auto empty_dims = msg;
  empty_dims.width = 0;
  EXPECT_TRUE(mat_over(empty_dims).empty());
}

TEST(ImageBuffer, AcceptsAPaddedStep)
{
  // A publisher is allowed to pad rows, and a reader that assumes width*3 reads
  // a progressively shifted image. Nothing in the message says "no padding".
  sensor_msgs::msg::Image msg;
  msg.encoding = "bgr8";
  msg.width = 5;
  msg.height = 4;
  msg.step = 5 * 3 + 7;
  msg.data.assign(static_cast<std::size_t>(msg.step) * msg.height, 0u);
  msg.data[msg.step * 2 + 3 * 3 + 1] = 42u;   // row 2, col 3, green

  const cv::Mat view = mat_over(msg);
  ASSERT_FALSE(view.empty());
  EXPECT_EQ(view.at<cv::Vec3b>(2, 3)[1], 42u);
}

// --- 32FC1 -------------------------------------------------------------------
//
// The same arithmetic over four-byte pixels, and it had no tests until
// 2026-09-19 because it had no home: it sat in an anonymous namespace inside
// fusion_node.cpp, one translation unit away from anything that could call it.
// Every distance the TSDF integrates comes through it.

namespace
{

/// A depth map whose every pixel is a function of its position, in metres.
///
/// Distinct per pixel and monotonic in both axes, so a shear, a transpose or a
/// row read one short lands on a value that is wrong rather than on one that is
/// merely from somewhere else. `0.5 + r + c/100` keeps every value inside the
/// range a real depth map holds, which is what the callers clamp against.
sensor_msgs::msg::Image depth_ramp(int rows, int cols, std::uint32_t pad = 0)
{
  sensor_msgs::msg::Image msg;
  msg.encoding = "32FC1";
  msg.height = static_cast<std::uint32_t>(rows);
  msg.width = static_cast<std::uint32_t>(cols);
  msg.step = static_cast<std::uint32_t>(cols) * sizeof(float) + pad;
  msg.data.assign(static_cast<std::size_t>(msg.step) * msg.height, 0u);
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const float metres = 0.5F + static_cast<float>(r) + static_cast<float>(c) / 100.0F;
      std::memcpy(
        msg.data.data() + static_cast<std::size_t>(r) * msg.step + c * sizeof(float),
        &metres, sizeof(metres));
    }
  }
  return msg;
}

}  // namespace

TEST(DepthBuffer, ReadsEveryPixelWhereTheMessagePutIt)
{
  const sensor_msgs::msg::Image msg = depth_ramp(7, 11);
  const cv::Mat view = depth_mat_over(msg);
  ASSERT_FALSE(view.empty());
  ASSERT_EQ(view.type(), CV_32FC1);
  EXPECT_EQ(view.rows, 7);
  EXPECT_EQ(view.cols, 11);

  for (int r = 0; r < view.rows; ++r) {
    for (int c = 0; c < view.cols; ++c) {
      EXPECT_FLOAT_EQ(view.at<float>(r, c), 0.5F + static_cast<float>(r) + c / 100.0F)
        << "at (" << r << ", " << c << ")";
    }
  }
}

TEST(DepthBuffer, MatSharesTheMessagesMemory)
{
  // Same claim as the bgr8 case, and it matters more here: fusion_node hands this
  // Mat straight to the integrator, one per frame at 17 Hz.
  sensor_msgs::msg::Image msg = depth_ramp(4, 4);
  const cv::Mat view = depth_mat_over(msg);
  ASSERT_FALSE(view.empty());
  EXPECT_EQ(reinterpret_cast<const void *>(view.data), msg.data.data());
}

TEST(DepthBuffer, AcceptsAPaddedStep)
{
  // The failure a padded row causes on a float image is the one worth naming: not
  // a shear, but a map whose columns drift by a pixel per row — a room of
  // diagonal streaks, which reads as a depth model producing noise.
  const sensor_msgs::msg::Image msg = depth_ramp(5, 6, /*pad=*/12);
  const cv::Mat view = depth_mat_over(msg);
  ASSERT_FALSE(view.empty());
  EXPECT_FLOAT_EQ(view.at<float>(3, 4), 0.5F + 3.0F + 0.04F);
}

TEST(DepthBuffer, RefusesAnythingItCannotVouchFor)
{
  const sensor_msgs::msg::Image good = depth_ramp(4, 4);

  // A depth map is 32FC1 and nothing else. 16UC1 millimetres is the other common
  // spelling, and reading its shorts as floats gives distances around 1e-41 — a
  // volume that integrates nothing, with no error anywhere.
  auto wrong_encoding = good;
  wrong_encoding.encoding = "16UC1";
  EXPECT_TRUE(depth_mat_over(wrong_encoding).empty());

  auto bgr = good;
  bgr.encoding = "bgr8";
  EXPECT_TRUE(depth_mat_over(bgr).empty());

  auto truncated = good;
  truncated.data.resize(truncated.data.size() / 2);
  EXPECT_TRUE(depth_mat_over(truncated).empty());

  // Narrower than a row of floats: the Mat would overlap its own rows.
  auto narrow = good;
  narrow.step = narrow.width * sizeof(float) - 1;
  EXPECT_TRUE(depth_mat_over(narrow).empty());

  auto no_width = good;
  no_width.width = 0;
  EXPECT_TRUE(depth_mat_over(no_width).empty());

  auto no_height = good;
  no_height.height = 0;
  EXPECT_TRUE(depth_mat_over(no_height).empty());
}

TEST(DepthBuffer, IsStricterThanMatOverAboutAZeroStep)
{
  // The one place the two disagree, pinned so that a later tidy-up has to be
  // deliberate. `mat_over` reads step 0 as "not set" and derives width*3, because
  // bgr8 arrives from publishers this workspace does not own; the only 32FC1
  // publisher here is depth_node, which always sets it, so a zero is a malformed
  // message rather than an omission.
  //
  // Asserted as a *pair* rather than one refusal, because what is worth pinning
  // is the difference: a test that only said "32FC1 refuses step 0" would be
  // satisfied by both functions refusing, and the asymmetry would quietly
  // disappear the first time somebody made them consistent.
  sensor_msgs::msg::Image depth = depth_ramp(3, 3);
  depth.step = 0;
  EXPECT_TRUE(depth_mat_over(depth).empty()) << "32FC1 should refuse a step of 0";

  sensor_msgs::msg::Image colour;
  colour.encoding = "bgr8";
  colour.width = 3;
  colour.height = 3;
  colour.step = 0;
  colour.data.assign(3u * 3u * 3u, 7u);
  EXPECT_FALSE(mat_over(colour).empty()) << "bgr8 should derive a step of 0 from width";
}
