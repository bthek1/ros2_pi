// The bgr8 layout arithmetic, which is the kind of code that is either exactly
// right or quietly produces a plausible image.
//
// A wrong `step` shears the frame by a column per row, which looks like a camera
// fault. A wrong encoding string swaps red and blue, which looks like bad white
// balance and survives all the way to a blue-tinted mesh. A Mat built over a
// message that is too short reads past the end of a vector and usually gets away
// with it. None of these throw.

#include <cstdint>
#include <vector>

#include "gtest/gtest.h"
#include "opencv2/core.hpp"
#include "pimesh_perception/image_buffer.hpp"
#include "sensor_msgs/msg/image.hpp"

using pimesh_perception::fill_bgr8;
using pimesh_perception::mat_over;

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
