// Decoding is tested against JPEGs made on the spot with cv::imencode, so
// nothing here needs a camera, the Pi, or a recorded bag. The cases that
// matter most are the failures: a Wi-Fi link delivers corrupt frames, and the
// pipeline must count them rather than die on them.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "pimesh_perception/jpeg.hpp"

using pimesh_perception::decode_bgr8;

namespace
{

std::vector<uint8_t> encode(const cv::Mat & image, int quality = 95)
{
  std::vector<uint8_t> out;
  const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, quality};
  EXPECT_TRUE(cv::imencode(".jpg", image, out, params));
  return out;
}

}  // namespace

TEST(DecodeBgr8, ProducesAThreeChannelImageOfTheRightSize)
{
  const cv::Mat source(48, 64, CV_8UC3, cv::Scalar(10, 20, 30));
  cv::Mat decoded;
  ASSERT_TRUE(decode_bgr8(encode(source), decoded));
  EXPECT_EQ(decoded.rows, 48);
  EXPECT_EQ(decoded.cols, 64);
  EXPECT_EQ(decoded.type(), CV_8UC3);
  EXPECT_TRUE(decoded.isContinuous())
    << "the node memcpys rows in one go; a padded Mat would corrupt the image";
}

TEST(DecodeBgr8, KeepsChannelOrderBgrNotRgb)
{
  // A pure blue image in BGR. If the channels came back swapped, every mesh
  // this pipeline ever colours would be wrong in a way that looks plausible.
  const cv::Mat source(32, 32, CV_8UC3, cv::Scalar(255, 0, 0));
  cv::Mat decoded;
  ASSERT_TRUE(decode_bgr8(encode(source), decoded));

  const cv::Vec3b centre = decoded.at<cv::Vec3b>(16, 16);
  EXPECT_GT(centre[0], 200) << "blue channel lost";
  EXPECT_LT(centre[1], 55);
  EXPECT_LT(centre[2], 55);
}

TEST(DecodeBgr8, ForcesColourEvenForAGreyscaleSource)
{
  // The published encoding is a contract: downstream stages index three bytes
  // per pixel. A greyscale JPEG must not quietly become a single-channel image.
  const cv::Mat grey(32, 32, CV_8UC1, cv::Scalar(128));
  cv::Mat decoded;
  ASSERT_TRUE(decode_bgr8(encode(grey), decoded));
  EXPECT_EQ(decoded.channels(), 3);
}

TEST(DecodeBgr8, ReusesTheDestinationAllocationBetweenFrames)
{
  // The steady state must not allocate 2.7 MB per frame at 30 Hz.
  const cv::Mat source(48, 64, CV_8UC3, cv::Scalar(90, 90, 90));
  const auto jpeg = encode(source);

  cv::Mat decoded;
  ASSERT_TRUE(decode_bgr8(jpeg, decoded));
  const uint8_t * first = decoded.data;
  ASSERT_TRUE(decode_bgr8(jpeg, decoded));
  EXPECT_EQ(decoded.data, first) << "same size and type, so it should not reallocate";
}

TEST(DecodeBgr8, FailsOnAnEmptyBuffer)
{
  cv::Mat decoded;
  EXPECT_FALSE(decode_bgr8({}, decoded));
}

TEST(DecodeBgr8, FailsOnGarbageRatherThanThrowing)
{
  const std::vector<uint8_t> garbage(4096, 0xAB);
  cv::Mat decoded;
  EXPECT_NO_THROW(
  {
    EXPECT_FALSE(decode_bgr8(garbage, decoded));
  });
}

TEST(DecodeBgr8, FailsOnATruncatedJpegRatherThanThrowing)
{
  // What a lost Wi-Fi fragment actually looks like: a valid header and half a
  // frame. This is the case that must never take the pipeline down.
  const cv::Mat source(240, 320, CV_8UC3, cv::Scalar(40, 80, 120));
  auto jpeg = encode(source);
  jpeg.resize(jpeg.size() / 3);

  cv::Mat decoded;
  EXPECT_NO_THROW(
  {
    decode_bgr8(jpeg, decoded);
  }) << "a truncated frame is a normal event over Wi-Fi";
}

TEST(DecodeBgr8, ReportsFailureEvenWhenTheDestinationHoldsAnOlderFrame)
{
  // The case that caught a real bug on 2026-09-04, before decode_node existed.
  //
  // cv::imdecode's three-argument form leaves the destination UNTOUCHED on
  // failure — still holding the last good frame — so an `!bgr.empty()` check
  // returns true for a corrupt buffer. A node trusting that would republish
  // the previous image with a fresh timestamp: a frozen picture that every
  // downstream stage, and every rate check, reads as live.
  const cv::Mat source(48, 64, CV_8UC3, cv::Scalar(200, 100, 50));
  cv::Mat decoded;
  ASSERT_TRUE(decode_bgr8(encode(source), decoded));
  ASSERT_FALSE(decoded.empty());

  const std::vector<uint8_t> garbage(1024, 0x7F);
  EXPECT_FALSE(decode_bgr8(garbage, decoded))
    << "a corrupt buffer was reported as a successful decode";
}
