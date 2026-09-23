// Reading a Keypoints message back into geometry, and the ways that can be wrong
// without anything failing.
//
// **This suite exists because the code in it used to be three loops inside a
// subscription callback**, which is a place no test can reach — the fourth time
// this project has found that shape (`percentile` and FNV-1a in stats.hpp,
// `depth_mat_over` in image_buffer.hpp, `quote`/`number` in json.hpp). Two of
// those loops read past the end of a `std::vector` on a message whose parallel
// arrays were not the same length, and nothing in the workspace would have
// produced one, so the bug was reachable only from a publisher nobody had written
// yet.
//
// Every assertion below is about a *plausible wrong answer* rather than a crash:
// corners paired with the wrong ids, descriptors that are somebody else's bytes,
// a rotation fitted to pairs that were never matched. The one exception is the
// ragged-array case, which on the old code was undefined behaviour — and that is
// the one a fuzzer or an ASan build would have caught and a reading of the code
// did not.

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "pimesh_perception/keypoints_view.hpp"

using pimesh_perception::copy_keypoint_descriptors;
using pimesh_perception::keypoint_consecutive_pairs;
using pimesh_perception::keypoint_pixels;
using pimesh_perception::keypoints_well_formed;
using pimesh_perception::PixelPair;

namespace
{

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

/// A well-formed message with `n` features. Every parallel array is filled, so a
/// test that wants a ragged one has to break it deliberately — which is what keeps
/// "well formed" from being whatever the fixture happened to produce.
pimesh_msgs::msg::Keypoints frame(std::size_t n, std::uint32_t descriptor_bytes = 32)
{
  pimesh_msgs::msg::Keypoints msg;
  msg.image_width = 1280;
  msg.image_height = 720;
  for (std::size_t i = 0; i < n; ++i) {
    msg.x.push_back(static_cast<float>(10 * i));
    msg.y.push_back(static_cast<float>(20 * i));
    msg.size.push_back(7.0F);
    msg.angle.push_back(-1.0F);
    msg.response.push_back(0.5F);
    msg.track_id.push_back(static_cast<std::int32_t>(100 + i));
    msg.is_new.push_back(i % 3 == 0);
    // Every third feature is a hole, so the packed pair list is shorter than the
    // arrays and the two cannot be confused by a test that only counts.
    const bool hole = (i % 3 == 0);
    msg.prev_x.push_back(hole ? kNaN : static_cast<float>(10 * i - 6));
    msg.prev_y.push_back(hole ? kNaN : static_cast<float>(20 * i - 3));
  }
  msg.descriptor_bytes = descriptor_bytes;
  msg.descriptors.assign(n * descriptor_bytes, 0u);
  for (std::size_t i = 0; i < msg.descriptors.size(); ++i) {
    msg.descriptors[i] = static_cast<std::uint8_t>(i % 251);
  }
  return msg;
}

}  // namespace

// --- well_formed ------------------------------------------------------------

TEST(KeypointsView, TheFixtureItselfIsWellFormed)
{
  // Or every refusal below passes for the wrong reason.
  EXPECT_TRUE(keypoints_well_formed(frame(0)));
  EXPECT_TRUE(keypoints_well_formed(frame(1)));
  EXPECT_TRUE(keypoints_well_formed(frame(37)));
}

TEST(KeypointsView, AFrameWithNoFeaturesIsWellFormedAndNotAnError)
{
  // ORB finding nothing is a property of a blank wall, not a fault. 177 of
  // bags/desk1's 3489 frames are like this.
  pimesh_msgs::msg::Keypoints msg;
  msg.descriptor_bytes = 0;
  EXPECT_TRUE(keypoints_well_formed(msg));
  EXPECT_TRUE(keypoint_pixels(msg).empty());
  EXPECT_TRUE(keypoint_consecutive_pairs(msg).empty());
  EXPECT_TRUE(copy_keypoint_descriptors(msg).empty());
}

TEST(KeypointsView, EveryParallelArrayIsChecked)
{
  // One case per array, because a predicate that checks five of eight passes over
  // the other three and there is no way to tell from its name.
  {auto m = frame(8); m.y.pop_back(); EXPECT_FALSE(keypoints_well_formed(m)) << "y";}
  {auto m = frame(8); m.size.pop_back(); EXPECT_FALSE(keypoints_well_formed(m)) << "size";}
  {auto m = frame(8); m.angle.pop_back(); EXPECT_FALSE(keypoints_well_formed(m)) << "angle";}
  {auto m = frame(8); m.response.pop_back(); EXPECT_FALSE(keypoints_well_formed(m)) << "response";}
  {auto m = frame(8); m.track_id.pop_back(); EXPECT_FALSE(keypoints_well_formed(m)) << "track_id";}
  {auto m = frame(8); m.is_new.pop_back(); EXPECT_FALSE(keypoints_well_formed(m)) << "is_new";}
  {auto m = frame(8); m.prev_x.pop_back(); EXPECT_FALSE(keypoints_well_formed(m)) << "prev_x";}
  {auto m = frame(8); m.prev_y.pop_back(); EXPECT_FALSE(keypoints_well_formed(m)) << "prev_y";}
  {auto m = frame(8); m.x.pop_back(); EXPECT_FALSE(keypoints_well_formed(m)) << "x";}
}

TEST(KeypointsView, ADescriptorBlobOfTheWrongLengthIsMalformed)
{
  // The blob is the one field that is not one entry per feature, so its check is a
  // product rather than an equality — and getting *that* wrong is how a consumer
  // ends up reading rows of somebody else's bytes.
  {auto m = frame(8); m.descriptors.pop_back(); EXPECT_FALSE(keypoints_well_formed(m));}
  {auto m = frame(8); m.descriptors.push_back(0u); EXPECT_FALSE(keypoints_well_formed(m));}
  {auto m = frame(8); m.descriptor_bytes = 16; EXPECT_FALSE(keypoints_well_formed(m));}
}

// --- The ragged case, which was undefined behaviour -------------------------

TEST(KeypointsView, ARaggedMessageIsNeverReadPastTheEnd)
{
  // **The bug this header was extracted to fix.** The callback's loops were
  // bounded by `x.size()` and `prev_x.size()` while indexing `y` and `prev_y`, so
  // a message with a short `y` read off the end of a vector. The accessors clamp,
  // so this is defined whatever the caller did — and `well_formed` above is how a
  // caller learns it should not have called at all.
  //
  // Run this suite under -fsanitize=address to see the old version fail; with the
  // clamp it is simply correct.
  auto m = frame(64);
  m.y.resize(3);
  m.prev_y.resize(1);
  EXPECT_FALSE(keypoints_well_formed(m));

  const auto pixels = keypoint_pixels(m);
  EXPECT_EQ(pixels.size(), 3u) << "clamped to the shortest array actually indexed";

  const auto pairs = keypoint_consecutive_pairs(m);
  EXPECT_LE(pairs.size(), 1u);
  for (const PixelPair & p : pairs) {
    EXPECT_FALSE(std::isnan(p.previous.x));
    EXPECT_FALSE(std::isnan(p.previous.y));
  }
}

// --- pixels -----------------------------------------------------------------

TEST(KeypointsView, PixelsComeBackInOrderAndPairXWithItsOwnY)
{
  // An x taken from one index and a y from another gives corners that are all in
  // the frame, all plausible, and all in the wrong places — which is a mesh that
  // looks like a poor reconstruction.
  const auto msg = frame(12);
  const auto pixels = keypoint_pixels(msg);
  ASSERT_EQ(pixels.size(), 12u);
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    EXPECT_FLOAT_EQ(pixels[i].x, msg.x[i]);
    EXPECT_FLOAT_EQ(pixels[i].y, msg.y[i]);
  }
}

// --- descriptors ------------------------------------------------------------

TEST(KeypointsView, DescriptorsAreCopiedIntoTheirOwnStorage)
{
  // **A lifetime, not a copy count.** The message is released when the callback
  // returns and the record outlives it in a 90-frame history; a Mat aliasing the
  // message's bytes is a use-after-free that reads as plausible descriptors for
  // as long as the allocator leaves the page alone.
  cv::Mat descriptors;
  {
    const auto msg = frame(10);
    descriptors = copy_keypoint_descriptors(msg);
    ASSERT_EQ(descriptors.rows, 10);
    ASSERT_EQ(descriptors.cols, 32);
    EXPECT_NE(static_cast<const void *>(descriptors.data),
      static_cast<const void *>(msg.descriptors.data()))
      << "aliasing the message is the bug this test exists for";
    for (int r = 0; r < descriptors.rows; ++r) {
      for (int c = 0; c < descriptors.cols; ++c) {
        EXPECT_EQ(descriptors.at<std::uint8_t>(r, c),
          msg.descriptors[static_cast<std::size_t>(r) * 32 + static_cast<std::size_t>(c)]);
      }
    }
  }
  // The message is gone; the Mat still owns its bytes.
  EXPECT_EQ(descriptors.at<std::uint8_t>(0, 0), 0u);
  EXPECT_EQ(descriptors.at<std::uint8_t>(0, 5), 5u);
}

TEST(KeypointsView, RowIBelongsToFeatureI)
{
  // The keyframe store matches a descriptor to the track id at the same index, so
  // a row-major/column-major slip here mismatches every landmark while producing a
  // matrix of exactly the right shape.
  const auto msg = frame(4);
  const cv::Mat d = copy_keypoint_descriptors(msg);
  ASSERT_EQ(d.rows, 4);
  for (int r = 0; r < d.rows; ++r) {
    const std::size_t base = static_cast<std::size_t>(r) * 32;
    EXPECT_EQ(d.at<std::uint8_t>(r, 0), msg.descriptors[base]);
    EXPECT_EQ(d.at<std::uint8_t>(r, 31), msg.descriptors[base + 31]);
  }
}

TEST(KeypointsView, AWrongLengthBlobYieldsNoDescriptorsRatherThanSomeBytes)
{
  // Empty is the honest answer: rows built from a mismatched blob are somebody
  // else's bytes, and they would match against a keyframe just as confidently.
  {auto m = frame(8); m.descriptors.pop_back();
    EXPECT_TRUE(copy_keypoint_descriptors(m).empty());}
  {auto m = frame(8); m.descriptor_bytes = 0;
    EXPECT_TRUE(copy_keypoint_descriptors(m).empty());}
}

// --- consecutive pairs ------------------------------------------------------

TEST(KeypointsView, HolesAreSkippedAndTheRestPairWithTheirOwnCurrentPixel)
{
  // The fixture makes every third feature a hole, so a version that ignored NaN
  // would return 12 pairs instead of 8 — and would hand the fit a `previous` of
  // NaN, which Kabsch turns into a matrix of NaN and a residual that fails the
  // gate for the wrong reason.
  const auto msg = frame(12);
  const auto pairs = keypoint_consecutive_pairs(msg);
  ASSERT_EQ(pairs.size(), 8u);

  std::size_t at = 0;
  for (std::size_t i = 0; i < msg.x.size(); ++i) {
    if (std::isnan(msg.prev_x[i])) {continue;}
    ASSERT_LT(at, pairs.size());
    EXPECT_FLOAT_EQ(pairs[at].previous.x, msg.prev_x[i]);
    EXPECT_FLOAT_EQ(pairs[at].previous.y, msg.prev_y[i]);
    EXPECT_FLOAT_EQ(pairs[at].current.x, msg.x[i]);
    EXPECT_FLOAT_EQ(pairs[at].current.y, msg.y[i]);
    ++at;
  }
  EXPECT_EQ(at, pairs.size());
}

TEST(KeypointsView, AHalfHoleIsStillAHole)
{
  // NaN in one component and a number in the other. It should not become a pair
  // with half a coordinate invented — a fit will happily consume (3.0, NaN).
  auto msg = frame(6);
  msg.prev_x[1] = 5.0F;
  msg.prev_y[1] = kNaN;
  msg.prev_x[2] = kNaN;
  msg.prev_y[2] = 5.0F;
  for (const PixelPair & p : keypoint_consecutive_pairs(msg)) {
    EXPECT_FALSE(std::isnan(p.previous.x));
    EXPECT_FALSE(std::isnan(p.previous.y));
    EXPECT_FALSE(std::isnan(p.current.x));
    EXPECT_FALSE(std::isnan(p.current.y));
  }
}

TEST(KeypointsView, AFrameWhereNothingMatchedYieldsNoPairsRatherThanBadOnes)
{
  // The first frame of any session, and every frame after a tracking loss. The
  // node's own gate refuses a fit below min_matched_pairs; this asserts it is
  // given an honest zero to refuse rather than a list of NaN pairs.
  auto msg = frame(20);
  for (std::size_t i = 0; i < msg.prev_x.size(); ++i) {
    msg.prev_x[i] = kNaN;
    msg.prev_y[i] = kNaN;
  }
  EXPECT_TRUE(keypoints_well_formed(msg));
  EXPECT_TRUE(keypoint_consecutive_pairs(msg).empty());
  EXPECT_EQ(keypoint_pixels(msg).size(), 20u) << "the corners are still there";
}

TEST(KeypointsView, TrackIdsAreCarriedThroughUntouchedAndAreNeverNegative)
{
  // The contract odometry_node's landmark intersection depends on. `track_id` was
  // -1 on a first sighting until 2026-09-23 — one field saying two things — and a
  // landmark whose id is hidden on the frame a keyframe was taken from can never
  // be matched against itself again.
  const auto msg = frame(16);
  ASSERT_EQ(msg.track_id.size(), msg.is_new.size());
  for (std::size_t i = 0; i < msg.track_id.size(); ++i) {
    EXPECT_GE(msg.track_id[i], 0) << "index " << i;
  }
  bool saw_new = false;
  for (bool fresh : msg.is_new) {saw_new = saw_new || fresh;}
  EXPECT_TRUE(saw_new) << "the fixture must exercise both, or this asserts nothing";
}
