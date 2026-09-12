// The tracker, against synthetic frames whose true motion is known.
//
// What makes this worth testing is that the tracker cannot fail outright. A
// matcher with the wrong norm, a window that never fills, a one-to-many match
// assignment — all of them still produce keypoints, still draw green circles in the
// preview, and still publish a plausible message. They show up as a mesh that
// drifts, several milestones downstream, with nothing in between to point at.
//
// The frames here are a textured pattern translated by a known amount. Translation
// rather than rotation, because it is the motion a unit test can generate exactly
// with cv::warpAffine; what the tracker is asked is whether it recognises the same
// corners afterwards, which does not depend on what kind of motion moved them.

#include <cstdint>
#include <set>
#include <vector>

#include "gtest/gtest.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "pimesh_perception/orb_tracker.hpp"

using pimesh_perception::OrbTracker;
using pimesh_perception::TrackedFrame;

namespace
{

/// A field of blobs at irregular spacing: plenty of corners, none of them
/// repeating, so a match can be wrong without being ambiguous. A chequerboard would
/// be the worst possible fixture here — every corner looks like every other one.
cv::Mat textured(int rows = 480, int cols = 640)
{
  cv::Mat image(rows, cols, CV_8UC1, cv::Scalar(30));
  cv::RNG rng(20260912);
  for (int i = 0; i < 400; ++i) {
    const cv::Point centre(rng.uniform(10, cols - 10), rng.uniform(10, rows - 10));
    cv::circle(image, centre, rng.uniform(2, 7), cv::Scalar(rng.uniform(90, 255)), -1);
  }
  cv::GaussianBlur(image, image, cv::Size(3, 3), 0.8);
  return image;
}

cv::Mat shifted(const cv::Mat & src, double dx, double dy)
{
  cv::Mat out;
  const cv::Matx23d shift(1.0, 0.0, dx, 0.0, 1.0, dy);
  cv::warpAffine(src, out, cv::Mat(shift), src.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT);
  return out;
}

OrbTracker::Config default_config()
{
  OrbTracker::Config config;    // 500 features, window 10, Hamming 64 — P3's numbers
  return config;
}

}  // namespace

TEST(OrbTracker, FirstFrameMatchesNothingAndInventsEverything)
{
  OrbTracker tracker(default_config());
  const TrackedFrame frame = tracker.process(textured());

  ASSERT_GT(frame.keypoints.size(), 100u);
  EXPECT_EQ(frame.matched(), 0u);
  EXPECT_DOUBLE_EQ(frame.matched_fraction(), 0.0);
  EXPECT_TRUE(frame.consecutive_pairs.empty());

  // Every feature still gets an id, so the *next* frame has something to inherit.
  // Published, though, they are all -1: this frame is the first sighting of each.
  for (std::int32_t id : frame.ids) {EXPECT_GE(id, 0);}
  for (std::int32_t id : frame.published_track_ids()) {EXPECT_EQ(id, -1);}
}

TEST(OrbTracker, DescriptorsAreThirtyTwoByteRowsPerKeypoint)
{
  // The message carries `descriptor_bytes` and a flat blob of
  // descriptor_bytes * features. If ORB ever handed back a different width, that
  // blob's layout would be wrong and every consumer would read shifted bytes.
  OrbTracker tracker(default_config());
  const TrackedFrame frame = tracker.process(textured());
  EXPECT_EQ(frame.descriptors.cols, 32);
  EXPECT_EQ(frame.descriptors.rows, static_cast<int>(frame.keypoints.size()));
  EXPECT_EQ(frame.descriptors.type(), CV_8U);
}

TEST(OrbTracker, RecognisesMostCornersAfterASmallShift)
{
  OrbTracker tracker(default_config());
  const cv::Mat first = textured();
  tracker.process(first);
  const TrackedFrame second = tracker.process(shifted(first, 4.0, 2.0));

  // The actual claim of P3: a few hundred corners, most of them recognised. The
  // floor is deliberately well below what this fixture achieves — it is here to
  // catch a tracker that has stopped matching, not to pin a number that depends on
  // OpenCV's version.
  EXPECT_GT(second.matched_fraction(), 0.6)
    << second.matched() << " of " << second.keypoints.size() << " matched";
  EXPECT_GE(second.consecutive_pairs.size(), 8u);
}

TEST(OrbTracker, PairsPointAtTheRealDisplacement)
{
  // A pair is only useful if it says where the corner went. This checks the
  // displacement the tracker reports against the shift that was applied — the one
  // assertion that catches pairs whose two halves come from the wrong frames, which
  // is the failure the PixelPair form of the API exists to prevent.
  OrbTracker tracker(default_config());
  const cv::Mat first = textured();
  tracker.process(first);
  const TrackedFrame second = tracker.process(shifted(first, 6.0, 0.0));

  ASSERT_GE(second.consecutive_pairs.size(), 20u);
  std::vector<double> dx;
  for (const auto & pair : second.consecutive_pairs) {
    dx.push_back(static_cast<double>(pair.current.x - pair.previous.x));
  }
  std::sort(dx.begin(), dx.end());
  const double median = dx[dx.size() / 2];
  // warpAffine shifts the *content* by +6 px, so a corner moves +6 px.
  EXPECT_NEAR(median, 6.0, 1.0) << "median reported displacement " << median;
}

TEST(OrbTracker, TrackIdsPersistAcrossFrames)
{
  OrbTracker tracker(default_config());
  const cv::Mat first = textured();
  const TrackedFrame f1 = tracker.process(first);
  const TrackedFrame f2 = tracker.process(shifted(first, 3.0, 1.0));
  const TrackedFrame f3 = tracker.process(shifted(first, 6.0, 2.0));

  const std::set<std::int32_t> ids1(f1.ids.begin(), f1.ids.end());
  std::size_t carried = 0;
  for (std::size_t i = 0; i < f3.ids.size(); ++i) {
    if (!f3.is_new[i] && ids1.count(f3.ids[i]) > 0) {++carried;}
  }
  // A track that survives two frames is the thing that makes this tracking rather
  // than per-frame detection.
  EXPECT_GT(carried, 50u) << carried << " tracks from frame 1 still alive in frame 3";
  EXPECT_GT(f2.matched(), 0u);
}

TEST(OrbTracker, TrackIdsAreNeverSharedWithinAFrame)
{
  // The one-to-one rule. Plain nearest-neighbour matching is many-to-one: two
  // corners can both name the same older feature, and both would inherit its id —
  // one track at two places at once, which no consumer can detect and the geometry
  // cannot express.
  OrbTracker tracker(default_config());
  const cv::Mat first = textured();
  tracker.process(first);
  const TrackedFrame second = tracker.process(shifted(first, 2.0, 2.0));

  std::set<std::int32_t> seen;
  for (std::size_t i = 0; i < second.ids.size(); ++i) {
    if (second.is_new[i]) {continue;}
    EXPECT_TRUE(seen.insert(second.ids[i]).second)
      << "track " << second.ids[i] << " claimed twice in one frame";
  }
}

TEST(OrbTracker, IdsAreNeverReused)
{
  OrbTracker tracker(default_config());
  const cv::Mat first = textured();
  tracker.process(first);
  const std::int32_t after_first = tracker.tracks_created();
  tracker.process(shifted(first, 40.0, 40.0));

  EXPECT_GT(tracker.tracks_created(), after_first);
  // A reset clears the window but not the counter: an id that came back after a
  // reset would be indistinguishable, to anything that had recorded the first one,
  // from the original track.
  tracker.reset();
  EXPECT_EQ(tracker.window_size(), 0u);
  const std::int32_t before = tracker.tracks_created();
  tracker.process(first);
  EXPECT_GE(tracker.tracks_created(), before);
  const TrackedFrame after = tracker.process(shifted(first, 1.0, 0.0));
  for (std::size_t i = 0; i < after.ids.size(); ++i) {
    if (after.is_new[i]) {EXPECT_GE(after.ids[i], before);}
  }
}

TEST(OrbTracker, AWindowForgivesDetectionChurn)
{
  // The reason the window exists. A feature that drops out for one frame and comes
  // back is *detection churn* at the feature cap, not motion — and strict
  // frame-to-frame matching counts it as a new corner. A blank frame in the middle
  // is the extreme version: with a window, the frame after it still recognises the
  // room; with a window of 1, it cannot.
  const cv::Mat first = textured();
  const cv::Mat blank(first.size(), CV_8UC1, cv::Scalar(30));

  OrbTracker::Config windowed = default_config();
  OrbTracker::Config strict = default_config();
  strict.match_window = 1;

  OrbTracker with_window(windowed);
  OrbTracker without(strict);

  with_window.process(first);
  without.process(first);
  with_window.process(blank);
  without.process(blank);

  const TrackedFrame recovered = with_window.process(shifted(first, 1.0, 0.0));
  const TrackedFrame lost = without.process(shifted(first, 1.0, 0.0));

  EXPECT_GT(recovered.matched_fraction(), 0.5);
  EXPECT_LT(lost.matched_fraction(), 0.1);
  EXPECT_GT(recovered.matched_fraction(), lost.matched_fraction() + 0.25)
    << "windowed " << recovered.matched_fraction() << " vs strict " << lost.matched_fraction();
}

TEST(OrbTracker, MatchesNothingBetweenUnrelatedScenes)
{
  // The Hamming threshold earning its keep. Two different rooms must not match: a
  // tracker that matches anything to anything produces a high matched fraction, a
  // low residual, and a pose that is pure fiction.
  OrbTracker tracker(default_config());
  cv::Mat a = textured();
  cv::Mat b(a.size(), CV_8UC1, cv::Scalar(20));
  cv::RNG rng(777);
  for (int i = 0; i < 300; ++i) {
    cv::rectangle(
      b, cv::Point(rng.uniform(0, 600), rng.uniform(0, 440)),
      cv::Point(rng.uniform(0, 620), rng.uniform(0, 460)),
      cv::Scalar(rng.uniform(100, 255)), -1);
  }

  tracker.process(a);
  const TrackedFrame unrelated = tracker.process(b);
  EXPECT_LT(unrelated.matched_fraction(), 0.25)
    << unrelated.matched() << " of " << unrelated.keypoints.size() << " matched across scenes";
}

TEST(OrbTracker, RespectsTheFeatureCap)
{
  OrbTracker::Config config = default_config();
  config.max_features = 120;
  OrbTracker tracker(config);
  const TrackedFrame frame = tracker.process(textured());
  // `nfeatures` is a target rather than a hard ceiling: ORB distributes the quota
  // across its pyramid levels and rounds up per level, so asking for 120 returned
  // 121 here. Measured, not assumed — which is why this allows a small overshoot
  // instead of asserting an exact cap that happens to hold on one OpenCV version.
  EXPECT_LE(frame.keypoints.size(), 132u);
  EXPECT_GT(frame.keypoints.size(), 50u);
}

TEST(OrbTracker, WindowIsBoundedByItsConfiguredLength)
{
  // An unbounded window is a memory leak that looks like improving performance: the
  // matched fraction climbs, the cost climbs with it, and the node dies in an hour.
  OrbTracker::Config config = default_config();
  config.match_window = 4;
  OrbTracker tracker(config);
  const cv::Mat first = textured();
  for (int i = 0; i < 12; ++i) {tracker.process(shifted(first, i, 0.0));}
  EXPECT_EQ(tracker.window_size(), 4u);
}

TEST(OrbTracker, SurvivesAFrameWithNoFeaturesAtAll)
{
  // A lens cap, or a camera pointed at a blank wall in the dark. detectAndCompute
  // returns an empty descriptor matrix, and every matcher call below would throw on
  // it — which in the node means a worker thread dying and a pipeline that stops
  // with no error anywhere.
  OrbTracker tracker(default_config());
  const cv::Mat blank(480, 640, CV_8UC1, cv::Scalar(0));

  TrackedFrame frame;
  ASSERT_NO_THROW(frame = tracker.process(blank));
  EXPECT_TRUE(frame.keypoints.empty());
  EXPECT_DOUBLE_EQ(frame.matched_fraction(), 0.0);

  // And it recovers: the blank frame must not poison the window.
  ASSERT_NO_THROW(tracker.process(textured()));
  const TrackedFrame after = tracker.process(shifted(textured(), 2.0, 0.0));
  EXPECT_GT(after.matched_fraction(), 0.5);
}
