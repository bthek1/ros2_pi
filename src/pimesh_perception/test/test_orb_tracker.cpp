// The tracker, driven by synthetic images.
//
// A scene of drawn blobs on noise gives ORB something real to find, and
// translating that scene by a known amount gives the matcher something real to
// match — without a camera, a room, or the Pi. What is being tested is not
// OpenCV's ORB (that is upstream's job) but the bookkeeping around it: that the
// two matchings answer their two different questions, that track ids follow a
// point rather than an index, and that the arrays a consumer reads stay in step
// with each other.

#include <vector>

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "pimesh_perception/orb_tracker.hpp"

using pimesh_perception::OrbTracker;
using pimesh_perception::TrackedFrame;

namespace
{

/// A repeatable textured scene. Noise alone gives ORB nothing stable to hold
/// on to between frames, so the corners are drawn explicitly on top of it.
cv::Mat scene(int width = 640, int height = 480, unsigned seed = 5)
{
  cv::Mat image(height, width, CV_8UC1);
  cv::RNG rng(seed);
  rng.fill(image, cv::RNG::UNIFORM, 0, 60);
  cv::RNG shapes(seed + 1);
  for (int i = 0; i < 120; ++i) {
    const int x = shapes.uniform(30, width - 30);
    const int y = shapes.uniform(30, height - 30);
    cv::rectangle(
      image, cv::Rect(x, y, shapes.uniform(6, 18), shapes.uniform(6, 18)),
      cv::Scalar(shapes.uniform(120, 255)), cv::FILLED);
  }
  return image;
}

/// The same scene shifted, which is what a small camera pan looks like.
cv::Mat shifted(const cv::Mat & src, double dx, double dy)
{
  cv::Mat out;
  const cv::Matx23d m(1.0, 0.0, dx, 0.0, 1.0, dy);
  cv::warpAffine(src, out, cv::Mat(m), src.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT);
  return out;
}

OrbTracker::Options small_options()
{
  OrbTracker::Options o;
  o.max_features = 200;   // enough to be meaningful, fast enough for a test
  o.window = 5;
  o.max_distance = 64;
  return o;
}

}  // namespace

TEST(OrbTracker, FirstFrameDetectsButMatchesNothing)
{
  OrbTracker tracker(small_options());
  const TrackedFrame f = tracker.track(scene());

  EXPECT_GT(f.count(), 50u) << "the synthetic scene should be rich enough to detect in";
  // Nothing to match against yet, and saying so is different from saying
  // "everything is new" — the caller must be able to tell startup from loss.
  EXPECT_EQ(f.recent_match_count, 0u);
  EXPECT_TRUE(f.prev_matched.empty());
  EXPECT_TRUE(f.curr_matched.empty());
  for (const auto index : f.match_index) {
    EXPECT_EQ(index, -1);
  }
}

TEST(OrbTracker, EveryArrayStaysInStepWithTheKeypoints)
{
  // The Keypoints message is parallel flat arrays; a consumer indexes all of
  // them with one loop counter. A length mismatch here is a wrong-answer bug
  // that would surface as features drawn in the wrong place, or worse, a
  // silently misread descriptor.
  OrbTracker tracker(small_options());
  const cv::Mat base = scene();
  tracker.track(base);
  const TrackedFrame f = tracker.track(shifted(base, 4.0, -2.0));

  ASSERT_GT(f.count(), 0u);
  EXPECT_EQ(f.match_index.size(), f.count());
  EXPECT_EQ(f.track_id.size(), f.count());
  EXPECT_EQ(f.recently_seen.size(), f.count());
  EXPECT_EQ(static_cast<std::size_t>(f.descriptors.rows), f.count());
  EXPECT_EQ(f.descriptors.cols, 32) << "ORB descriptors are 256 bits";
  EXPECT_EQ(f.prev_matched.size(), f.curr_matched.size());
}

TEST(OrbTracker, ASmallShiftIsMostlyMatched)
{
  OrbTracker tracker(small_options());
  const cv::Mat base = scene();
  tracker.track(base);
  const TrackedFrame f = tracker.track(shifted(base, 3.0, 2.0));

  // A three-pixel shift of the same scene is about what one frame of a slow
  // hand-held pan looks like. Most features must survive it, or the descriptor
  // threshold is set so tight that the odometer would starve.
  EXPECT_GT(f.matched_fraction(), 0.5)
    << "matched " << f.recent_match_count << " of " << f.count();
  EXPECT_GT(f.prev_matched.size(), 20u);
}

TEST(OrbTracker, MatchedPixelPairsMoveTheWayTheSceneMoved)
{
  // This is the property the rotation estimator actually depends on: that a
  // pair really is the same physical point, so the displacement between them
  // is motion rather than noise. Verified in the one case where the answer is
  // known — a pure translation of the whole image.
  OrbTracker tracker(small_options());
  const cv::Mat base = scene();
  tracker.track(base);
  const double dx = 5.0;
  const TrackedFrame f = tracker.track(shifted(base, dx, 0.0));

  ASSERT_GT(f.prev_matched.size(), 20u);
  std::vector<double> deltas;
  deltas.reserve(f.prev_matched.size());
  for (std::size_t i = 0; i < f.prev_matched.size(); ++i) {
    deltas.push_back(f.curr_matched[i].x - f.prev_matched[i].x);
  }
  std::sort(deltas.begin(), deltas.end());
  const double median = deltas[deltas.size() / 2];
  EXPECT_NEAR(median, dx, 1.5);
}

TEST(OrbTracker, TrackIdsFollowThePointNotTheIndex)
{
  OrbTracker tracker(small_options());
  const cv::Mat base = scene();
  const TrackedFrame first = tracker.track(base);
  const TrackedFrame second = tracker.track(shifted(base, 2.0, 1.0));

  ASSERT_GT(first.count(), 0u);
  std::size_t inherited = 0;
  for (std::size_t i = 0; i < second.count(); ++i) {
    if (second.match_index[i] >= 0) {
      // A matched feature IS the previous one, so it must carry that identity
      // forward — inheriting by index instead would hand the id to whichever
      // corner happened to sort into that slot this frame.
      EXPECT_EQ(
        second.track_id[i],
        first.track_id[static_cast<std::size_t>(second.match_index[i])]);
      ++inherited;
    } else {
      // A new track, and its id must be one nothing else has used.
      EXPECT_GE(second.track_id[i], static_cast<int32_t>(first.count()));
    }
  }
  EXPECT_GT(inherited, 20u);
}

TEST(OrbTracker, TrackIdsAreNeverReused)
{
  OrbTracker tracker(small_options());
  const cv::Mat base = scene();
  const TrackedFrame first = tracker.track(base);
  const int32_t highest =
    *std::max_element(first.track_id.begin(), first.track_id.end());

  // Even across a reset: a consumer holding an old id must never find it
  // silently pointing at a different physical point.
  tracker.reset();
  const TrackedFrame after = tracker.track(scene(640, 480, 99));
  for (const auto id : after.track_id) {
    EXPECT_GT(id, highest);
  }
}

TEST(OrbTracker, ResetForgetsTheHistory)
{
  OrbTracker tracker(small_options());
  const cv::Mat base = scene();
  tracker.track(base);
  tracker.reset();
  const TrackedFrame f = tracker.track(shifted(base, 2.0, 0.0));

  // The next frame is a first frame again: nothing to match into.
  EXPECT_EQ(f.recent_match_count, 0u);
  EXPECT_TRUE(f.prev_matched.empty());
}

TEST(OrbTracker, TheWindowForgivesAFrameOfChurnThatConsecutiveOnlyWouldNot)
{
  // The reason `match_window` exists. Frame 3 is a scene the tracker has never
  // seen — the moment of a blur, or a hand across the lens — and frame 4
  // returns to the original view. Consecutive-only matching sees frame 4 as
  // entirely new; the pooled window recognises it, because the original frames
  // are still in the pool.
  const cv::Mat base = scene();
  const cv::Mat interruption = scene(640, 480, 77);

  OrbTracker pooled(small_options());
  pooled.track(base);
  pooled.track(shifted(base, 2.0, 0.0));
  pooled.track(interruption);
  const TrackedFrame recovered = pooled.track(shifted(base, 3.0, 1.0));

  OrbTracker consecutive([] {
      auto o = small_options();
      o.window = 1;
      return o;
    }());
  consecutive.track(base);
  consecutive.track(shifted(base, 2.0, 0.0));
  consecutive.track(interruption);
  const TrackedFrame lost = consecutive.track(shifted(base, 3.0, 1.0));

  EXPECT_GT(recovered.matched_fraction(), lost.matched_fraction() + 0.2)
    << "pooled " << recovered.matched_fraction() << " vs consecutive "
    << lost.matched_fraction();
}

TEST(OrbTracker, StrictPairsIgnoreTheWindowEntirely)
{
  // The trap this file exists to prevent: using the pooled result for
  // odometry. After an interrupting frame the strict match must be thin —
  // because the PREVIOUS frame really was a different scene — even though the
  // pooled match recovers. A "match" six frames back carries six frames of
  // motion, and feeding that to a one-frame estimator is a confident wrong
  // answer.
  const cv::Mat base = scene();
  OrbTracker tracker(small_options());
  tracker.track(base);
  tracker.track(scene(640, 480, 77));
  const TrackedFrame f = tracker.track(shifted(base, 2.0, 0.0));

  EXPECT_GT(f.matched_fraction(), 0.4) << "the pooled window should recognise the scene";
  EXPECT_LT(f.prev_matched.size(), f.recent_match_count)
    << "strict pairs must not inherit the window's memory";
}

TEST(OrbTracker, AFeaturelessFrameYieldsNothingRatherThanGarbage)
{
  OrbTracker tracker(small_options());
  const cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC1);

  const TrackedFrame f = tracker.track(blank);
  EXPECT_EQ(f.count(), 0u);
  EXPECT_TRUE(f.descriptors.empty());

  // And the tracker must survive it: a blank frame in the middle of a session
  // (a hand over the lens) cannot be allowed to poison the state.
  const TrackedFrame after = tracker.track(scene());
  EXPECT_GT(after.count(), 50u);
}
