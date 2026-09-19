// The keyframe store's admission rule.
//
// **Nothing consumes this store yet, which is exactly why it needs a test.** A
// threshold that never fires and one that fires on every frame both produce a
// pipeline that runs perfectly: the first leaves one keyframe for the session, the
// second leaves eight thousand, and the only symptom of either is a memory figure
// nobody is looking at. There is no downstream stage to notice, no topic to go
// quiet, and no gate that could tell the difference — which is the definition of
// what belongs in this directory.

#include <gtest/gtest.h>

#include <cmath>

#include "opencv2/core.hpp"
#include "pimesh_perception/keyframe_store.hpp"

using pimesh_perception::angle_between;
using pimesh_perception::Keyframe;
using pimesh_perception::KeyframeStore;

namespace
{

cv::Matx33d rotation_about(const cv::Vec3d & axis, double angle)
{
  const cv::Vec3d u = cv::normalize(axis);
  const double c = std::cos(angle);
  const double s = std::sin(angle);
  const cv::Matx33d cross(
    0.0, -u[2], u[1],
    u[2], 0.0, -u[0],
    -u[1], u[0], 0.0);
  return cv::Matx33d::eye() * c + cross * s + cv::Matx33d(u * u.t()) * (1.0 - c);
}

cv::Affine3d at(const cv::Vec3d & position, double yaw_deg = 0.0)
{
  return cv::Affine3d(rotation_about({0.0, 0.0, 1.0}, yaw_deg * CV_PI / 180.0), position);
}

/// A keyframe the size of a real one: 500 ORB features, roughly half of which had
/// a usable depth reading. The numbers come from bags/desk1 and the point of
/// building it here is bytes(), below.
Keyframe realistic()
{
  Keyframe frame;
  frame.descriptors = cv::Mat::zeros(500, 32, CV_8U);
  frame.track_ids.assign(500, 0);
  frame.bearings.assign(500, cv::Vec3d(0.0, 0.0, 1.0));
  frame.landmarks.assign(250, cv::Vec3d(0.0, 0.0, 2.0));
  frame.landmark_row.assign(250, 0);
  return frame;
}

KeyframeStore::Config config()
{
  // The node's defaults: 18 degrees or 0.3 m.
  return KeyframeStore::Config{18.0 * CV_PI / 180.0, 0.3, 500};
}

}  // namespace

TEST(AngleBetween, IsZeroForIdenticalOrientationsRatherThanNaN)
{
  // acos(1.0000000002) is NaN, and a NaN compares false against every threshold —
  // so a store whose angle test returned NaN would simply never admit a keyframe
  // on view change, silently, for the life of the session.
  const cv::Matx33d r = rotation_about({0.3, -0.5, 0.8}, 1.1);
  EXPECT_TRUE(std::isfinite(angle_between(r, r)));
  EXPECT_NEAR(angle_between(r, r), 0.0, 1e-7);
  EXPECT_TRUE(std::isfinite(angle_between(cv::Matx33d::eye(), cv::Matx33d::eye())));
}

TEST(AngleBetween, IsTheAngleAndNotSomethingProportionalToIt)
{
  for (double degrees : {1.0, 17.0, 45.0, 90.0, 179.0}) {
    const double radians = degrees * CV_PI / 180.0;
    const cv::Matx33d a = rotation_about({0.2, 0.9, -0.4}, 0.7);
    const cv::Matx33d b = a * rotation_about({0.0, 1.0, 0.0}, radians);
    EXPECT_NEAR(angle_between(a, b), radians, 1e-7) << degrees;
  }
}

TEST(KeyframeStore, TheFirstOneIsAlwaysAdmitted)
{
  KeyframeStore store(config());
  EXPECT_TRUE(store.empty());
  EXPECT_TRUE(store.would_insert(at({0.0, 0.0, 0.0})));
  EXPECT_TRUE(store.maybe_insert(Keyframe{}));
  EXPECT_EQ(store.size(), 1u);
}

TEST(KeyframeStore, RefusesAPoseThatHasBarelyMoved)
{
  KeyframeStore store(config());
  ASSERT_TRUE(store.maybe_insert(Keyframe{}));

  Keyframe nearby;
  nearby.odom_from_camera = at({0.05, 0.0, 0.0}, 5.0);
  EXPECT_FALSE(store.would_insert(nearby.odom_from_camera));
  EXPECT_FALSE(store.maybe_insert(nearby));
  EXPECT_EQ(store.size(), 1u);
}

TEST(KeyframeStore, EitherThresholdAloneIsEnough)
{
  // The two conditions are separate rather than combined, and each has a case
  // the other cannot see: a camera panning on the spot travels no distance and
  // sees an entirely different room; a camera sliding along a wall turns through
  // no angle and does the same.
  {
    KeyframeStore store(config());
    ASSERT_TRUE(store.maybe_insert(Keyframe{}));
    Keyframe turned;
    turned.odom_from_camera = at({0.0, 0.0, 0.0}, 20.0);   // no translation at all
    EXPECT_TRUE(store.maybe_insert(turned));
  }
  {
    KeyframeStore store(config());
    ASSERT_TRUE(store.maybe_insert(Keyframe{}));
    Keyframe slid;
    slid.odom_from_camera = at({0.0, 0.4, 0.0}, 0.0);      // no rotation at all
    EXPECT_TRUE(store.maybe_insert(slid));
  }
}

TEST(KeyframeStore, MeasuresFromTheNewestKeyframeAndNotTheFirst)
{
  // Three 10-degree steps. Against the origin the third is 30 degrees away and
  // would be admitted; against its predecessor it is 10 and must not be. Getting
  // this wrong gives a store that fills up fast and then stops growing.
  KeyframeStore store(config());
  Keyframe first;
  first.odom_from_camera = at({0.0, 0.0, 0.0}, 0.0);
  ASSERT_TRUE(store.maybe_insert(first));

  Keyframe second;
  second.odom_from_camera = at({0.0, 0.0, 0.0}, 20.0);
  ASSERT_TRUE(store.maybe_insert(second));

  Keyframe third;
  third.odom_from_camera = at({0.0, 0.0, 0.0}, 30.0);
  EXPECT_FALSE(store.maybe_insert(third)) << "10 degrees past the newest, not 30 past the first";
  EXPECT_EQ(store.size(), 2u);
}

TEST(KeyframeStore, HasACeilingAndSaysWhenItHitsIt)
{
  KeyframeStore store(KeyframeStore::Config{18.0 * CV_PI / 180.0, 0.3, 3});
  for (int i = 0; i < 6; ++i) {
    Keyframe frame;
    frame.stamp_ns = i;
    frame.odom_from_camera = at({0.5 * i, 0.0, 0.0});
    ASSERT_TRUE(store.maybe_insert(frame)) << i;
  }
  EXPECT_EQ(store.size(), 3u);
  EXPECT_EQ(store.dropped(), 3u);
  EXPECT_EQ(store.latest().stamp_ns, 5);
  EXPECT_EQ(store.frames().front().stamp_ns, 3) << "the oldest is what goes";
}

TEST(KeyframeStore, ClearForgetsTheDropCountToo)
{
  KeyframeStore store(KeyframeStore::Config{18.0 * CV_PI / 180.0, 0.3, 1});
  for (int i = 0; i < 3; ++i) {
    Keyframe frame;
    frame.odom_from_camera = at({0.5 * i, 0.0, 0.0});
    store.maybe_insert(frame);
  }
  ASSERT_GT(store.dropped(), 0u);
  store.clear();
  EXPECT_TRUE(store.empty());
  EXPECT_EQ(store.dropped(), 0u);
  EXPECT_EQ(store.bytes(), 0u);
  EXPECT_TRUE(store.would_insert(at({0.0, 0.0, 0.0}))) << "an empty store admits anything";
}

TEST(KeyframeStore, OneKeyframeIsTensOfKilobytesAndNotHundreds)
{
  // **P7 budgeted ~16 kB each, and that is the descriptors alone.** Measured here:
  // 500 32-byte descriptors are 16 kB, and the three geometric arrays beside them
  // — a bearing ray for every feature, a landmark for the half that had a usable
  // depth reading, and the row index tying the two together — bring one keyframe
  // to ~37 kB. At the 500-keyframe ceiling that is ~18 MB, which is why the
  // difference is recorded rather than designed away.
  //
  // The assertion is a band, not a number: it is here to catch an order of
  // magnitude — a descriptor matrix accidentally kept as CV_32F, or a store
  // holding whole frames — and not to pin a struct layout.
  KeyframeStore store(config());
  ASSERT_TRUE(store.maybe_insert(realistic()));
  const double kb = static_cast<double>(store.bytes()) / 1024.0;
  EXPECT_GT(kb, 16.0) << "smaller than the descriptors alone means something is not being kept";
  EXPECT_LT(kb, 80.0) << "one keyframe is landmarks and descriptors, not a frame";

  Keyframe second = realistic();
  second.odom_from_camera = at({1.0, 0.0, 0.0});
  ASSERT_TRUE(store.maybe_insert(second));
  EXPECT_NEAR(static_cast<double>(store.bytes()) / 1024.0, 2.0 * kb, 0.01);
}
