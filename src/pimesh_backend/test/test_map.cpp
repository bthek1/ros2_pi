// The map: points observed by many keyframes, the covisibility graph, and culling.
//
// **Everything wrong here makes the numbers look better.** A map that never culls
// has more points and more keyframes. A point that accepts two observations from
// one keyframe reaches "three observations" sooner. A covisibility query that only
// ever returns the reference keyframe gives a local map that tracks exactly as well
// as P7 did, because it *is* P7. So each test here asserts the thing a count would
// never show — that a cull fired, that a claim was refused, that a local map had
// more than one keyframe in it.

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <set>
#include <vector>

#include "pimesh_backend/local_ba.hpp"
#include "pimesh_backend/map.hpp"
#include "pimesh_backend/triangulation.hpp"

using pimesh_backend::InsertReport;
using pimesh_backend::KeyframeId;
using pimesh_backend::KeyframeInput;
using pimesh_backend::kNoPoint;
using pimesh_backend::Map;
using pimesh_backend::PointId;

namespace
{

cv::Matx33d camera_k()
{
  return cv::Matx33d(517.3, 0.0, 318.6, 0.0, 516.5, 255.3, 0.0, 0.0, 1.0);
}

/// A fixed room: points in a box two to four metres in front of the origin, each
/// with its own random descriptor.
struct World
{
  std::vector<cv::Vec3d> points;
  std::vector<cv::Mat> descriptors;

  explicit World(std::size_t n, unsigned seed = 11)
  {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> x(-1.0, 1.0);
    std::uniform_real_distribution<double> y(-0.6, 0.6);
    std::uniform_real_distribution<double> z(2.0, 4.0);
    std::uniform_int_distribution<int> byte(0, 255);
    for (std::size_t i = 0; i < n; ++i) {
      points.emplace_back(x(gen), y(gen), z(gen));
      cv::Mat d(1, 32, CV_8U);
      for (int b = 0; b < 32; ++b) {d.at<std::uint8_t>(0, b) = static_cast<std::uint8_t>(byte(gen));}
      descriptors.push_back(d);
    }
  }
};

cv::Affine3d camera_at(const cv::Vec3d & centre)
{
  return cv::Affine3d(cv::Matx33d::eye(), centre);
}

/// What a camera at `pose` would hand the map: every world point in view, with the
/// point's index as its track id and the network reading `scale` times the truth.
KeyframeInput view_from(
  const World & world, const cv::Affine3d & pose, double scale = 1.0,
  const std::set<std::size_t> & only = {})
{
  KeyframeInput in;
  in.map_from_camera = pose;
  in.k = camera_k();
  std::vector<cv::Mat> rows;
  for (std::size_t i = 0; i < world.points.size(); ++i) {
    if (!only.empty() && only.count(i) == 0) {continue;}
    const cv::Vec3d c = pose.inv() * world.points[i];
    cv::Point2d pixel;
    if (!pimesh_backend::project(pose, in.k, world.points[i], pixel)) {continue;}
    if (pixel.x < 0 || pixel.x >= 640 || pixel.y < 0 || pixel.y >= 480) {continue;}
    in.pixels.emplace_back(static_cast<float>(pixel.x), static_cast<float>(pixel.y));
    in.track_ids.push_back(static_cast<std::int32_t>(i));
    rows.push_back(world.descriptors[i]);
    in.network_points.push_back(c * scale);
    in.has_depth.push_back(1);
    in.matched.push_back(kNoPoint);
  }
  cv::vconcat(rows, in.descriptors);
  return in;
}

/// Fill `matched` the way tracking does in its first stage: by track id.
void match_by_track(const Map & map, KeyframeInput & in)
{
  const auto views = map.lookup_tracks(in.track_ids);
  for (std::size_t i = 0; i < views.size(); ++i) {in.matched[i] = views[i].id;}
}

KeyframeId insert_tracked(Map & map, const World & world, const cv::Affine3d & pose,
  double scale = 1.0, const std::set<std::size_t> & only = {})
{
  KeyframeInput in = view_from(world, pose, scale, only);
  match_by_track(map, in);
  const InsertReport r = map.insert(in);
  EXPECT_TRUE(r.ok);
  return r.id;
}

}  // namespace

// --- Insertion and observations ------------------------------------------------

TEST(MapInsert, TheFirstKeyframeCreatesAPointPerFeatureWithDepth)
{
  const World world(200);
  Map map(Map::Config{});
  KeyframeInput in = view_from(world, camera_at({0, 0, 0}));
  // A third of the features have no depth reading, and make no point.
  for (std::size_t i = 0; i < in.has_depth.size(); i += 3) {in.has_depth[i] = 0;}
  const InsertReport r = map.insert(in);
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.id, 0);
  EXPECT_EQ(r.associated, 0u);
  EXPECT_EQ(r.created, in.pixels.size() - (in.pixels.size() + 2) / 3);
  EXPECT_EQ(map.stats().points, r.created);
}

TEST(MapInsert, ASecondViewObservesTheSamePointsRatherThanMakingNewOnes)
{
  const World world(200);
  Map map(Map::Config{});
  const KeyframeId a = insert_tracked(map, world, camera_at({0, 0, 0}));
  const std::size_t points = map.stats().points;

  KeyframeInput in = view_from(world, camera_at({0.1, 0, 0}));
  match_by_track(map, in);
  const InsertReport r = map.insert(in);
  ASSERT_TRUE(r.ok);
  EXPECT_GT(r.associated, points * 8 / 10);
  EXPECT_EQ(map.stats().points, points + r.created);

  // Covisibility is symmetric and counts the shared points.
  const auto local = map.local_map(r.id);
  ASSERT_EQ(local.keyframes.size(), 2u);
  EXPECT_EQ(local.keyframes[0], r.id);
  EXPECT_EQ(local.keyframes[1], a);
}

TEST(MapInsert, AKeyframeObservesAPointOnceHoweverManyFeaturesClaimIt)
{
  // **The map's version of "a track id is never claimed twice in one frame".** Two
  // features of one keyframe both claiming point 0 — a repeated texture, or a
  // matcher that let one landmark win twice. If the second were accepted the point
  // would reach three observations in two keyframes, and "fraction with >= 3
  // observations" would be counting a matcher bug as a well-observed landmark.
  const World world(50);
  Map map(Map::Config{});
  insert_tracked(map, world, camera_at({0, 0, 0}));
  KeyframeInput in = view_from(world, camera_at({0, 0, 0}));
  match_by_track(map, in);
  ASSERT_GE(in.matched.size(), 2u);
  const PointId claimed = in.matched[0];
  in.matched[1] = claimed;
  in.pixels[1] = in.pixels[0];
  const InsertReport r = map.insert(in);
  EXPECT_EQ(r.refused_duplicate, 1u);
  // And the refused feature did not become a duplicate point of its own.
  EXPECT_EQ(r.created, 0u);
  // Point 0: one observation from each keyframe, never two from one.
  EXPECT_EQ(map.stats().points_3plus, 0u);
}

TEST(MapInsert, ReadmittingATrackIdOnADifferentFeatureDoesNotObserveTheOldPoint)
{
  // The tracker hands a track id to a corner 60 px from where the point that id
  // belonged to should be — the same id, a different thing in the world. The map
  // checks the claim against the geometry and refuses it; the feature becomes its
  // own point and the track now belongs to that.
  const World world(50);
  Map map(Map::Config{});
  insert_tracked(map, world, camera_at({0, 0, 0}));
  KeyframeInput in = view_from(world, camera_at({0, 0, 0}));
  match_by_track(map, in);
  const PointId old_point = in.matched[0];
  in.pixels[0].x += 60.0F;
  const InsertReport r = map.insert(in);
  EXPECT_EQ(r.refused_reprojection, 1u);
  EXPECT_EQ(r.created, 1u);
  const auto now = map.lookup_tracks({in.track_ids[0]});
  EXPECT_NE(now[0].id, old_point);
  EXPECT_NE(now[0].id, kNoPoint);
}

TEST(MapInsert, RaggedInputIsRefusedWhole)
{
  const World world(50);
  Map map(Map::Config{});
  KeyframeInput in = view_from(world, camera_at({0, 0, 0}));
  in.has_depth.pop_back();
  EXPECT_FALSE(map.insert(in).ok);
  EXPECT_EQ(map.stats().keyframes, 0u);
  EXPECT_EQ(map.stats().points, 0u);
}

// --- Culling ------------------------------------------------------------------

TEST(MapCull, APointNoLaterKeyframeSawIsCulled)
{
  // Points 0-24 are seen only by the first keyframe; 25-49 by every keyframe.
  // Two keyframes later the first group has had its chance and goes.
  const World world(50);
  std::set<std::size_t> all;
  std::set<std::size_t> later;
  for (std::size_t i = 0; i < 50; ++i) {
    all.insert(i);
    if (i >= 25) {later.insert(i);}
  }
  Map map(Map::Config{});
  insert_tracked(map, world, camera_at({0, 0, 0}), 1.0, all);
  const std::size_t before = map.stats().points;
  insert_tracked(map, world, camera_at({0.05, 0, 0}), 1.0, later);
  const KeyframeId third = insert_tracked(map, world, camera_at({0.1, 0, 0}), 1.0, later);
  const auto report = map.cull(third);
  EXPECT_GT(report.points, 0u);
  EXPECT_LT(map.stats().points, before);
  // Every survivor was seen by all three keyframes.
  EXPECT_EQ(map.stats().points, map.stats().points_3plus);
}

TEST(MapCull, APointTrackingKeepsMissingIsCulled)
{
  const World world(20);
  Map map(Map::Config{});
  insert_tracked(map, world, camera_at({0, 0, 0}));
  const auto points = map.lookup_tracks({0, 1});
  // Point for track 0 predicted visible nine times and never found; track 1 found
  // every time.
  for (int i = 0; i < 9; ++i) {map.record_tracking({points[0].id, points[1].id}, {points[1].id});}
  EXPECT_EQ(map.cull(map.newest()).points, 1u);
  EXPECT_EQ(map.lookup_tracks({0})[0].id, kNoPoint);
  EXPECT_NE(map.lookup_tracks({1})[0].id, kNoPoint);
}

TEST(MapCull, ACullThatFindsNothingStillSaysItLooked)
{
  // Two keyframes that share every point, both young: nothing is culled, and the
  // report must still say the cull *ran* — points judged, keyframes judged — or a
  // map that never culls and one that culled nothing read the same.
  const World world(60);
  Map map(Map::Config{});
  insert_tracked(map, world, camera_at({0, 0, 0}));
  const auto report = map.cull(insert_tracked(map, world, camera_at({0.05, 0, 0})));
  EXPECT_EQ(report.points, 0u);
  EXPECT_EQ(report.keyframes, 0u);
  EXPECT_GT(report.points_judged, 0u);
  EXPECT_EQ(report.keyframes_judged, 0u);  // the first keyframe is never a candidate
  EXPECT_EQ(map.stats().judged_points, report.points_judged);
}

TEST(MapCull, AnEstablishedPointIsNotJudgedAgain)
{
  // Seen by two keyframes, it survives its probation. Many keyframes later nothing
  // sees it any more — the camera turned away — and it must still be there: a map
  // that forgets what is behind the camera is not a map.
  const World world(60);
  std::set<std::size_t> early;
  std::set<std::size_t> late;
  for (std::size_t i = 0; i < 60; ++i) {(i < 30 ? early : late).insert(i);}
  Map map(Map::Config{});
  insert_tracked(map, world, camera_at({0, 0, 0}));
  map.cull(insert_tracked(map, world, camera_at({0.05, 0, 0})));
  for (int k = 0; k < 6; ++k) {
    map.cull(insert_tracked(map, world, camera_at({0.1 + 0.02 * k, 0, 0}), 1.0, late));
  }
  const auto survivors = map.lookup_tracks({0, 1, 2});
  for (const auto & p : survivors) {EXPECT_NE(p.id, kNoPoint);}
}

TEST(MapCull, ARedundantKeyframeIsCulledAndTheFirstNeverIs)
{
  // Six keyframes a centimetre apart all seeing the same room: every point is
  // observed by five other keyframes, so the middle ones add nothing.
  const World world(80);
  Map map(Map::Config{});
  std::vector<KeyframeId> ids;
  for (int k = 0; k < 6; ++k) {ids.push_back(insert_tracked(map, world, camera_at({0.01 * k, 0, 0})));}
  const auto report = map.cull(ids.back());
  EXPECT_GT(report.keyframes, 0u);
  EXPECT_EQ(map.stats().culled_keyframes, report.keyframes);
  EXPECT_GE(report.keyframes_judged, report.keyframes);
  // The first and the newest survive whatever the redundancy.
  const auto local = map.local_map(ids.back());
  EXPECT_EQ(local.keyframes.front(), ids.back());
  EXPECT_NE(std::find(local.keyframes.begin(), local.keyframes.end(), ids.front()),
    local.keyframes.end());
  // And culling keyframes did not cull the points they shared with survivors.
  EXPECT_EQ(map.stats().points, world.points.size());
}

TEST(MapCull, CullingAKeyframeRemovesItsCovisibilityEdges)
{
  const World world(80);
  Map map(Map::Config{});
  std::vector<KeyframeId> ids;
  for (int k = 0; k < 6; ++k) {ids.push_back(insert_tracked(map, world, camera_at({0.01 * k, 0, 0})));}
  map.cull(ids.back());
  const std::size_t live = map.stats().keyframes;
  // The newest keyframe's local map names only keyframes that still exist.
  const auto local = map.local_map(ids.back());
  EXPECT_EQ(local.keyframes.size(), live);
}

// --- The local map --------------------------------------------------------------

TEST(LocalMap, IsTheReferenceAndItsCovisibleKeyframesNotTheReferenceAlone)
{
  // **The plan's third false green.** A covisibility query that silently returns one
  // keyframe makes "tracking against the local map" P7's tracker with extra steps,
  // and every number the gate prints would be P7's number with nothing saying so.
  const World world(300);
  Map map(Map::Config{});
  KeyframeId last = 0;
  for (int k = 0; k < 5; ++k) {last = insert_tracked(map, world, camera_at({0.15 * k, 0, 0}));}
  const auto local = map.local_map(last);
  EXPECT_GT(local.keyframes.size(), 1u);
  // Points are the union over those keyframes, each once.
  std::set<PointId> ids;
  for (const auto & p : local.points) {EXPECT_TRUE(ids.insert(p.id).second);}
}

TEST(LocalMap, LeavesOutKeyframesThatShareTooLittle)
{
  // Two keyframes looking at disjoint halves of the room share nothing, and a third
  // shares only five points with the first — under the 15-point floor.
  const World world(120);
  std::set<std::size_t> left;
  std::set<std::size_t> right;
  std::set<std::size_t> sliver;
  for (std::size_t i = 0; i < 120; ++i) {
    (i < 60 ? left : right).insert(i);
    if (i < 5 || i >= 60) {sliver.insert(i);}
  }
  Map map(Map::Config{});
  const KeyframeId a = insert_tracked(map, world, camera_at({0, 0, 0}), 1.0, left);
  insert_tracked(map, world, camera_at({0.02, 0, 0}), 1.0, right);
  insert_tracked(map, world, camera_at({0.04, 0, 0}), 1.0, sliver);
  const auto local = map.local_map(a);
  EXPECT_EQ(local.keyframes.size(), 1u);
}

TEST(LocalMap, FallsBackToTheNewestKeyframeWhenTheReferenceIsGone)
{
  const World world(40);
  Map map(Map::Config{});
  insert_tracked(map, world, camera_at({0, 0, 0}));
  const KeyframeId b = insert_tracked(map, world, camera_at({0.1, 0, 0}));
  const auto local = map.local_map(9999);
  ASSERT_FALSE(local.keyframes.empty());
  EXPECT_EQ(local.keyframes.front(), b);
}

// --- Triangulation inside the map ----------------------------------------------

TEST(MapTriangulate, PutsPointsWhereTheGeometryIsWhenTheNetworkIsWrong)
{
  // **The first independent opinion about the depth network, as a test.** The
  // network reads every point 5% too far; the camera poses are exact. After the
  // second keyframe the points are triangulated from the two views, land where
  // they really are, and report a depth ratio of 1/1.05 — which is the number the
  // gate prints, and here it has a known right answer.
  //
  // The association threshold is loosened for this one, and the reason is worth
  // having written down: a network wrong by 5% against *exact* poses puts a point
  // 30 cm of baseline away up to ~4 px from where the second camera sees it, so the
  // map's own association check would refuse the observations the triangulation
  // needs. On the real pipeline the poses were solved against the network's
  // landmarks and share its scale, so that inconsistency cannot arise there.
  const World world(150);
  Map::Config config;
  config.associate_px = 8.0;
  config.use_triangulation = true;
  config.align_scale = false;
  Map map(config);
  insert_tracked(map, world, camera_at({0, 0, 0}), 1.05);
  const KeyframeId b = insert_tracked(map, world, camera_at({0.3, 0, 0}), 1.05);
  const auto report = map.triangulate(b);
  ASSERT_GT(report.triangulated, 50u);
  for (double ratio : report.depth_ratio) {EXPECT_NEAR(ratio, 1.0 / 1.05, 0.005);}
  for (double error : report.error_px) {EXPECT_LT(error, 0.05);}
  EXPECT_EQ(map.stats().triangulated, report.triangulated);

  const auto tracked = map.lookup_tracks({0, 1, 2, 3});
  for (std::size_t i = 0; i < tracked.size(); ++i) {
    if (tracked[i].id == kNoPoint) {continue;}
    EXPECT_LT(cv::norm(tracked[i].position - world.points[i]), 0.01);
  }
}

TEST(MapTriangulate, APanKeepsTheNetworkReading)
{
  // Two keyframes from the same place, rotated: no baseline, no parallax, nothing
  // to triangulate from. Every point keeps its network position, and the refusal is
  // counted as parallax rather than as a rejection.
  const World world(150);
  Map map(Map::Config{});
  insert_tracked(map, world, camera_at({0, 0, 0}), 1.2);
  const double a = 5.0 * CV_PI / 180.0;
  const cv::Affine3d turned(
    cv::Matx33d(std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a)),
    cv::Vec3d(0, 0, 0));
  const KeyframeId b = insert_tracked(map, world, turned, 1.2);
  const auto report = map.triangulate(b);
  EXPECT_GT(report.attempted, 50u);
  EXPECT_EQ(report.triangulated, 0u);
  EXPECT_EQ(report.low_parallax, report.attempted);
}

// --- Where a point is -------------------------------------------------------------

TEST(MapPosition, APointFollowsItsFreshestReadingNotItsFirst)
{
  // **The first version kept a point where its first keyframe put it**, and tracking
  // against first readings measured 0.47-0.54 m of ATE on fr1/desk against
  // 0.25-0.35 m for P7's tracker, which re-reads every landmark at every keyframe.
  // So: the first keyframe reads the room 8% near, the second 8% far, and a point
  // both saw sits at the second's reading — exactly where P7's reference landmark
  // would be. Alignment is off here so the readings are the network's own.
  const World world(80);
  Map::Config config;
  config.align_scale = false;
  config.associate_px = 50.0;
  Map map(config);
  insert_tracked(map, world, camera_at({0, 0, 0}), 0.92);
  insert_tracked(map, world, camera_at({0.02, 0, 0}), 1.08);
  const auto p = map.lookup_tracks({5});
  const cv::Vec3d second_centre(0.02, 0, 0);
  const cv::Vec3d expected = second_centre + 1.08 * (world.points[5] - second_centre);
  EXPECT_LT(cv::norm(p[0].position - expected), 1e-9);
}

TEST(MapPosition, AWindowAveragesTheNewestReadings)
{
  const World world(80);
  Map::Config config;
  config.align_scale = false;
  config.associate_px = 50.0;
  config.reading_window = 2;
  Map map(config);
  insert_tracked(map, world, camera_at({0, 0, 0}), 0.92);
  insert_tracked(map, world, camera_at({0, 0, 0}), 1.08);
  const auto p = map.lookup_tracks({5});
  EXPECT_LT(cv::norm(p[0].position - world.points[5]), 1e-9);
}

TEST(MapPosition, LosingTheNewestObservationFallsBackToThePreviousReading)
{
  // The keyframes that gave a point its position are culled; the point must not
  // keep a reading from a keyframe that no longer exists. The last three keyframes
  // read 10% far and the first two exactly; with the redundancy bar at one other
  // observer, a cull run from the second keyframe takes the last three.
  const World world(80);
  Map::Config config;
  config.align_scale = false;
  config.associate_px = 50.0;
  config.redundant_observers = 1;
  Map map(config);
  std::vector<KeyframeId> ids;
  for (double scale : {1.0, 1.0, 1.10, 1.10, 1.10}) {
    ids.push_back(insert_tracked(map, world, camera_at({0, 0, 0}), scale));
  }
  EXPECT_LT(cv::norm(map.lookup_tracks({5})[0].position - 1.10 * world.points[5]), 1e-9);
  ASSERT_EQ(map.cull(ids[1]).keyframes, 3u);
  EXPECT_LT(cv::norm(map.lookup_tracks({5})[0].position - world.points[5]), 1e-9);
}

// --- Scale alignment -------------------------------------------------------------

namespace
{

/// Two keyframes on the same spot: the first reads the room at the network's scale
/// 1.0, the second at `second_scale`. Returns the second keyframe's report and the
/// ratio of a point *it created* to where that point really is.
std::pair<pimesh_backend::InsertReport, double> breathe(
  double second_scale, double gain, std::size_t only_new_from = 100)
{
  const World world(200);
  Map::Config config;
  config.align_gain = gain;
  // Wide enough that a 10% scale error still associates: this test is about what
  // happens to the points the second keyframe *creates*.
  config.associate_px = 50.0;
  Map map(config);
  std::set<std::size_t> first;
  for (std::size_t i = 0; i < only_new_from; ++i) {first.insert(i);}
  insert_tracked(map, world, camera_at({0, 0, 0}), 1.0, first);
  KeyframeInput in = view_from(world, camera_at({0.05, 0, 0}), second_scale);
  match_by_track(map, in);
  const auto report = map.insert(in);
  // A point only the second keyframe saw.
  const auto created = map.lookup_tracks({static_cast<std::int32_t>(only_new_from)});
  const cv::Vec3d centre(0.05, 0, 0);
  const double ratio = cv::norm(created[0].position - centre) /
    cv::norm(world.points[only_new_from] - centre);
  return {report, ratio};
}

}  // namespace

TEST(MapAlign, FullGainPutsANewDepthMapExactlyOnTheMapsScale)
{
  // The second depth map reads 10% far. Fully corrected, the points it creates land
  // at the true distance — the scale of the map the first keyframe laid down.
  const auto [report, ratio] = breathe(1.10, 1.0);
  EXPECT_GE(report.scale_pairs, 20u);
  EXPECT_NEAR(report.scale_median, 1.0 / 1.10, 0.01);
  EXPECT_NEAR(ratio, 1.0, 0.01);
}

TEST(MapAlign, TheGainCorrectsPartOfTheWayAndZeroCorrectsNothing)
{
  // **Why part of the way:** full correction makes each keyframe inherit the map's
  // scale, and inheritance of a measured quantity is a random walk — one fr1/desk
  // run finished at a fitted scale of 0.699 that way. At gain 0.5 the correction is
  // the square root of the measured ratio, and at 0 there is none at all.
  const auto half = breathe(1.10, 0.5);
  EXPECT_NEAR(half.second, 1.10 * std::sqrt(1.0 / 1.10), 0.01);
  const auto none = breathe(1.10, 0.0);
  EXPECT_NEAR(none.first.scale, 1.0, 1e-12);
  EXPECT_NEAR(none.second, 1.10, 0.01);
  // The measurement is reported whatever the gain, so the breathing is visible even
  // with the correction switched off.
  EXPECT_NEAR(none.first.scale_median, 1.0 / 1.10, 0.01);
}

TEST(MapAlign, TooFewAssociationsMeansNoCorrectionAndSaysSo)
{
  // Ten shared points, under the floor of twenty. The scale stays exactly 1 and
  // `scale_pairs` is 0 — which is how "not measured" is told apart from "measured
  // as 1", since both have the same scale.
  const auto [report, ratio] = breathe(1.10, 1.0, 10);
  (void)ratio;
  EXPECT_EQ(report.scale_pairs, 0u);
  EXPECT_EQ(report.scale, 1.0);
}

TEST(MapAlign, ACorrectionIsClampedRatherThanTrusted)
{
  // A ratio of 1.6 is a keyframe tracked badly, not a network that breathed 60%.
  const auto [report, ratio] = breathe(1.0 / 1.6, 1.0);
  (void)ratio;
  EXPECT_NEAR(report.scale, 1.15, 1e-9);
}

// --- The bundle-adjustment window -------------------------------------------------

TEST(MapBaWindow, FreesTheNewestAndItsNeighboursAndFixesTheFirst)
{
  const World world(150);
  Map map(Map::Config{});
  std::vector<KeyframeId> ids;
  for (int k = 0; k < 5; ++k) {ids.push_back(insert_tracked(map, world, camera_at({0.1 * k, 0, 0})));}
  const auto problem = map.ba_window(ids.back(), 3, 20);
  ASSERT_FALSE(problem.keyframes.empty());
  EXPECT_EQ(problem.keyframes.front().id, ids.back());
  EXPECT_FALSE(problem.keyframes.front().fixed);
  std::size_t free = 0;
  bool first_seen = false;
  for (const auto & kf : problem.keyframes) {
    free += kf.fixed ? 0 : 1;
    if (kf.id == ids.front()) {
      first_seen = true;
      // The map's origin is never moved, whichever part of the window it is in.
      EXPECT_TRUE(kf.fixed);
    }
  }
  EXPECT_LE(free, 3u);
  EXPECT_TRUE(first_seen);
  // Every observation names a keyframe and a point inside the problem, and carries
  // the depth reading as seen by *that* keyframe.
  for (const auto & o : problem.observations) {
    ASSERT_LT(o.keyframe, problem.keyframes.size());
    ASSERT_LT(o.point, problem.points.size());
    const cv::Vec3d in_camera =
      problem.keyframes[o.keyframe].map_from_camera.inv() * problem.points[o.point].position;
    EXPECT_NEAR(o.depth, in_camera[2], 1e-6);
  }
}

TEST(MapBaWindow, AWindowWithNothingOutsideItHoldsItsOldestKeyframe)
{
  // Two keyframes, both inside a window of ten: nothing is left to be fixed, and a
  // window with nothing fixed floats — BA would slide the whole map. The oldest is
  // held instead (here it is also the map's first, so it is doubly so).
  const World world(80);
  Map map(Map::Config{});
  insert_tracked(map, world, camera_at({0, 0, 0}));
  const KeyframeId b = insert_tracked(map, world, camera_at({0.1, 0, 0}));
  const auto problem = map.ba_window(b, 10, 20);
  ASSERT_EQ(problem.keyframes.size(), 2u);
  std::size_t fixed = 0;
  for (const auto & kf : problem.keyframes) {fixed += kf.fixed ? 1 : 0;}
  EXPECT_EQ(fixed, 1u);
}

TEST(MapApplyBa, MovesWhatStillExistsAndDropsOutlierObservations)
{
  const World world(80);
  Map map(Map::Config{});
  insert_tracked(map, world, camera_at({0, 0, 0}));
  const KeyframeId b = insert_tracked(map, world, camera_at({0.1, 0, 0}));
  const auto problem = map.ba_window(b, 10, 20);
  pimesh_backend::BaResult result;
  result.ran = true;
  for (const auto & kf : problem.keyframes) {result.map_from_camera.push_back(kf.map_from_camera);}
  for (const auto & p : problem.points) {result.positions.push_back(p.position + cv::Vec3d(0, 0, 0.01));}
  result.outlier.assign(problem.observations.size(), 0);
  result.outlier[0] = 1;
  const auto applied = map.apply_ba(problem, result);
  EXPECT_EQ(applied.points, problem.points.size());
  EXPECT_EQ(applied.observations_dropped, 1u);
  // A point BA has solved keeps its solved position when the next keyframe reads it:
  // the reading becomes the next solve's measurement, not a replacement.
  const auto moved = map.lookup_tracks({0});
  ASSERT_NE(moved[0].id, kNoPoint);
  const cv::Vec3d solved = moved[0].position;
  insert_tracked(map, world, camera_at({0.2, 0, 0}));
  EXPECT_LT(cv::norm(map.lookup_tracks({0})[0].position - solved), 1e-12);
}

TEST(MapApplyBa, AMismatchedResultIsIgnoredRatherThanAppliedByIndex)
{
  const World world(40);
  Map map(Map::Config{});
  insert_tracked(map, world, camera_at({0, 0, 0}));
  const KeyframeId b = insert_tracked(map, world, camera_at({0.1, 0, 0}));
  const auto problem = map.ba_window(b, 10, 20);
  pimesh_backend::BaResult result;
  result.ran = true;
  result.map_from_camera.assign(problem.keyframes.size(), cv::Affine3d::Identity());
  result.positions.assign(problem.points.size() - 1, cv::Vec3d(0, 0, 0));
  result.outlier.assign(problem.observations.size(), 0);
  const auto applied = map.apply_ba(problem, result);
  EXPECT_EQ(applied.keyframes, 0u);
  EXPECT_EQ(applied.points, 0u);
}

// --- The representative descriptor ----------------------------------------------

TEST(Descriptor, IsARealObservationClosestToTheOthersNotAnAverage)
{
  const World world(1);
  Map map(Map::Config{});
  KeyframeInput a = view_from(world, camera_at({0, 0, 0}));
  map.insert(a);
  // Second observation: one bit flipped. Third: every bit flipped — a mismatch.
  KeyframeInput b = view_from(world, camera_at({0.01, 0, 0}));
  b.descriptors.at<std::uint8_t>(0, 0) ^= 0x01;
  match_by_track(map, b);
  map.insert(b);
  KeyframeInput c = view_from(world, camera_at({0.02, 0, 0}));
  cv::bitwise_not(c.descriptors, c.descriptors);
  match_by_track(map, c);
  map.insert(c);
  const auto p = map.lookup_tracks({0});
  ASSERT_NE(p[0].id, kNoPoint);
  EXPECT_LE(pimesh_backend::hamming(p[0].descriptor, world.descriptors[0]), 1);
}

TEST(Hamming, CountsBits)
{
  cv::Mat a = cv::Mat::zeros(1, 32, CV_8U);
  cv::Mat b = a.clone();
  b.at<std::uint8_t>(0, 3) = 0xFF;
  b.at<std::uint8_t>(0, 31) = 0x01;
  EXPECT_EQ(pimesh_backend::hamming(a, b), 9);
}
