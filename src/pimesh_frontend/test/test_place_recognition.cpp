// Place recognition (#12's P16) on synthetic keyframes whose answer is known.
//
// The failure this exists for does not look like a failure: a false closure is a
// confident, well-matched, wrong answer, and the pose graph downstream will pull two
// places together on it. So the sharpest case here is a candidate whose descriptors
// match as well as a true revisit's and whose geometry does not — the repeated
// texture — and it has to be refused on geometry alone.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "pimesh_frontend/place_recognition.hpp"

using pimesh_frontend::find_place;
using pimesh_frontend::Keyframe;
using pimesh_frontend::PlaceConfig;
using pimesh_frontend::PlaceRecognizer;
using pimesh_frontend::PlaceResult;

namespace
{

constexpr std::int64_t kSecond = 1000000000LL;

cv::Matx33d yaw(double a)
{
  return cv::Matx33d(std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a));
}

/// A room: points, and one 256-bit descriptor per point — what ORB would see there.
struct Room
{
  std::vector<cv::Vec3d> points;
  cv::Mat descriptors;

  Room(std::size_t n, unsigned seed)
  {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<double> x(-2.0, 2.0), y(-1.0, 1.0), z(2.0, 5.0);
    std::uniform_int_distribution<int> byte(0, 255);
    descriptors.create(static_cast<int>(n), 32, CV_8U);
    for (std::size_t i = 0; i < n; ++i) {
      points.emplace_back(x(gen), y(gen), z(gen));
      for (int b = 0; b < 32; ++b) {descriptors.at<std::uint8_t>(static_cast<int>(i), b) = static_cast<std::uint8_t>(byte(gen));}
    }
  }
};

/// What a camera at `map_from_camera` keeps as a keyframe: every point in front of it
/// within a 60-degree cone, a few descriptor bits flipped (a real re-detection is
/// never bit-identical), and a depth landmark for two points in three.
///
/// Track ids are offset by the seed: a place seen again after the tracker lost it is
/// re-detected under **new** ids. A test wanting two views the tracker still connects
/// passes the same `track_base` to both.
Keyframe view(const Room & room, const cv::Affine3d & map_from_camera, std::int64_t stamp,
  unsigned seed, std::int32_t track_base = -1)
{
  if (track_base < 0) {track_base = static_cast<std::int32_t>(seed) * 100000;}
  std::mt19937 gen(seed);
  std::uniform_int_distribution<int> bit(0, 255);
  Keyframe kf;
  kf.stamp_ns = stamp;
  kf.odom_from_camera = map_from_camera;
  const cv::Affine3d camera_from_map = map_from_camera.inv();
  std::vector<int> rows;
  for (std::size_t i = 0; i < room.points.size(); ++i) {
    const cv::Vec3d p = camera_from_map * room.points[i];
    if (p[2] < 0.5 || std::abs(p[0] / p[2]) > 0.6 || std::abs(p[1] / p[2]) > 0.45) {continue;}
    rows.push_back(static_cast<int>(i));
    kf.bearings.push_back(p / cv::norm(p));
    kf.track_ids.push_back(track_base + static_cast<std::int32_t>(i));
    if (i % 3 != 0) {
      kf.landmark_row.push_back(static_cast<std::int32_t>(rows.size() - 1));
      kf.landmarks.push_back(p);
    }
  }
  kf.descriptors.create(static_cast<int>(rows.size()), 32, CV_8U);
  for (std::size_t r = 0; r < rows.size(); ++r) {
    room.descriptors.row(rows[r]).copyTo(kf.descriptors.row(static_cast<int>(r)));
    for (int flip = 0; flip < 6; ++flip) {
      const int b = bit(gen);
      kf.descriptors.at<std::uint8_t>(static_cast<int>(r), b / 8) ^= static_cast<std::uint8_t>(1u << (b % 8));
    }
  }
  return kf;
}

PlaceConfig config()
{
  PlaceConfig c;
  c.focal_px = 517.0;
  return c;
}

double rotation_error_deg(const cv::Affine3d & a, const cv::Affine3d & b)
{
  return pimesh_frontend::angle_between(a.rotation(), b.rotation()) * 180.0 / CV_PI;
}

}  // namespace

TEST(FindPlace, RecognisesARevisitAndRecoversTheRelativePose)
{
  const Room room(400, 1);
  const Room elsewhere(400, 2);
  const cv::Affine3d here(yaw(0.0), cv::Vec3d(0, 0, 0));
  const cv::Affine3d back(yaw(0.12), cv::Vec3d(0.2, 0.0, 0.1));
  std::deque<Keyframe> db;
  db.push_back(view(room, here, 0, 10));
  db.push_back(view(elsewhere, here, 5 * kSecond, 11));
  const Keyframe query = view(room, back, 20 * kSecond, 12);

  const PlaceResult r = find_place(db, query, config());
  // The candidate is stamped 0 on purpose: the first version used a zero stamp to mean
  // "no candidate" and refused this revisit at 205 inliers.
  ASSERT_TRUE(r.accepted) << "inliers " << r.best.inliers;
  EXPECT_EQ(r.best.candidate_stamp_ns, 0);
  EXPECT_EQ(r.searched, 2u);
  // The edge P17 will carry: candidate frame into query frame.
  const cv::Affine3d truth = back.inv() * here;
  EXPECT_LT(rotation_error_deg(r.best.query_from_candidate, truth), 0.5);
  EXPECT_LT(cv::norm(r.best.query_from_candidate.translation() - truth.translation()), 0.02);
}

TEST(FindPlace, RepeatedTextureWithoutTheGeometryIsRefused)
{
  // **The false closure.** The candidate carries the query's own descriptors, so it
  // wins the descriptor vote outright, but its landmarks are the room's points
  // shuffled among themselves — the same corners, somewhere else. A detector that
  // accepted on match count would take it; only the PnP can say no.
  const Room room(400, 1);
  const cv::Affine3d here(yaw(0.0), cv::Vec3d(0, 0, 0));
  Keyframe fake = view(room, here, 0, 10);
  std::mt19937 gen(7);
  std::shuffle(fake.landmarks.begin(), fake.landmarks.end(), gen);
  std::deque<Keyframe> db{fake};
  const Keyframe query = view(room, here, 20 * kSecond, 12);

  const PlaceResult r = find_place(db, query, config());
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.verified, 1u);
  EXPECT_GE(r.best.matches, 100u) << "the control: descriptors alone would have accepted this";
  EXPECT_LT(r.best.inliers, config().min_inliers);
}

TEST(FindPlace, AnythingInsideTheGapIsNotSearched)
{
  // The same place a second later is odometry, not a revisit.
  const Room room(400, 1);
  const cv::Affine3d here(yaw(0.0), cv::Vec3d(0, 0, 0));
  std::deque<Keyframe> db{view(room, here, 0, 10)};
  const Keyframe query = view(room, here, 1 * kSecond, 12);
  const PlaceResult r = find_place(db, query, config());
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.searched, 0u);
  EXPECT_EQ(r.too_recent, 1u);
  // And "nothing verified" has its own spelling — `verified == 0` — rather than a
  // sentinel stamp. The candidate here *is* stamped 0, which is exactly why.
  EXPECT_EQ(r.verified, 0u);
}

TEST(FindPlace, AViewTheTrackerStillFollowsIsNotARevisit)
{
  // Found on bags/desk1: with the time gap alone, 7 of 16 queries closed onto the
  // keyframe just before them. Same place, same features under the same track ids —
  // odometry's own edge, not a loop. Both views here share every id.
  const Room room(400, 1);
  const cv::Affine3d here(yaw(0.0), cv::Vec3d(0, 0, 0));
  std::deque<Keyframe> db{view(room, here, 0, 10, 7)};
  const Keyframe query = view(room, cv::Affine3d(yaw(0.1), cv::Vec3d(0.1, 0, 0)), 20 * kSecond, 12, 7);
  const PlaceResult r = find_place(db, query, config());
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.still_tracked, 1u);
  EXPECT_EQ(r.searched, 0u);

  // The control: the same two views under fresh ids are a revisit, and accepted.
  const Keyframe relost = view(room, cv::Affine3d(yaw(0.1), cv::Vec3d(0.1, 0, 0)), 20 * kSecond, 12);
  EXPECT_TRUE(find_place(db, relost, config()).accepted);
}

TEST(FindPlace, AnUnrelatedPlaceNeverReachesVerification)
{
  const Room room(400, 1);
  const Room elsewhere(400, 2);
  const cv::Affine3d here(yaw(0.0), cv::Vec3d(0, 0, 0));
  std::deque<Keyframe> db{view(elsewhere, here, 0, 10)};
  const Keyframe query = view(room, here, 20 * kSecond, 12);
  const PlaceResult r = find_place(db, query, config());
  EXPECT_FALSE(r.accepted);
  EXPECT_EQ(r.searched, 1u);
  EXPECT_EQ(r.verified, 0u);
}

TEST(FindPlace, OnlyTheTopCandidatesAreVerified)
{
  // Verification is the expensive half. Five views of the same place, top three.
  const Room room(400, 1);
  std::deque<Keyframe> db;
  for (int i = 0; i < 5; ++i) {
    db.push_back(view(room, cv::Affine3d(yaw(0.02 * i), cv::Vec3d(0, 0, 0)), i * kSecond,
      static_cast<unsigned>(20 + i)));
  }
  const Keyframe query = view(room, cv::Affine3d::Identity(), 30 * kSecond, 12);
  const PlaceResult r = find_place(db, query, config());
  EXPECT_TRUE(r.accepted);
  EXPECT_EQ(r.searched, 5u);
  EXPECT_EQ(r.verified, config().candidates);
}

// --- The thread -------------------------------------------------------------------

TEST(PlaceRecognizer, ABacklogIsAddedInFullAndOnlyItsNewestQueried)
{
  // Submitted before the thread starts, so all five arrive as one backlog. The
  // database must hold all five — a dropped keyframe is a place that can never be
  // recognised — and four queries were skipped, and said so.
  const Room room(200, 1);
  PlaceRecognizer::Config c;
  c.place = config();
  c.nice = 0;
  PlaceRecognizer recognizer(c);
  for (int i = 0; i < 5; ++i) {
    recognizer.submit(view(room, cv::Affine3d::Identity(), i * 10 * kSecond, static_cast<unsigned>(i)));
  }
  recognizer.start();
  recognizer.flush();
  const PlaceRecognizer::Stats s = recognizer.stats();
  EXPECT_EQ(s.submitted, 5u);
  EXPECT_EQ(s.database, 5u);
  EXPECT_EQ(s.queries, 1u);
  EXPECT_EQ(s.skipped, 4u);
  const std::vector<PlaceResult> results = recognizer.take_results();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].query_stamp_ns, 40 * kSecond);
  EXPECT_EQ(results[0].searched, 4u);
  EXPECT_TRUE(results[0].accepted);
  EXPECT_TRUE(recognizer.take_results().empty()) << "results are handed over once";
}

TEST(PlaceRecognizer, TheDatabaseIsBoundedOldestFirst)
{
  const Room room(100, 1);
  PlaceRecognizer::Config c;
  c.place = config();
  c.max_keyframes = 3;
  c.nice = 0;
  PlaceRecognizer recognizer(c);
  recognizer.start();
  for (int i = 0; i < 6; ++i) {
    recognizer.submit(view(room, cv::Affine3d::Identity(), i * 10 * kSecond, static_cast<unsigned>(i)));
    recognizer.flush();
  }
  EXPECT_EQ(recognizer.stats().database, 3u);
  EXPECT_EQ(recognizer.stats().queries, 6u);
  // The last query searched the three before it and no more.
  const std::vector<PlaceResult> results = recognizer.take_results();
  ASSERT_EQ(results.size(), 6u);
  EXPECT_EQ(results.back().searched, 3u);
}

TEST(PlaceRecognizer, StopsWhenIdle)
{
  // The worker parked on its condition variable must be woken by the destructor —
  // LocalMapper's AnIdleThreadStopsToo found that a test which destroys the object
  // before the worker reaches its wait tests nothing, so this one waits first.
  PlaceRecognizer::Config c;
  c.nice = 0;
  auto recognizer = std::make_unique<PlaceRecognizer>(c);
  recognizer->start();
  const Room room(50, 1);
  recognizer->submit(view(room, cv::Affine3d::Identity(), 0, 1));
  recognizer->flush();
  recognizer.reset();
  SUCCEED();
}

namespace
{

/// Flip exactly `bits` distinct bits in all but every `keep`-th descriptor row — a
/// re-detection from far enough away that most corners describe themselves
/// differently.
void degrade(Keyframe & kf, int bits, int keep, unsigned seed)
{
  std::mt19937 gen(seed);
  for (int r = 0; r < kf.descriptors.rows; ++r) {
    if (r % keep == 0) {continue;}
    std::vector<int> order(256);
    for (int i = 0; i < 256; ++i) {order[static_cast<std::size_t>(i)] = i;}
    std::shuffle(order.begin(), order.end(), gen);
    for (int i = 0; i < bits; ++i) {
      const int b = order[static_cast<std::size_t>(i)];
      kf.descriptors.at<std::uint8_t>(r, b / 8) ^= static_cast<std::uint8_t>(1u << (b % 8));
    }
  }
}

}  // namespace

TEST(FindPlace, TheGuidedSearchFindsWhatTheBlindOneCouldNot)
{
  // bags/walk1's revisit of its own start, in miniature: the right candidate, too
  // few blind matches to pass on their own. 55 flipped bits is past the blind
  // search's 50 and inside the guided 64, on all but one corner in sixteen. The
  // RANSAC seed has to be below the floor and the final count above it — both
  // asserted, so this cannot pass by the blind search simply succeeding.
  const Room room(600, 1);
  const cv::Affine3d here(yaw(0.0), cv::Vec3d(0, 0, 0));
  const cv::Affine3d back(yaw(0.1), cv::Vec3d(0.15, 0.0, 0.05));
  std::deque<Keyframe> db{view(room, here, 0, 10)};
  Keyframe query = view(room, back, 20 * kSecond, 12);
  degrade(query, 55, 16, 3);

  const PlaceResult r = find_place(db, query, config());
  ASSERT_EQ(r.verified, 1u);
  EXPECT_LT(r.best.ransac_inliers, config().min_inliers);
  EXPECT_GE(r.best.ransac_inliers, config().min_ransac_inliers);
  EXPECT_GE(r.best.inliers, config().min_inliers);
  EXPECT_TRUE(r.accepted);
  const cv::Affine3d truth = back.inv() * here;
  EXPECT_LT(rotation_error_deg(r.best.query_from_candidate, truth), 0.5);
}

// --- Gaps closed after the first gate run ------------------------------------------

TEST(FindPlace, ASingleCandidateCornerCannotCollectTheWholeQuery)
{
  // The one-to-one claim in the blind search. A candidate with one descriptor, and a
  // query of fifty near-copies of it: without one-to-one every query corner matches
  // that single row (k-NN over one row has no second neighbour, so the ratio test
  // never fires) and the candidate reads as fifty matches. With it, one.
  Keyframe candidate;
  candidate.stamp_ns = 0;
  candidate.descriptors = cv::Mat::zeros(1, 32, CV_8U);
  candidate.bearings.push_back(cv::Vec3d(0, 0, 1));
  candidate.track_ids.push_back(1);
  Keyframe query;
  query.stamp_ns = 20 * kSecond;
  query.descriptors = cv::Mat::zeros(50, 32, CV_8U);
  for (int r = 0; r < 50; ++r) {
    query.descriptors.at<std::uint8_t>(r, r % 32) = static_cast<std::uint8_t>(1u << (r % 8));
    query.bearings.push_back(cv::Vec3d(0, 0, 1));
    query.track_ids.push_back(1000 + r);
  }
  PlaceConfig c = config();
  c.min_matches = 1;
  const PlaceResult r = find_place(std::deque<Keyframe>{candidate}, query, c);
  ASSERT_EQ(r.verified, 1u);
  EXPECT_EQ(r.best.matches, 1u);
}

TEST(FindPlace, RaggedKeyframesAreNotReadPastTheEnd)
{
  // Keyframe is struct-of-arrays, the hazard odometry_node's keypoints_view had:
  // nothing enforces that the arrays agree. A landmark_row pointing past the
  // descriptors, more landmarks than rows, and a query with fewer bearings than
  // descriptors — each must be skipped, not dereferenced. The expectation is on the
  // result as well as on surviving: the well-formed part still verifies.
  const Room room(400, 1);
  const cv::Affine3d here(yaw(0.0), cv::Vec3d(0, 0, 0));
  Keyframe candidate = view(room, here, 0, 10);
  // **Far** past the end, on purpose. The first version pointed 50 rows past it; with
  // the bound check deleted, that write landed in mapped heap and this test passed
  // over undefined behaviour. 2^28 rows is ~1 GB beyond the allocation, so a missing
  // guard faults instead of passing.
  candidate.landmark_row.push_back(1 << 28);
  candidate.landmarks.push_back(cv::Vec3d(0, 0, 3));
  candidate.landmarks.push_back(cv::Vec3d(0, 0, 4));  // one more than landmark_row
  Keyframe query = view(room, cv::Affine3d(yaw(0.05), cv::Vec3d(0.1, 0, 0)), 20 * kSecond, 12);
  query.bearings.resize(query.bearings.size() - 20);
  const PlaceResult r = find_place(std::deque<Keyframe>{candidate}, query, config());
  EXPECT_EQ(r.verified, 1u);
  EXPECT_TRUE(r.accepted);
}

TEST(PlaceRecognizer, AQueryNeverFindsItselfEvenWithNoGap)
{
  // The thread searches the database *before* the query joins it. With the gap at 0
  // and the tracked-candidate rule disabled, that ordering is the only thing stopping
  // a keyframe from recognising itself at a perfect score.
  const Room room(200, 1);
  PlaceRecognizer::Config c;
  c.place = config();
  c.place.min_gap_s = 0.0;
  c.place.max_shared_tracks = 1000000;
  c.nice = 0;
  PlaceRecognizer recognizer(c);
  recognizer.start();
  recognizer.submit(view(room, cv::Affine3d::Identity(), 5 * kSecond, 1));
  recognizer.flush();
  const std::vector<PlaceResult> results = recognizer.take_results();
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].searched, 0u);
  EXPECT_FALSE(results[0].accepted);
  EXPECT_EQ(recognizer.stats().database, 1u);
}

TEST(PlaceRecognizer, StopsWithWorkStillQueued)
{
  // Destroyed with a backlog it never started on: the destructor must not wait for
  // the backlog, and must not deadlock on a worker that was never started.
  const Room room(100, 1);
  PlaceRecognizer::Config c;
  c.nice = 0;
  {
    PlaceRecognizer never_started(c);
    for (int i = 0; i < 5; ++i) {
      never_started.submit(view(room, cv::Affine3d::Identity(), i * 10 * kSecond, static_cast<unsigned>(i)));
    }
  }
  auto running = std::make_unique<PlaceRecognizer>(c);
  running->start();
  for (int i = 0; i < 5; ++i) {
    running->submit(view(room, cv::Affine3d::Identity(), i * 10 * kSecond, static_cast<unsigned>(i)));
  }
  running.reset();
  SUCCEED();
}
