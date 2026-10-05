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
#include <memory>
#include <random>
#include <vector>

#include "pimesh_frontend/place_recognition.hpp"
#include "pimesh_frontend/relocaliser.hpp"

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

// --- Loop closure through the thread (#12's P17) ------------------------------------

namespace
{

/// Two views of one room, the second after odometry drifted: its odom pose is 0.5 m
/// off where the camera truly was. The closure the search finds carries the truth.
struct DriftedRevisit
{
  Room room {400, 1};
  cv::Affine3d here {yaw(0.0), cv::Vec3d(0, 0, 0)};
  cv::Affine3d back {yaw(0.1), cv::Vec3d(0.2, 0.0, 0.1)};
  cv::Affine3d drift {cv::Matx33d::eye(), cv::Vec3d(0.5, 0.0, 0.0)};

  Keyframe first() const {return view(room, here, 0, 10);}
  Keyframe second() const
  {
    Keyframe kf = view(room, back, 20 * kSecond, 12);
    kf.odom_from_camera = drift * back;   // where odometry *thinks* it is
    return kf;
  }
};

PlaceRecognizer::Config loop_config(bool close_loops)
{
  PlaceRecognizer::Config c;
  c.place = config();
  c.nice = 0;
  c.close_loops = close_loops;
  return c;
}

}  // namespace

TEST(PlaceRecognizerLoops, AnAcceptedClosureMovesMapFromOdomOnlyWhenClosingLoops)
{
  // The control P17's gate runs is the same search with close_loops off: the closure
  // is found and map -> odom stays identity, exactly. On, it moves — toward undoing
  // the 0.5 m of drift.
  const DriftedRevisit scene;
  for (bool close : {false, true}) {
    PlaceRecognizer recognizer(loop_config(close));
    recognizer.start();
    recognizer.submit(scene.first());
    recognizer.flush();
    recognizer.submit(scene.second());
    recognizer.flush();
    ASSERT_EQ(recognizer.stats().accepted, 1u) << "close_loops=" << close;
    const double moved = cv::norm(recognizer.map_from_odom().translation());
    if (close) {
      EXPECT_EQ(recognizer.stats().loops, 1u);
      EXPECT_EQ(recognizer.stats().solves, 1u);
      EXPECT_GT(moved, 0.1);
      // The corrected second keyframe is nearer the truth than odometry was.
      const auto traj = recognizer.trajectory();
      ASSERT_EQ(traj.size(), 2u);
      EXPECT_LT(cv::norm(traj[1].second.translation() - scene.back.translation()), 0.5);
    } else {
      EXPECT_EQ(recognizer.stats().loops, 0u);
      EXPECT_EQ(moved, 0.0);
    }
  }
}

TEST(PlaceRecognizerLoops, EveryKeyframeReachesTheGraphIncludingSkippedQueries)
{
  // A backlog is queried only at its newest, but every keyframe in it is an odometry
  // edge. Five submitted before the thread starts: five in the trajectory, in order.
  const Room room(100, 1);
  PlaceRecognizer recognizer(loop_config(true));
  for (int i = 0; i < 5; ++i) {
    recognizer.submit(view(room, cv::Affine3d::Identity(), i * 10 * kSecond, static_cast<unsigned>(i)));
  }
  recognizer.start();
  recognizer.flush();
  EXPECT_EQ(recognizer.stats().skipped, 4u);
  const auto traj = recognizer.trajectory();
  ASSERT_EQ(traj.size(), 5u);
  for (std::size_t i = 0; i < traj.size(); ++i) {
    EXPECT_EQ(traj[i].first, static_cast<std::int64_t>(i) * 10 * kSecond);
  }
}

TEST(PlaceRecognizerLoops, TheCorrectionsHandedOutReproduceTheCorrectedTrajectory)
{
  // What /pose_graph/corrections carries, at the thread rather than inside the graph:
  // each keyframe's correction applied to its odometry pose must be its corrected
  // pose, and before any closure every correction must be identity exactly — P18's
  // rebuild applies these to every remembered frame, so a correction off by a
  // keyframe is a surface rebuilt in the wrong place.
  const DriftedRevisit scene;
  PlaceRecognizer recognizer(loop_config(true));
  recognizer.start();
  recognizer.submit(scene.first());
  recognizer.flush();
  for (const auto & [stamp, c] : recognizer.corrections()) {
    (void)stamp;
    EXPECT_EQ(cv::norm(c.translation()), 0.0) << "identity before any closure";
  }
  recognizer.submit(scene.second());
  recognizer.flush();
  ASSERT_EQ(recognizer.stats().solves, 1u);
  const auto corrections = recognizer.corrections();
  const auto trajectory = recognizer.trajectory();
  ASSERT_EQ(corrections.size(), 2u);
  ASSERT_EQ(trajectory.size(), 2u);
  const cv::Affine3d odom[2] = {scene.first().odom_from_camera, scene.second().odom_from_camera};
  for (std::size_t k = 0; k < 2; ++k) {
    EXPECT_EQ(corrections[k].first, trajectory[k].first);
    const cv::Affine3d applied = corrections[k].second * odom[k];
    EXPECT_LT(cv::norm(applied.translation() - trajectory[k].second.translation()), 1e-9) << k;
  }
  EXPECT_GT(cv::norm(corrections[1].second.translation()), 0.1) << "the drifted keyframe moved";
}


// --- #13's P20: relocalisation against another session's map ---------------------

using pimesh_frontend::Relocalisation;
using pimesh_frontend::Relocaliser;
using pimesh_frontend::relocalised_pose;

TEST(Relocalisation, ThePoseIsComposedTheRightWayRound)
{
  // Asymmetric on purpose: a rotation *and* an offset, so the inverse composition
  // lands somewhere else. Two coincident views would pass either way, and a
  // relocalisation onto a nearly identical view is exactly the case that would.
  const cv::Affine3d candidate(yaw(0.4), cv::Vec3d(1.0, 0.2, -0.5));
  const cv::Affine3d query(yaw(-0.3), cv::Vec3d(-0.7, 0.0, 1.1));
  const cv::Affine3d query_from_candidate = query.inv() * candidate;
  const cv::Affine3d got = relocalised_pose(candidate, query_from_candidate);
  EXPECT_LT(cv::norm(cv::Vec3d(got.translation()) - cv::Vec3d(query.translation())), 1e-12);
  EXPECT_LT(rotation_error_deg(got, query), 1e-9);
  // And the composition a hurried reading of the name suggests is not the same pose.
  const cv::Affine3d wrong = candidate * query_from_candidate;
  EXPECT_GT(cv::norm(cv::Vec3d(wrong.translation()) - cv::Vec3d(query.translation())), 0.5);
}

TEST(FindPlace, AcrossSessionsNeitherStampsNorTrackIdsExclude)
{
  // A saved keyframe from a *later* recording, sharing every track id with the query
  // because both sessions numbered their tracks from zero. Within one session both
  // facts would exclude it; across sessions neither means anything.
  const Room room(400, 1);
  const cv::Affine3d here(yaw(0.0), cv::Vec3d(0, 0, 0));
  std::deque<Keyframe> db{view(room, here, 900 * kSecond, 10, 0)};
  const Keyframe query = view(room, cv::Affine3d(yaw(0.1), cv::Vec3d(0.1, 0, 0)), 5 * kSecond, 11, 0);

  const PlaceResult within = find_place(db, query, config());
  EXPECT_FALSE(within.accepted);
  EXPECT_EQ(within.searched, 0u);

  PlaceConfig across = config();
  across.across_sessions = true;
  const PlaceResult r = find_place(db, query, across);
  EXPECT_EQ(r.searched, 1u);
  EXPECT_TRUE(r.accepted) << r.best.inliers << " inliers";
}

namespace
{

Relocaliser::Config reloc_config()
{
  Relocaliser::Config c;
  c.place = config();
  c.nice = 0;
  return c;
}

}  // namespace

TEST(Relocaliser, RecoversTheQuerysPoseInTheSavedMap)
{
  // The saved session's keyframes, at their map poses; this session's query, at a
  // known map pose but with an odom pose that has nothing to do with it — a new
  // session's odom starts wherever the camera happened to be.
  const Room room(500, 3);
  std::deque<Keyframe> map;
  map.push_back(view(room, cv::Affine3d(yaw(-0.2), cv::Vec3d(-0.4, 0, 0)), 1 * kSecond, 20));
  map.push_back(view(room, cv::Affine3d(yaw(0.25), cv::Vec3d(0.5, 0.1, 0.2)), 2 * kSecond, 21));
  const cv::Affine3d truth(yaw(0.15), cv::Vec3d(0.3, 0.05, 0.1));
  Keyframe query = view(room, truth, 5 * kSecond, 22, 0);
  const cv::Affine3d odom(yaw(1.2), cv::Vec3d(7.0, -3.0, 0.4));
  query.odom_from_camera = odom;

  Relocaliser reloc(reloc_config(), map);
  reloc.start();
  reloc.submit(query);
  reloc.flush();
  const std::vector<Relocalisation> out = reloc.take_results();
  ASSERT_EQ(out.size(), 1u);
  const Relocalisation & r = out[0];
  ASSERT_TRUE(r.accepted) << r.inliers << " inliers";
  EXPECT_EQ(r.candidate_stamp_ns, 2 * kSecond) << "the nearer view should win";
  EXPECT_LT(cv::norm(cv::Vec3d(r.map_from_camera.translation()) - cv::Vec3d(truth.translation())), 0.02);
  EXPECT_LT(rotation_error_deg(r.map_from_camera, truth), 0.5);
  // The odom pose comes back as submitted, so map <- odom is one product away.
  EXPECT_EQ(cv::norm(r.odom_from_camera.matrix - odom.matrix), 0.0);
  const cv::Affine3d map_from_odom = r.map_from_camera * r.odom_from_camera.inv();
  EXPECT_LT(cv::norm(cv::Vec3d((map_from_odom * odom).translation()) - cv::Vec3d(truth.translation())), 0.02);
  EXPECT_EQ(reloc.stats().accepted, 1u);
}

TEST(Relocaliser, AnotherRoomIsRefused)
{
  // The control the gate runs on real data, here on synthetic: a confident search
  // over a map of somewhere else must come back empty.
  const Room saved(500, 4);
  const Room elsewhere(500, 5);
  std::deque<Keyframe> map;
  for (int i = 0; i < 5; ++i) {
    map.push_back(view(saved, cv::Affine3d(yaw(0.1 * i), cv::Vec3d(0.1 * i, 0, 0)), i * kSecond, 30 + i));
  }
  Relocaliser reloc(reloc_config(), map);
  reloc.start();
  for (int i = 0; i < 4; ++i) {
    reloc.submit(view(elsewhere, cv::Affine3d(yaw(0.05 * i), cv::Vec3d(0, 0, 0)), 100 * kSecond + i, 40 + i, 0));
    reloc.flush();
  }
  for (const Relocalisation & r : reloc.take_results()) {EXPECT_FALSE(r.accepted);}
  EXPECT_EQ(reloc.stats().queries, 4u);
  EXPECT_EQ(reloc.stats().accepted, 0u);
}

TEST(Relocaliser, TheNewestQueryWins)
{
  // Submitted before the thread starts, so none has been taken: the two older ones
  // are replaced, counted as skipped, and never searched.
  const Room room(300, 6);
  std::deque<Keyframe> map{view(room, cv::Affine3d::Identity(), 0, 50)};
  Relocaliser reloc(reloc_config(), map);
  for (int i = 1; i <= 3; ++i) {
    reloc.submit(view(room, cv::Affine3d::Identity(), i * kSecond, 50 + i, 0));
  }
  reloc.start();
  reloc.flush();
  const std::vector<Relocalisation> out = reloc.take_results();
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(out[0].query_stamp_ns, 3 * kSecond);
  EXPECT_EQ(reloc.stats().skipped, 2u);
  EXPECT_EQ(reloc.stats().submitted, 3u);
}

TEST(Relocaliser, StopsWithAQueryWaiting)
{
  const Room room(300, 7);
  auto reloc = std::make_unique<Relocaliser>(
    reloc_config(), std::deque<Keyframe>{view(room, cv::Affine3d::Identity(), 0, 60)});
  reloc->submit(view(room, cv::Affine3d::Identity(), kSecond, 61, 0));
  reloc.reset();   // never started: the destructor must not wait for a thread
  SUCCEED();
}
