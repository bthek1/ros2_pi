// The pose graph (#12's P17) on synthetic trajectories whose truth is known.
//
// Every way this goes wrong converges. A measurement composed in the wrong
// direction, an information matrix with rotation and translation swapped, a
// correction that does not reach keyframes added after it — each produces a graph
// that optimises to a small residual and a trajectory that is confidently wrong.
// P7's rotation was composed inverted for six days with every internal number
// right; these pin the conventions against answers worked out by hand.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "pimesh_backend/pose_graph.hpp"

using pimesh_backend::PoseGraph;
using pimesh_backend::PoseGraphConfig;
using pimesh_backend::PoseGraphResult;

namespace
{

constexpr std::int64_t kSecond = 1000000000LL;

cv::Matx33d yaw(double a)
{
  // About the optical frame's y axis, which points down: a camera turning in place.
  return cv::Matx33d(std::cos(a), 0, std::sin(a), 0, 1, 0, -std::sin(a), 0, std::cos(a));
}

double deg(const cv::Matx33d & r)
{
  const double c = std::max(-1.0, std::min(1.0, (cv::trace(r) - 1.0) / 2.0));
  return std::acos(c) * 180.0 / CV_PI;
}

/// A camera walking a closed square, 4 m a side, turning 90 degrees at each corner:
/// eight keyframes, the last back at the first's pose.
std::vector<cv::Affine3d> square_truth()
{
  std::vector<cv::Affine3d> truth;
  cv::Affine3d pose = cv::Affine3d::Identity();
  for (int i = 0; i < 8; ++i) {
    truth.push_back(pose);
    // Forward 2 m along the camera's own z, and at every second keyframe a corner.
    const cv::Affine3d step(i % 2 == 1 ? yaw(CV_PI / 2) : cv::Matx33d::eye(), cv::Vec3d(0, 0, 2.0));
    pose = pose * step;
  }
  truth.push_back(pose);  // the ninth, back where the first was
  return truth;
}

/// Odometry that turns 3 degrees too far at every step — drift, compounding.
std::vector<cv::Affine3d> drifting(const std::vector<cv::Affine3d> & truth)
{
  std::vector<cv::Affine3d> odom{truth[0]};
  for (std::size_t i = 1; i < truth.size(); ++i) {
    const cv::Affine3d step = truth[i - 1].inv() * truth[i];
    const cv::Affine3d biased(yaw(3.0 * CV_PI / 180.0) * step.rotation(), step.translation());
    odom.push_back(odom.back() * biased);
  }
  return odom;
}

}  // namespace

TEST(PoseGraph, WithoutALoopItIsTheOdometryChainAndDoesNotRun)
{
  // The control P17's gate runs: loop_closure off is this graph with no loop edge,
  // and its answer has to be odometry exactly — not odometry plus a solver's
  // round-off, which is why it refuses rather than solving to confirm it.
  PoseGraph graph(PoseGraphConfig{});
  const auto truth = square_truth();
  for (std::size_t i = 0; i < truth.size(); ++i) {graph.add_keyframe(static_cast<std::int64_t>(i) * kSecond, truth[i]);}
  const PoseGraphResult r = graph.optimize();
  EXPECT_FALSE(r.ran);
  EXPECT_STREQ(r.refusal, "no loop");
  const auto traj = graph.trajectory();
  for (std::size_t i = 0; i < truth.size(); ++i) {
    EXPECT_EQ(cv::norm(traj[i].second.translation() - truth[i].translation()), 0.0);
  }
  EXPECT_EQ(cv::norm(graph.map_from_odom().translation()), 0.0);
}

TEST(PoseGraph, AClosureTakesOutTheDriftAroundTheLoop)
{
  const auto truth = square_truth();
  const auto odom = drifting(truth);
  PoseGraph graph(PoseGraphConfig{});
  for (std::size_t i = 0; i < odom.size(); ++i) {graph.add_keyframe(static_cast<std::int64_t>(i) * kSecond, odom[i]);}
  const double odom_error = cv::norm(odom.back().translation() - truth.back().translation());
  ASSERT_GT(odom_error, 1.0) << "the scene has to drift for the closure to have work to do";

  // The last keyframe recognises the first: the true relative pose between them.
  const std::int64_t last = static_cast<std::int64_t>(odom.size() - 1) * kSecond;
  ASSERT_TRUE(graph.add_loop(last, 0, truth.back().inv() * truth.front()));
  const PoseGraphResult r = graph.optimize();
  ASSERT_TRUE(r.ran) << r.refusal;
  EXPECT_GT(r.iterations, 0);
  EXPECT_LT(r.chi2_after, r.chi2_before);
  EXPECT_EQ(r.inconsistent_loops, 0u);

  const auto traj = graph.trajectory();
  const double corrected_error = cv::norm(traj.back().second.translation() - truth.back().translation());
  EXPECT_LT(corrected_error, 0.2 * odom_error);
  EXPECT_LT(deg(traj.back().second.rotation().t() * truth.back().rotation()), 3.0);
  // And the correction is what TF will publish, not identity.
  EXPECT_GT(r.correction_m, 1.0);
  EXPECT_GT(cv::norm(graph.map_from_odom().translation()), 0.5);
}

TEST(PoseGraph, TheLoopMeasurementIsQueryFromCandidateNotItsInverse)
{
  // Two keyframes on a line. Odometry puts the second 1 m ahead of the first along
  // x; the closure says 2 m. The answer lies between — **on the far side of 1 m**.
  // A measurement taken as candidate_from_query instead pulls it back past the
  // origin, toward -2, and converges just as happily.
  PoseGraph graph(PoseGraphConfig{});
  graph.add_keyframe(0, cv::Affine3d::Identity());
  graph.add_keyframe(10 * kSecond, cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(1, 0, 0)));
  const cv::Affine3d query_pose(cv::Matx33d::eye(), cv::Vec3d(2, 0, 0));
  ASSERT_TRUE(graph.add_loop(10 * kSecond, 0, query_pose.inv() * cv::Affine3d::Identity()));
  ASSERT_TRUE(graph.optimize().ran);
  const double x = graph.trajectory().back().second.translation()[0];
  EXPECT_GT(x, 1.05);
  EXPECT_LT(x, 2.0);
}

TEST(PoseGraph, RotationAndTranslationUncertaintyAreNotSwapped)
{
  // The same two-keyframe line, with a closure that disagrees only in translation.
  // Give the closure a translation sigma of a kilometre and a rotation sigma of a
  // hundredth of a degree: it should be ignored, and the pose stay at odometry's
  // 1 m. With the information matrix's halves swapped the tiny sigma lands on
  // translation and the closure wins outright, at 2 m.
  PoseGraphConfig c;
  c.loop_trans_m = 1000.0;
  c.loop_rot_deg = 0.01;
  PoseGraph graph(c);
  graph.add_keyframe(0, cv::Affine3d::Identity());
  graph.add_keyframe(10 * kSecond, cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(1, 0, 0)));
  const cv::Affine3d query_pose(cv::Matx33d::eye(), cv::Vec3d(2, 0, 0));
  ASSERT_TRUE(graph.add_loop(10 * kSecond, 0, query_pose.inv()));
  ASSERT_TRUE(graph.optimize().ran);
  EXPECT_NEAR(graph.trajectory().back().second.translation()[0], 1.0, 0.01);
}

TEST(PoseGraph, AKeyframeAddedAfterAClosureStartsInTheCorrectedFrame)
{
  // Otherwise every keyframe after a closure jumps back to raw odometry until the
  // next one, and map -> odom — computed at the newest keyframe — flips back to
  // identity with it.
  const auto truth = square_truth();
  const auto odom = drifting(truth);
  PoseGraph graph(PoseGraphConfig{});
  for (std::size_t i = 0; i < odom.size(); ++i) {graph.add_keyframe(static_cast<std::int64_t>(i) * kSecond, odom[i]);}
  const std::int64_t last = static_cast<std::int64_t>(odom.size() - 1) * kSecond;
  ASSERT_TRUE(graph.add_loop(last, 0, truth.back().inv() * truth.front()));
  ASSERT_TRUE(graph.optimize().ran);
  const cv::Affine3d correction = graph.map_from_odom();

  const cv::Affine3d next_odom = odom.back() * cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(0, 0, 0.5));
  graph.add_keyframe(last + kSecond, next_odom);
  const cv::Affine3d added = graph.trajectory().back().second;
  EXPECT_LT(cv::norm(added.translation() - (correction * next_odom).translation()), 1e-9);
  EXPECT_LT(cv::norm(graph.map_from_odom().translation() - correction.translation()), 1e-9);
}

TEST(PoseGraph, AClosureTheGraphCannotReconcileIsReportedWhenOdometryIsTight)
{
  // A false closure claiming the last keyframe is 5 m from where the whole chain
  // says, against odometry trusted to a degree and two centimetres a second. The
  // Huber kernel bounds its pull, and the residual it is left with says so.
  const auto truth = square_truth();
  PoseGraphConfig tight;
  tight.odom_rot_deg_per_s = 1.0;
  tight.odom_trans_m_per_s = 0.02;
  PoseGraph graph(tight);
  for (std::size_t i = 0; i < truth.size(); ++i) {graph.add_keyframe(static_cast<std::int64_t>(i) * kSecond, truth[i]);}
  const std::int64_t last = static_cast<std::int64_t>(truth.size() - 1) * kSecond;
  ASSERT_TRUE(graph.add_loop(last, 0, cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(5, 0, 0))));
  const PoseGraphResult r = graph.optimize();
  ASSERT_TRUE(r.ran);
  EXPECT_EQ(r.inconsistent_loops, 1u);
}

TEST(PoseGraph, UnderTheMeasuredOdometryModelAFalseClosureIsAbsorbedSilently)
{
  // **The same false closure under the noise model measured on fr1/desk is absorbed
  // completely** — found by the first version of the test above, which expected it
  // reported and got zero. Seven degrees a second on each of eight edges is enough
  // freedom to bend a 16 m square around a 5 m lie. So `inconsistent_loops == 0` is
  // not a safety check at these sigmas; the defence against a wrong-place closure is
  // P16's precision, and this pins that it has to be. If the model tightens enough
  // for this to fail, the graph has become a second line of defence — say so.
  const auto truth = square_truth();
  PoseGraph graph(PoseGraphConfig{});
  for (std::size_t i = 0; i < truth.size(); ++i) {graph.add_keyframe(static_cast<std::int64_t>(i) * kSecond, truth[i]);}
  const std::int64_t last = static_cast<std::int64_t>(truth.size() - 1) * kSecond;
  ASSERT_TRUE(graph.add_loop(last, 0, cv::Affine3d(cv::Matx33d::eye(), cv::Vec3d(5, 0, 0))));
  const PoseGraphResult r = graph.optimize();
  ASSERT_TRUE(r.ran);
  EXPECT_EQ(r.inconsistent_loops, 0u);
  // And it did real damage while saying nothing: the true trajectory was exact.
  EXPECT_GT(cv::norm(graph.trajectory().back().second.translation() - truth.back().translation()), 1.0);
}

TEST(PoseGraph, AClosureBetweenUnknownKeyframesIsRefused)
{
  PoseGraph graph(PoseGraphConfig{});
  graph.add_keyframe(0, cv::Affine3d::Identity());
  graph.add_keyframe(kSecond, cv::Affine3d::Identity());
  EXPECT_FALSE(graph.add_loop(kSecond, 5, cv::Affine3d::Identity()));
  EXPECT_FALSE(graph.add_loop(kSecond, kSecond, cv::Affine3d::Identity())) << "a keyframe onto itself";
  EXPECT_EQ(graph.loops(), 0u);
  PoseGraph empty(PoseGraphConfig{});
  EXPECT_STREQ(empty.optimize().refusal, "fewer than two keyframes");
}

TEST(PoseGraph, PerKeyframeCorrectionsAreExactIdentityUntilASolveAndConsistentAfter)
{
  // What P18's rebuild reads. Before any solve every correction must be identity to
  // the bit — the rebuild's control is "the same memory at the uncorrected poses",
  // and a correction of 1e-14 would make it a rebuild at *nearly* those poses. After
  // one, applying keyframe k's correction to its odometry pose must give its
  // corrected pose, for every k, not only the newest.
  const auto truth = square_truth();
  const auto odom = drifting(truth);
  PoseGraph graph(PoseGraphConfig{});
  for (std::size_t i = 0; i < odom.size(); ++i) {graph.add_keyframe(static_cast<std::int64_t>(i) * kSecond, odom[i]);}
  for (const auto & [stamp, c] : graph.corrections()) {
    (void)stamp;
    EXPECT_EQ(cv::norm(c.translation()), 0.0);
    EXPECT_EQ(cv::norm(cv::Mat(c.rotation() - cv::Matx33d::eye())), 0.0);
  }
  const std::int64_t last = static_cast<std::int64_t>(odom.size() - 1) * kSecond;
  ASSERT_TRUE(graph.add_loop(last, 0, truth.back().inv() * truth.front()));
  ASSERT_TRUE(graph.optimize().ran);
  const auto corrections = graph.corrections();
  const auto traj = graph.trajectory();
  ASSERT_EQ(corrections.size(), odom.size());
  double moved = 0.0;
  for (std::size_t k = 0; k < odom.size(); ++k) {
    EXPECT_EQ(corrections[k].first, traj[k].first);
    const cv::Affine3d applied = corrections[k].second * odom[k];
    EXPECT_LT(cv::norm(applied.translation() - traj[k].second.translation()), 1e-9) << k;
    moved = std::max(moved, cv::norm(corrections[k].second.translation()));
  }
  EXPECT_GT(moved, 1.0) << "the corrections differ from identity after a real closure";
}
