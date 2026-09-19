// P7's geometry, and the loop test_rotation_fit left open.
//
// **The suite that matters here is CameraStep.** Everything else pins a piece:
// that a depth reading unprojects with z and not ray length, that a corner on an
// occlusion edge is refused rather than believed, that a rigid fit recovers a
// known transform and survives outliers. CameraStep pins the *composition* —
// simulate a camera with a known trajectory, show it what it would have seen, run
// the whole estimator, and assert the pose that comes out is the trajectory that
// went in.
//
// That check did not exist until 2026-09-19, and its absence cost six days of a
// published pose that turned the wrong way. `test_rotation_fit` asserted the fit
// was a rotation of the right size about the right axis and that a change of basis
// moved the axis correctly — every property except the direction of the one
// inverse between them. A frame that moves when you pan looks right in RViz, the
// residual gate is indifferent to the sign, and a TSDF built from consistently
// mirrored poses still produces a surface.

#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "opencv2/core.hpp"
#include "pimesh_perception/rgbd_odometry.hpp"
#include "pimesh_perception/rotation_fit.hpp"

using pimesh_perception::bearing;
using pimesh_perception::camera_step;
using pimesh_perception::change_basis;
using pimesh_perception::fit_rigid;
using pimesh_perception::fit_rigid_robust;
using pimesh_perception::fit_rotation_robust;
using pimesh_perception::mean_residual_m;
using pimesh_perception::RigidFit;
using pimesh_perception::ScaleHandling;
using pimesh_perception::sample_depth;
using pimesh_perception::unproject;

namespace
{

/// This camera, at 720p: P9's measured intrinsics.
cv::Matx33d camera_k()
{
  return cv::Matx33d(953.4, 0.0, 627.7, 0.0, 957.6, 334.6, 0.0, 0.0, 1.0);
}

/// base_link <- camera_optical_frame, the inverse of the quaternion
/// pimesh.yaml publishes once as camera_to_optical. Written out rather than
/// derived, because it is the number a test is allowed to hard-code: if the
/// static transform ever changes, this suite should fail and be read.
cv::Matx33d optical_basis()
{
  return cv::Matx33d(
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);
}

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

/// A cloud of points in front of a camera, deterministic across runs.
std::vector<cv::Vec3d> scene(std::size_t n, unsigned seed = 7)
{
  std::mt19937 gen(seed);
  std::uniform_real_distribution<double> lateral(-1.5, 1.5);
  std::uniform_real_distribution<double> depth(1.0, 5.0);
  std::vector<cv::Vec3d> points;
  points.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    points.emplace_back(lateral(gen), lateral(gen), depth(gen));
  }
  return points;
}

std::vector<cv::Vec3d> seen_from(const cv::Affine3d & world_from_camera,
  const std::vector<cv::Vec3d> & world)
{
  const cv::Affine3d camera_from_world = world_from_camera.inv();
  std::vector<cv::Vec3d> out;
  out.reserve(world.size());
  for (const cv::Vec3d & point : world) {out.push_back(cv::Vec3d(camera_from_world * point));}
  return out;
}

double transform_error(const cv::Affine3d & a, const cv::Affine3d & b)
{
  return cv::norm(cv::Matx44d(a.matrix) - cv::Matx44d(b.matrix));
}

}  // namespace

// --- Unprojection ------------------------------------------------------------

TEST(Unproject, UsesZAndNotRayLength)
{
  const cv::Matx33d k = camera_k();

  // At the principal point the two conventions agree exactly, which is why a bug
  // here is invisible to anybody testing on a centred target.
  const cv::Vec3d centre = unproject(k, 627.7, 334.6, 3.0);
  EXPECT_NEAR(centre[0], 0.0, 1e-9);
  EXPECT_NEAR(centre[1], 0.0, 1e-9);
  EXPECT_NEAR(centre[2], 3.0, 1e-12);
  EXPECT_NEAR(cv::norm(centre), 3.0, 1e-9);

  // In the corner they do not. z stays exactly what the depth map said; the
  // distance from the camera is larger, and by a lot — the same 30% disagreement
  // test_tsdf_volume pins on the integrator, which is the convention this has to
  // match or every landmark in the corners of the frame is placed too far out.
  const cv::Vec3d corner = unproject(k, 0.0, 0.0, 3.0);
  EXPECT_NEAR(corner[2], 3.0, 1e-12) << "z must survive unprojection untouched";
  EXPECT_GT(cv::norm(corner), 3.4) << "a corner ray is longer than its z, by construction";
}

TEST(Unproject, InvertsTheProjectionItCameFrom)
{
  const cv::Matx33d k = camera_k();
  for (double x : {12.0, 400.0, 627.7, 1200.0}) {
    for (double y : {5.0, 334.6, 700.0}) {
      const cv::Vec3d p = unproject(k, x, y, 2.5);
      EXPECT_NEAR(k(0, 0) * p[0] / p[2] + k(0, 2), x, 1e-9);
      EXPECT_NEAR(k(1, 1) * p[1] / p[2] + k(1, 2), y, 1e-9);
    }
  }
}

TEST(Unproject, ZeroFocalLengthIsTheOriginRatherThanNaN)
{
  const cv::Vec3d p = unproject(cv::Matx33d::zeros(), 100.0, 100.0, 2.0);
  EXPECT_TRUE(std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]));
  EXPECT_EQ(cv::norm(p), 0.0);
}

// --- Reading a depth map at a corner -----------------------------------------

TEST(SampleDepth, TakesTheMedianOfThePatch)
{
  cv::Mat depth(20, 20, CV_32FC1, cv::Scalar(2.0f));
  // One outlier inside the window. A mean would move; a median must not.
  depth.at<float>(10, 10) = 2.05f;
  double metres = 0.0;
  ASSERT_TRUE(sample_depth(depth, 10, 10, 3, 0.2, 6.0, 0.1, metres));
  EXPECT_NEAR(metres, 2.0, 1e-6);
}

TEST(SampleDepth, RefusesAPatchStraddlingADepthStep)
{
  // The case this function exists for: ORB puts corners on occlusion edges, and a
  // single-pixel read there lands on either side of the step at random. Half the
  // window at 1 m, half at 4 m — a perfectly ordinary door frame.
  cv::Mat depth(20, 20, CV_32FC1, cv::Scalar(4.0f));
  depth.colRange(0, 10).setTo(1.0f);

  double metres = 0.0;
  EXPECT_FALSE(sample_depth(depth, 10, 10, 3, 0.2, 6.0, 0.1, metres))
    << "a corner on an occlusion edge must be refused, not averaged";

  // And five columns away, entirely on the near surface, it is fine.
  EXPECT_TRUE(sample_depth(depth, 5, 10, 3, 0.2, 6.0, 0.1, metres));
  EXPECT_NEAR(metres, 1.0, 1e-6);
}

TEST(SampleDepth, RefusesTheFrameEdgeRatherThanShrinkingTheWindow)
{
  cv::Mat depth(20, 20, CV_32FC1, cv::Scalar(2.0f));
  double metres = 0.0;
  EXPECT_FALSE(sample_depth(depth, 0, 10, 3, 0.2, 6.0, 0.1, metres));
  EXPECT_FALSE(sample_depth(depth, 19, 10, 3, 0.2, 6.0, 0.1, metres));
  EXPECT_TRUE(sample_depth(depth, 1, 10, 3, 0.2, 6.0, 0.1, metres));
}

TEST(SampleDepth, RefusesOutOfRangeAndNonFinite)
{
  cv::Mat far(20, 20, CV_32FC1, cv::Scalar(9.0f));
  cv::Mat nan(20, 20, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  cv::Mat infinite(20, 20, CV_32FC1, cv::Scalar(std::numeric_limits<float>::infinity()));
  double metres = 1234.0;
  EXPECT_FALSE(sample_depth(far, 10, 10, 3, 0.2, 6.0, 0.1, metres));
  EXPECT_FALSE(sample_depth(nan, 10, 10, 3, 0.2, 6.0, 0.1, metres));
  EXPECT_FALSE(sample_depth(infinite, 10, 10, 3, 0.2, 6.0, 0.1, metres));
  EXPECT_EQ(metres, 1234.0) << "a refusal must leave the output untouched";
}

TEST(SampleDepth, NeedsMoreThanHalfTheWindow)
{
  // Five of nine finite: usable. Four of nine: not. A corner mostly on the sky
  // gives readings only from the pixels nearest the boundary, which are the least
  // representative ones in the patch.
  cv::Mat depth(20, 20, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  double metres = 0.0;
  for (int i = 0; i < 4; ++i) {depth.at<float>(9 + i / 3, 9 + i % 3) = 2.0f;}
  EXPECT_FALSE(sample_depth(depth, 10, 10, 3, 0.2, 6.0, 0.1, metres));
  depth.at<float>(11, 11) = 2.0f;
  EXPECT_TRUE(sample_depth(depth, 10, 10, 3, 0.2, 6.0, 0.1, metres));
}

TEST(SampleDepth, RefusesAnythingThatIsNot32FC1)
{
  cv::Mat wrong(20, 20, CV_16UC1, cv::Scalar(2000));
  double metres = 0.0;
  EXPECT_FALSE(sample_depth(wrong, 10, 10, 3, 0.2, 6.0, 0.1, metres));
  EXPECT_FALSE(sample_depth(cv::Mat(), 10, 10, 3, 0.2, 6.0, 0.1, metres));
}

// --- The rigid fit ------------------------------------------------------------

TEST(FitRigid, RecoversAKnownRotationAndTranslation)
{
  const std::vector<cv::Vec3d> from = scene(60);
  const cv::Affine3d truth(rotation_about({0.2, 1.0, -0.3}, 0.25), cv::Vec3d(0.4, -0.1, 0.7));
  std::vector<cv::Vec3d> to;
  for (const cv::Vec3d & p : from) {to.push_back(cv::Vec3d(truth * p));}

  const RigidFit got = fit_rigid(from, to);
  EXPECT_LT(transform_error(got.motion(), truth), 1e-9);
  EXPECT_NEAR(got.scale, 1.0, 1e-9) << "two views of a rigid scene differ by no scale";
}

TEST(FitRigid, SeesPureTranslation)
{
  // The one thing fit_rotation() cannot do even in principle, and the whole reason
  // P7 needed a second estimator: bearing rays from a common origin have no
  // centroid to remove, so a translated scene looks to them like a rotated one.
  const std::vector<cv::Vec3d> from = scene(40);
  const cv::Affine3d truth(cv::Matx33d::eye(), cv::Vec3d(0.0, 0.0, 0.35));
  std::vector<cv::Vec3d> to;
  for (const cv::Vec3d & p : from) {to.push_back(cv::Vec3d(truth * p));}

  const RigidFit got = fit_rigid(from, to);
  EXPECT_LT(cv::norm(got.translation - cv::Vec3d(0.0, 0.0, 0.35)), 1e-9);
  EXPECT_LT(cv::norm(got.rotation - cv::Matx33d::eye()), 1e-9);
  EXPECT_NEAR(got.scale, 1.0, 1e-9)
    << "a forward translation is not a uniform scaling and must not be read as one";
}

TEST(FitRigid, NeverReturnsAReflection)
{
  // Every landmark on one wall, which is an ordinary thing to be looking at. A
  // degenerate cloud is where u*vt comes out with determinant -1 — an orthogonal
  // matrix that mirrors the room and passes every check but this one.
  std::vector<cv::Vec3d> from;
  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
      from.emplace_back(0.1 * i, 0.1 * j, 3.0);
    }
  }
  std::vector<cv::Vec3d> to = from;
  for (cv::Vec3d & p : to) {p[2] = 6.0 - p[2];}   // mirrored through the plane

  const RigidFit got = fit_rigid(from, to);
  EXPECT_GT(cv::determinant(got.rotation), 0.0) << "a rotation has determinant +1";
  EXPECT_NEAR(cv::determinant(got.rotation), 1.0, 1e-9);
  EXPECT_GT(got.scale, 0.0) << "and the scale carries the same sign correction";
}

TEST(FitRigidRobust, ThrowsOutOutliersAndKeepsTheAnswer)
{
  std::vector<cv::Vec3d> from = scene(80);
  const cv::Affine3d truth(rotation_about({0.0, 1.0, 0.0}, 0.08), cv::Vec3d(0.05, 0.0, 0.12));
  std::vector<cv::Vec3d> to;
  for (const cv::Vec3d & p : from) {to.push_back(cv::Vec3d(truth * p));}

  // Twelve of eighty landmarks with a depth reading off the far side of an
  // occlusion edge: metres out, not centimetres. A plain least-squares fit has no
  // defence at all.
  for (std::size_t i = 0; i < 12; ++i) {to[i * 6] += cv::Vec3d(0.0, 0.0, 2.5);}

  const auto naive = fit_rigid(from, to);
  const auto robust = fit_rigid_robust(from, to, 12, 0.05, 0.3, 2);
  ASSERT_TRUE(robust.ok);
  EXPECT_LT(transform_error(robust.motion(), truth), 1e-6);
  EXPECT_GT(transform_error(naive.motion(), truth), 0.1)
    << "if the naive fit also survives this, the outliers are not outliers";
}

TEST(FitRigidRobust, RefusesRatherThanGuessing)
{
  const std::vector<cv::Vec3d> from = scene(30);
  std::vector<cv::Vec3d> to;
  for (const cv::Vec3d & p : from) {to.push_back(p);}

  // Too few pairs. `ok` false means *hold the last pose*, which is the whole
  // contract: publishing identity here would say the camera stopped.
  EXPECT_FALSE(fit_rigid_robust({from[0], from[1]}, {to[0], to[1]}, 12).ok);
  EXPECT_FALSE(fit_rigid_robust(from, to, 100).ok);
  // Mismatched lengths: a caller bug, and a fit on the overlap would be worse than
  // a refusal because it would look like an answer.
  EXPECT_FALSE(fit_rigid_robust(from, {to.begin(), to.begin() + 10}, 4).ok);

  // Pure noise: no rigid transform explains it, and the residual gate says so.
  std::mt19937 gen(3);
  std::normal_distribution<double> jitter(0.0, 0.5);
  std::vector<cv::Vec3d> scrambled;
  for (const cv::Vec3d & p : from) {
    scrambled.emplace_back(p[0] + jitter(gen), p[1] + jitter(gen), p[2] + jitter(gen));
  }
  EXPECT_FALSE(fit_rigid_robust(from, scrambled, 12, 0.05, 0.3, 2).ok);
}

TEST(FitRigidRobust, NeverRejectsBelowItsOwnFloor)
{
  // Rejecting until a fit looks good is how a robust estimator becomes a way of
  // manufacturing agreement. With 14 pairs, a floor of 12 and a 30% reject
  // fraction, the first round would drop to 9 — so it must not happen at all.
  const std::vector<cv::Vec3d> from = scene(14);
  const cv::Affine3d truth(rotation_about({1.0, 0.0, 0.0}, 0.05), cv::Vec3d(0.0, 0.02, 0.0));
  std::vector<cv::Vec3d> to;
  for (const cv::Vec3d & p : from) {to.push_back(cv::Vec3d(truth * p));}

  const auto fit = fit_rigid_robust(from, to, 12, 0.05, 0.3, 3);
  EXPECT_EQ(fit.pairs_used, 14u);
  EXPECT_EQ(fit.refits, 0u);
  EXPECT_TRUE(fit.ok);
}

TEST(MeanResidual, IsMetresAndNotSquaredMetres)
{
  const std::vector<cv::Vec3d> from{{0, 0, 1}, {0, 0, 2}, {0, 0, 3}};
  std::vector<cv::Vec3d> to;
  for (const cv::Vec3d & p : from) {to.push_back(p + cv::Vec3d(0.0, 0.0, 0.1));}
  EXPECT_NEAR(mean_residual_m(RigidFit{}, from, to), 0.1, 1e-12);
}

// --- The composition, which is the suite that matters -------------------------

TEST(CameraStep, IsTheInverseOfThePointMotion)
{
  const cv::Affine3d motion(rotation_about({0.0, 1.0, 0.0}, 0.2), cv::Vec3d(0.1, 0.0, -0.3));
  EXPECT_LT(transform_error(camera_step(motion), motion.inv()), 1e-12);
  // And it is *not* the motion itself, which is the mistake it exists to name.
  EXPECT_GT(transform_error(camera_step(motion), motion), 0.1);
}

TEST(CameraStep, AWholeTrajectoryComesBackOutOfTheEstimator)
{
  // **The closed loop.** A camera panning right while sliding forward and a little
  // to its left — a hand-held sweep, in miniature. Everything the estimator does
  // in SixDof is here: unproject, fit, invert, change basis, compose.
  const std::vector<cv::Vec3d> world = scene(120);
  const cv::Affine3d basis(optical_basis(), cv::Vec3d(0.0, 0.0, 0.0));

  std::vector<cv::Affine3d> truth;
  for (int i = 0; i < 12; ++i) {
    const double t = 0.06 * i;
    truth.emplace_back(
      rotation_about({0.0, 1.0, 0.0}, t),                     // pan right
      cv::Vec3d(-0.05 * i, 0.0, 0.04 * i));                   // and drift
  }

  cv::Affine3d pose = truth.front() * basis.inv();            // odom <- base_link
  for (std::size_t i = 1; i < truth.size(); ++i) {
    const std::vector<cv::Vec3d> from = seen_from(truth[i - 1], world);
    const std::vector<cv::Vec3d> to = seen_from(truth[i], world);

    const auto fit = fit_rigid_robust(from, to, 12, 0.05, 0.3, 2);
    ASSERT_TRUE(fit.ok) << "step " << i;

    pose = pose * change_basis(basis, camera_step(fit.motion()));

    EXPECT_LT(transform_error(pose, truth[i] * basis.inv()), 1e-6)
      << "step " << i << ": the composed pose is not the trajectory that was simulated";
  }

  // The headline of P7, stated as an assertion rather than left implicit: the pose
  // that comes out has actually moved. A rotation-only run reports zero here on
  // exactly the same input.
  const double travelled = cv::norm(
    cv::Vec3d(pose.translation()) - cv::Vec3d((truth.front() * basis.inv()).translation()));
  EXPECT_GT(travelled, 0.5);
}

TEST(CameraStep, TheRotationOnlyPathComposesTheSameWayRound)
{
  // The same loop for P3's estimator, and the assertion that was missing while it
  // published a pose turning the wrong way. Bearing rays, one SVD, the same
  // inverse — and here the trap is at its most invisible, because the wrong answer
  // is a rotation of exactly the right size about exactly the right axis.
  const cv::Matx33d k = camera_k();
  const cv::Matx33d basis = optical_basis();
  const std::vector<cv::Vec3d> world = scene(90, 11);

  std::vector<cv::Matx33d> truth;
  for (int i = 0; i < 10; ++i) {truth.push_back(rotation_about({0.1, 1.0, 0.0}, 0.05 * i));}

  cv::Matx33d pose = truth.front() * basis.t();
  for (std::size_t i = 1; i < truth.size(); ++i) {
    std::vector<cv::Vec3d> from;
    std::vector<cv::Vec3d> to;
    for (const cv::Vec3d & point : world) {
      // Through K and back out again, so the pixel round trip is in the loop
      // rather than assumed away.
      const cv::Vec3d a = truth[i - 1].t() * point;
      const cv::Vec3d b = truth[i].t() * point;
      if (a[2] <= 0.1 || b[2] <= 0.1) {continue;}
      from.push_back(bearing(k, k(0, 0) * a[0] / a[2] + k(0, 2), k(1, 1) * a[1] / a[2] + k(1, 2)));
      to.push_back(bearing(k, k(0, 0) * b[0] / b[2] + k(0, 2), k(1, 1) * b[1] / b[2] + k(1, 2)));
    }
    ASSERT_GE(from.size(), 20u);

    const auto fit = fit_rotation_robust(from, to, 8, 0.03, 0.2);
    ASSERT_TRUE(fit.ok) << "step " << i;

    pose = pose * change_basis(basis, camera_step(fit.rotation));
    EXPECT_LT(cv::norm(pose - truth[i] * basis.t()), 1e-6) << "step " << i;
  }
}

TEST(CameraStep, PanningRightYawsRightAndNotLeft)
{
  // The same fact as the loop above, reduced to the one sentence somebody can
  // check against a real camera: pan right, features move left, the published
  // body frame yaws right. Body z is up, so a right-hand yaw to the right is a
  // *negative* rotation about z.
  const cv::Matx33d k = camera_k();
  const cv::Matx33d basis = optical_basis();
  const double angle = 0.15;
  // Optical y is down, so panning the camera to its right is +angle about y.
  const cv::Matx33d turned = rotation_about({0.0, 1.0, 0.0}, angle);

  std::vector<cv::Vec3d> from;
  std::vector<cv::Vec3d> to;
  for (const cv::Vec3d & point : scene(60, 5)) {
    const cv::Vec3d b = turned.t() * point;
    if (b[2] <= 0.1) {continue;}
    from.push_back(
      bearing(k, k(0, 0) * point[0] / point[2] + k(0, 2),
      k(1, 1) * point[1] / point[2] + k(1, 2)));
    to.push_back(bearing(k, k(0, 0) * b[0] / b[2] + k(0, 2), k(1, 1) * b[1] / b[2] + k(1, 2)));
  }
  ASSERT_GE(from.size(), 20u);

  // Features moved left in the image, which is what panning right looks like.
  double mean_shift = 0.0;
  for (std::size_t i = 0; i < from.size(); ++i) {mean_shift += to[i][0] - from[i][0];}
  EXPECT_LT(mean_shift, 0.0) << "the simulation itself is wrong if features moved right";

  const auto fit = fit_rotation_robust(from, to, 8, 0.03, 0.2);
  ASSERT_TRUE(fit.ok);
  const cv::Matx33d body = change_basis(basis, camera_step(fit.rotation));

  const cv::Vec3d axis(
    body(2, 1) - body(1, 2), body(0, 2) - body(2, 0), body(1, 0) - body(0, 1));
  const cv::Vec3d unit = cv::normalize(axis);
  EXPECT_NEAR(std::abs(unit[2]), 1.0, 1e-6) << "an optical pan must come out as body yaw";
  EXPECT_LT(unit[2], 0.0) << "panning right must yaw right; this is the P3 bug";
}

TEST(ChangeBasis, ALeverArmTurnsARotationIntoATranslation)
{
  // base_to_camera is identity today because the C922 is hand-held, and that is
  // precisely why this has to be right before anything is ever mounted: with a
  // zero offset the rotation-only overload and the full one agree exactly, so the
  // mistake is completely hidden.
  const cv::Affine3d mounted(optical_basis(), cv::Vec3d(0.10, 0.0, 0.05));
  const cv::Affine3d spin(rotation_about({0.0, 1.0, 0.0}, 0.3), cv::Vec3d(0.0, 0.0, 0.0));

  const cv::Affine3d body = change_basis(mounted, spin);
  EXPECT_GT(cv::norm(cv::Vec3d(body.translation())), 0.01)
    << "a camera swung about an offset does not stay in one place";

  // And with no offset it is exactly the rotation-only answer.
  const cv::Affine3d hand_held(optical_basis(), cv::Vec3d(0.0, 0.0, 0.0));
  const cv::Affine3d same = change_basis(hand_held, spin);
  EXPECT_LT(cv::norm(cv::Vec3d(same.translation())), 1e-12);
  EXPECT_LT(
    cv::norm(
      same.rotation() -
      pimesh_perception::change_basis(optical_basis(), spin.rotation())), 1e-12);
}

// --- The scale the depth network breathes ------------------------------------

TEST(ScaleHandling, DivideOutRemovesTheBreathingAndRigidAbsorbsItAsMotion)
{
  // **The measurement that decided P7's estimator, in miniature.** A camera that
  // has not moved at all, looking at a scene the depth network has rendered 4%
  // nearer than it did last frame. There is no motion to find here and both
  // answers are self-consistent; only one of them says so.
  const std::vector<cv::Vec3d> world = scene(80, 21);
  std::vector<cv::Vec3d> breathed;
  for (const cv::Vec3d & p : world) {breathed.push_back(p * 1.04);}

  const auto divided = fit_rigid_robust(
    world, breathed, 12, 0.05, 0.3, 2, ScaleHandling::DivideOut);
  ASSERT_TRUE(divided.ok);
  EXPECT_NEAR(divided.scale, 1.04, 1e-6) << "the breathing itself must be measurable";
  EXPECT_LT(cv::norm(divided.translation), 1e-6)
    << "a camera that did not move must not be told it did";

  const auto rigid = fit_rigid(world, breathed, ScaleHandling::Rigid);
  EXPECT_EQ(rigid.scale, 1.0);
  // What the rigid fit has to do instead: push the whole cloud by 4% of where its
  // centroid was, which at a ~3 m centroid is ~0.12 m of invented motion. Over a
  // 45 s clip at 17 Hz that is the 89.5 m path bags/desk1 measured.
  const double invented = cv::norm(rigid.translation);
  EXPECT_GT(invented, 0.05) << "if this is small the test scene has no depth to scale";
}

TEST(ScaleHandling, DividingOutTheScaleDoesNotEatRealTranslation)
{
  // The other half, and the one that makes DivideOut safe: a forward translation
  // is *not* a uniform scaling. Points at 1 m and at 5 m both move by the same
  // vector, so their ratio to the camera changes differently, and a similarity
  // fit can tell the two apart as long as the scene has depth to it.
  const std::vector<cv::Vec3d> from = scene(80, 31);
  const cv::Affine3d truth(cv::Matx33d::eye(), cv::Vec3d(0.0, 0.0, -0.25));
  std::vector<cv::Vec3d> to;
  for (const cv::Vec3d & p : from) {to.push_back(cv::Vec3d(truth * p));}

  const auto fit = fit_rigid_robust(from, to, 12, 0.05, 0.3, 2, ScaleHandling::DivideOut);
  ASSERT_TRUE(fit.ok);
  EXPECT_NEAR(fit.scale, 1.0, 0.02) << "real motion is not scale";
  EXPECT_LT(cv::norm(fit.translation - cv::Vec3d(0.0, 0.0, -0.25)), 0.02);
}

TEST(ScaleHandling, AFlatWallIsNotWhereTheTwoBecomeDegenerate)
{
  // **Written the other way round first, and measurement said no.** The obvious
  // worry about dividing the scale out is that approaching a flat wall *is* a
  // uniform scaling as far as the landmarks can tell. It is not, and the reason is
  // one line of arithmetic: a camera moving forward by d takes a landmark from
  // (X, Y, Z) to (X, Y, Z - d), leaving x and y **untouched**, where a scaling
  // takes it to (aX, aY, aZ) and shrinks the wall's lateral extent with it. The
  // two are separable on any patch that subtends a real angle, depth spread or no.
  std::vector<cv::Vec3d> from;
  for (int i = -10; i < 10; ++i) {
    for (int j = -10; j < 10; ++j) {
      from.emplace_back(0.2 * i, 0.15 * j, 3.0);   // one wall, one distance, full frame
    }
  }
  std::vector<cv::Vec3d> to;
  for (const cv::Vec3d & p : from) {to.push_back(p - cv::Vec3d(0.0, 0.0, 0.3));}

  const auto fit = fit_rigid_robust(from, to, 12, 0.05, 0.3, 2, ScaleHandling::DivideOut);
  ASSERT_TRUE(fit.ok);
  EXPECT_NEAR(fit.scale, 1.0, 0.01) << "approaching a wall is not the scene growing";
  EXPECT_LT(cv::norm(fit.translation - cv::Vec3d(0.0, 0.0, -0.3)), 0.01)
    << "and the motion survives dividing the scale out";
}

TEST(ScaleHandling, TheDegeneracyIsASmallPatchAtOneDistance)
{
  // Where it *does* come apart: a cluster of corners inside a ~5 degree cone, all
  // at the same distance. With no lateral extent to shrink and no depth spread to
  // disagree, scaling and approaching become nearly the same picture and the fit
  // splits the motion between them.
  //
  // **Measured 2026-09-19, and it is milder than a naive fit suggests**, which is
  // why the numbers are here rather than an adjective. Without the reject-worst
  // refits this geometry sends the scale to 0.75 and the translation to 0.44 m
  // against a true 0.30 m. *With* them — two rounds at 30% — the scale lands at
  // **0.961** and the translation at **0.424 m**, against the rigid fit's
  // 0.412 m. So the robust estimator absorbs most of it, the two answers are
  // within a centimetre of each other, and **both are 0.11 m wrong**, because a
  // patch that subtends nothing and has no depth carries almost no information
  // about motion in the first place.
  //
  // Recorded as a test rather than as a caveat because it is the one geometry
  // where `divide_out_depth_scale:=false` is the better answer, and because it is
  // narrow: ORB spreads its features across the frame, so reaching this takes a
  // camera close enough to a blank surface that only one small patch has corners
  // at all.
  std::mt19937 gen(9);
  std::normal_distribution<double> noise(0.0, 0.02);
  std::uniform_real_distribution<double> patch(-0.13, 0.13);

  std::vector<cv::Vec3d> from;
  std::vector<cv::Vec3d> to;
  for (int i = 0; i < 300; ++i) {
    const cv::Vec3d p(patch(gen), patch(gen), 3.0);
    from.push_back(p + cv::Vec3d(0.0, 0.0, p[2] * noise(gen)));
    const cv::Vec3d q = p - cv::Vec3d(0.0, 0.0, 0.3);
    to.push_back(q + cv::Vec3d(0.0, 0.0, q[2] * noise(gen)));
  }

  // The residual ceiling is off (5 m) on purpose: what is under test is which
  // answer the fit gives, not whether the pose gate would have refused it. In the
  // running node it would — this is what `regime=hold` is for.
  const auto divided = fit_rigid_robust(from, to, 12, 5.0, 0.3, 2, ScaleHandling::DivideOut);
  const auto rigid = fit_rigid_robust(from, to, 12, 5.0, 0.3, 2, ScaleHandling::Rigid);
  ASSERT_TRUE(divided.ok);
  ASSERT_TRUE(rigid.ok);

  const double truth = 0.3;
  EXPECT_LT(divided.scale, 0.98)
    << "if the scale stays at 1 this geometry is not degenerate and the test is wrong";
  EXPECT_GT(
    std::abs(cv::norm(divided.translation) - truth),
    std::abs(cv::norm(rigid.translation) - truth))
    << "on this one geometry the rigid fit is the better of the two";
  EXPECT_LT(
    std::abs(cv::norm(divided.translation) - cv::norm(rigid.translation)), 0.05)
    << "and it is better by a centimetre, not by the factor an unrobust fit shows";
}

TEST(ScaleHandling, TheResidualIsMeasuredAgainstTheModelThatWasFitted)
{
  // A residual computed without the scale would report the breathing as error —
  // ~0.12 m on the scene above — and the pose gate would refuse every frame of a
  // perfectly good run. This is the difference between fit.map() and fit.motion().
  const std::vector<cv::Vec3d> world = scene(60, 41);
  std::vector<cv::Vec3d> breathed;
  for (const cv::Vec3d & p : world) {breathed.push_back(p * 1.04);}

  const auto fit = fit_rigid_robust(
    world, breathed, 12, 0.05, 0.3, 2, ScaleHandling::DivideOut);
  EXPECT_LT(fit.residual_m, 1e-6);
  EXPECT_GT(mean_residual_m(RigidFit{fit.ok, fit.rotation, fit.translation, 0.0, 1.0,
      fit.pairs_in, fit.pairs_used, fit.refits}, world, breathed), 0.05)
    << "ignoring the scale in the residual would fail this fit";
}
