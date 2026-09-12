// The geometry, against rotations whose answer is known.
//
// This is the suite that matters most in the package, because none of what it
// covers fails loudly. A rotation fit given mismatched pairs returns an orthogonal
// matrix with a believable residual. A missing determinant correction returns a
// *mirror*, which is orthogonal too. A basis change applied the wrong way round
// turns a yaw into a roll, and the TF frame still moves when you pan the camera —
// just about the wrong axis, which looks like a mounting error several milestones
// later.

#include <cmath>
#include <vector>

#include "gtest/gtest.h"
#include "opencv2/core.hpp"
#include "pimesh_perception/rotation_fit.hpp"

using pimesh_perception::bearing;
using pimesh_perception::change_basis;
using pimesh_perception::fit_rotation;
using pimesh_perception::fit_rotation_robust;
using pimesh_perception::mean_residual_rad;
using pimesh_perception::quaternion_from_rotation;

namespace
{

/// This camera's real intrinsics at 720p (P9, 2026-09-12). Using the measured ones
/// rather than round numbers keeps the residuals in this file comparable with the
/// ones the node logs.
const cv::Matx33d kC922(953.4, 0.0, 627.7, 0.0, 957.6, 334.6, 0.0, 0.0, 1.0);

cv::Matx33d rotation_about(const cv::Vec3d & axis, double radians)
{
  const cv::Vec3d a = cv::normalize(axis);
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  const double t = 1.0 - c;
  return cv::Matx33d(
    t * a[0] * a[0] + c, t * a[0] * a[1] - s * a[2], t * a[0] * a[2] + s * a[1],
    t * a[0] * a[1] + s * a[2], t * a[1] * a[1] + c, t * a[1] * a[2] - s * a[0],
    t * a[0] * a[2] - s * a[1], t * a[1] * a[2] + s * a[0], t * a[2] * a[2] + c);
}

/// Rays spread over the frame, the way ORB corners are — not clustered at the
/// centre, where almost any rotation fits almost any motion.
std::vector<cv::Vec3d> spread_rays(int count = 60)
{
  std::vector<cv::Vec3d> rays;
  for (int i = 0; i < count; ++i) {
    const double u = 40.0 + (1200.0 * (i % 10)) / 9.0;
    const double v = 30.0 + (660.0 * (i / 10)) / 5.0;
    rays.push_back(bearing(kC922, u, v));
  }
  return rays;
}

std::vector<cv::Vec3d> rotate_all(const cv::Matx33d & r, const std::vector<cv::Vec3d> & rays)
{
  std::vector<cv::Vec3d> out;
  out.reserve(rays.size());
  for (const cv::Vec3d & ray : rays) {out.push_back(r * ray);}
  return out;
}

/// How close two rotations have to be before a test calls them equal, and why it
/// is not 1e-12.
///
/// Both this file's comparisons and the residual itself end in an `acos` of
/// something very near 1, and `acos` is where half the digits go: its derivative is
/// unbounded at 1, so an argument accurate to 1e-16 yields an angle accurate to
/// about sqrt(1e-16) = 1e-8. Measured here, an exactly-recovered rotation comes back
/// with a residual of 2e-8 rather than 0. That is the arithmetic being honest, not
/// the fit being loose — and it is worth knowing before reading a residual of 1e-8
/// in a log as a real disagreement.
constexpr double kAngleEps = 1e-6;

double angle_between(const cv::Matx33d & a, const cv::Matx33d & b)
{
  const cv::Matx33d d = a.t() * b;
  const double trace = d(0, 0) + d(1, 1) + d(2, 2);
  return std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
}

}  // namespace

TEST(Bearing, IsAUnitRayThroughThePixel)
{
  // The principal point maps to the optical axis exactly. If this is off, every
  // residual in the pipeline carries a constant bias that looks like miscalibration.
  const cv::Vec3d axis = bearing(kC922, 627.7, 334.6);
  EXPECT_NEAR(axis[0], 0.0, 1e-12);
  EXPECT_NEAR(axis[1], 0.0, 1e-12);
  EXPECT_NEAR(axis[2], 1.0, 1e-12);

  const cv::Vec3d ray = bearing(kC922, 1000.0, 100.0);
  EXPECT_NEAR(cv::norm(ray), 1.0, 1e-12);
  // Right of centre and above it: +x, -y in the optical convention.
  EXPECT_GT(ray[0], 0.0);
  EXPECT_LT(ray[1], 0.0);
  EXPECT_GT(ray[2], 0.0);
}

TEST(Bearing, FocalLengthSetsTheAngleScale)
{
  // One pixel off-axis is 1/fx radians, which is the conversion every residual in
  // this pipeline is quoted in. At fx=953, 0.03 rad is ~28 px.
  const cv::Vec3d one_px = bearing(kC922, 628.7, 334.6);
  EXPECT_NEAR(std::atan2(one_px[0], one_px[2]), 1.0 / 953.4, 1e-9);
}

TEST(Bearing, RefusesAZeroFocalLength)
{
  // An all-zero K is what a CameraInfo that never arrived looks like. Dividing by
  // it gives NaN, and a NaN ray poisons the SVD and every gate downstream reading
  // the residual — the pose would simply stop updating with no number to explain it.
  const cv::Matx33d empty = cv::Matx33d::zeros();
  const cv::Vec3d ray = bearing(empty, 100.0, 100.0);
  EXPECT_DOUBLE_EQ(ray[2], 1.0);
  EXPECT_FALSE(std::isnan(ray[0]));
}

TEST(FitRotation, RecoversAKnownRotationExactly)
{
  const auto from = spread_rays();
  for (double degrees : {0.2, 1.0, 5.0, 20.0}) {
    const cv::Matx33d truth = rotation_about({0.1, 1.0, 0.2}, degrees * CV_PI / 180.0);
    const cv::Matx33d fitted = fit_rotation(from, rotate_all(truth, from));
    EXPECT_LT(angle_between(truth, fitted), kAngleEps) << degrees << " degrees";
  }
}

TEST(FitRotation, IsARotationAndNotAReflection)
{
  const auto from = spread_rays();
  const cv::Matx33d truth = rotation_about({0.0, 1.0, 0.0}, 0.1);
  const cv::Matx33d fitted = fit_rotation(from, rotate_all(truth, from));

  EXPECT_NEAR(cv::determinant(fitted), 1.0, 1e-9);
  const cv::Matx33d should_be_identity = fitted * fitted.t();
  EXPECT_NEAR(cv::norm(should_be_identity - cv::Matx33d::eye()), 0.0, 1e-9);
}

TEST(FitRotation, DegeneratePairsDoNotProduceAMirror)
{
  // Every pair identical: the fit is unconstrained in two directions, which is
  // exactly where u*vt comes back with determinant -1. Without the correction this
  // returns a mirror — orthogonal, plausible, and it flips the world.
  std::vector<cv::Vec3d> from(10, cv::Vec3d(0.0, 0.0, 1.0));
  std::vector<cv::Vec3d> to(10, cv::Vec3d(0.0, 0.0, 1.0));
  const cv::Matx33d fitted = fit_rotation(from, to);
  EXPECT_GT(cv::determinant(fitted), 0.0);
}

TEST(FitRotation, RefusesTooFewPairs)
{
  std::vector<cv::Vec3d> from {cv::Vec3d(0, 0, 1), cv::Vec3d(0, 1, 0)};
  EXPECT_NEAR(cv::norm(fit_rotation(from, from) - cv::Matx33d::eye()), 0.0, 1e-12);
}

TEST(MeanResidual, IsZeroOnAPerfectFit)
{
  const auto from = spread_rays();
  const cv::Matx33d truth = rotation_about({0.0, 0.0, 1.0}, 0.05);
  EXPECT_NEAR(mean_residual_rad(truth, from, rotate_all(truth, from)), 0.0, kAngleEps);
}

TEST(MeanResidual, IsTheRotationAngleForRaysAcrossTheAxisOfRotation)
{
  // A pan — rotation about the optical frame's y axis — measured on rays along the
  // image's horizontal centre line, which all lie in the plane the pan turns in. For
  // those, the angle between a ray and its rotated self *is* the pan angle, which is
  // what makes the residual readable as "how far out the fit is, in radians".
  std::vector<cv::Vec3d> from;
  for (int i = 0; i < 20; ++i) {
    from.push_back(bearing(kC922, 100.0 + 55.0 * i, 334.6));
  }
  const cv::Matx33d pan = rotation_about({0.0, 1.0, 0.0}, 0.05);
  EXPECT_NEAR(mean_residual_rad(cv::Matx33d::eye(), from, rotate_all(pan, from)), 0.05, 1e-3);
}

TEST(MeanResidual, IsMuchSmallerThanTheRotationForARollAboutTheOpticalAxis)
{
  // The same 0.05 rad, applied about z instead — a roll — and the ray residual comes
  // out about 0.019 rad, a third of it. This is not a defect in the measure; it is
  // geometry: a ray *on* the axis of rotation does not move at all, and the rays ORB
  // finds are clustered within ~35 degrees of the optical axis.
  //
  // **The consequence is a real limitation of P3's pose gate**, and it is better
  // written down than discovered later: a residual ceiling of 0.03 rad rejects a
  // 0.03 rad error in pan or tilt but tolerates roughly three times that much error
  // in roll. The gate is not equally sharp in all three axes, and a rolled hand-held
  // sweep is exactly the case where it is least sharp.
  const auto from = spread_rays();
  const cv::Matx33d roll = rotation_about({0.0, 0.0, 1.0}, 0.05);
  const double residual = mean_residual_rad(cv::Matx33d::eye(), from, rotate_all(roll, from));
  EXPECT_GT(residual, 0.0);
  EXPECT_LT(residual, 0.5 * 0.05);
}

TEST(MeanResidual, SurvivesADotProductJustOverOne)
{
  // acos(1.0000000002) is NaN, and a NaN residual fails the gate silently in the
  // wrong direction: `NaN < ceiling` is false, so the pose would hold forever with
  // no number in the log to explain why.
  std::vector<cv::Vec3d> rays(20, cv::Vec3d(0.0, 0.0, 1.0));
  const double residual = mean_residual_rad(cv::Matx33d::eye(), rays, rays);
  EXPECT_FALSE(std::isnan(residual));
  EXPECT_NEAR(residual, 0.0, 1e-12);
}

TEST(RobustFit, PassesItsGatesOnCleanPairs)
{
  const auto from = spread_rays();
  const cv::Matx33d truth = rotation_about({0.2, 1.0, 0.0}, 0.01);
  const auto fit = fit_rotation_robust(from, rotate_all(truth, from));

  EXPECT_TRUE(fit.ok);
  EXPECT_LT(fit.residual_rad, kAngleEps);
  EXPECT_EQ(fit.pairs_in, from.size());
  EXPECT_LT(angle_between(truth, fit.rotation), kAngleEps);
}

TEST(RobustFit, RejectsOutliersInsteadOfAveragingThem)
{
  auto from = spread_rays();
  const cv::Matx33d truth = rotation_about({0.0, 1.0, 0.0}, 0.02);
  auto to = rotate_all(truth, from);

  // Six pairs in sixty pointing somewhere else entirely — a repeated corner on a
  // window frame, a reflection in a screen. Least squares has no defence against
  // these; the reject-worst refits do.
  for (int i = 0; i < 6; ++i) {
    to[static_cast<std::size_t>(i * 9)] = rotation_about({1.0, 0.0, 0.0}, 0.6) * from[
      static_cast<std::size_t>(i * 9)];
  }

  const cv::Matx33d naive = fit_rotation(from, to);
  const auto robust = fit_rotation_robust(from, to);

  EXPECT_TRUE(robust.ok);
  EXPECT_LT(angle_between(truth, robust.rotation), angle_between(truth, naive))
    << "rejection did not improve on the plain least-squares fit";
  EXPECT_LT(angle_between(truth, robust.rotation), kAngleEps);
  EXPECT_LT(robust.pairs_used, robust.pairs_in);
}

TEST(RobustFit, HoldsRatherThanGuessWhenThereAreTooFewPairs)
{
  const auto from = spread_rays(7);   // one short of the default floor of 8
  const auto fit = fit_rotation_robust(from, from);
  EXPECT_FALSE(fit.ok) << "a fit on 7 pairs must not be trusted at min_pairs=8";
  EXPECT_EQ(fit.pairs_used, 0u);
  // Identity is what the struct defaults to, and the caller must not publish it:
  // ok == false means hold the last pose, not publish this one.
  EXPECT_NEAR(cv::norm(fit.rotation - cv::Matx33d::eye()), 0.0, 1e-12);
}

TEST(RobustFit, HoldsWhenTheResidualIsTooLarge)
{
  // Pairs related by no *single* rotation: each ray is turned by a different
  // 0.1 rad, which is what a frame full of mismatched corners looks like. The fit
  // will return something; the gate is what stops it being believed.
  //
  // **The obvious fixture for this does not work, and the reason is worth keeping.**
  // Reversing the pairing of a symmetric grid of rays looks like nonsense and is
  // not: point-reflecting a centred grid through the optical axis *is* a rotation —
  // 180 degrees about z — so the fit recovered it with a residual of 0.0055 rad and
  // passed its gates, correctly. A test that a robust estimator rejects bad data
  // has to supply data that is genuinely not a rotation.
  auto from = spread_rays(40);
  std::vector<cv::Vec3d> to;
  cv::RNG rng(20260912);
  for (std::size_t i = 0; i < from.size(); ++i) {
    const cv::Vec3d axis(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0));
    to.push_back(rotation_about(axis, 0.1) * from[i]);
  }

  const auto fit = fit_rotation_robust(from, to);
  EXPECT_FALSE(fit.ok);
  EXPECT_GT(fit.residual_rad, 0.03);
}

TEST(RobustFit, NeverRejectsBelowItsOwnFloor)
{
  // With reject_fraction high and a 10-pair input, an unguarded loop would keep
  // discarding until the fit looked perfect on three pairs. That is not a robust
  // estimator, it is a way of manufacturing agreement.
  const auto from = spread_rays(10);
  const cv::Matx33d truth = rotation_about({0.0, 1.0, 0.0}, 0.01);
  const auto fit = fit_rotation_robust(from, rotate_all(truth, from), 8, 0.03, 0.5, 5);
  EXPECT_GE(fit.pairs_used, 8u);
}

TEST(ChangeBasis, MovesTheAxisAndKeepsTheAngle)
{
  // camera_link -> camera_optical_frame, the quaternion this project publishes once
  // as a static transform: (-0.5, 0.5, -0.5, 0.5). Inverted here to get
  // base_link <- optical, since base_to_camera is identity.
  const cv::Matx33d basis(
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0);
  ASSERT_NEAR(cv::determinant(basis), 1.0, 1e-12);

  // A pan of the camera is a rotation about the optical frame's *y* (down) axis.
  // In the body frame that must come out as yaw — about z (up) — and with the
  // opposite sign, because optical y points down where body z points up.
  const double angle = 0.1;
  const cv::Matx33d optical_yaw = rotation_about({0.0, 1.0, 0.0}, angle);
  const cv::Matx33d body = change_basis(basis, optical_yaw);

  EXPECT_NEAR(angle_between(cv::Matx33d::eye(), body), angle, 1e-12)
    << "a change of basis must not change how much rotation there is";

  // The body rotation's axis, read off the antisymmetric part.
  const cv::Vec3d axis(body(2, 1) - body(1, 2), body(0, 2) - body(2, 0), body(1, 0) - body(0, 1));
  const cv::Vec3d unit = cv::normalize(axis);
  EXPECT_NEAR(std::abs(unit[2]), 1.0, 1e-9) << "optical pan did not become body yaw";
  EXPECT_NEAR(unit[2], -1.0, 1e-9) << "the sign is wrong: panning left would yaw right";
}

TEST(ChangeBasis, IdentityBasisChangesNothing)
{
  const cv::Matx33d r = rotation_about({1.0, 2.0, 3.0}, 0.3);
  EXPECT_NEAR(cv::norm(change_basis(cv::Matx33d::eye(), r) - r), 0.0, 1e-12);
}

TEST(Quaternion, RoundTripsThroughARotationMatrix)
{
  for (double degrees : {0.0, 1.0, 45.0, 90.0, 179.0, 179.99}) {
    const cv::Matx33d truth = rotation_about({0.3, -0.5, 0.8}, degrees * CV_PI / 180.0);
    const cv::Vec4d q = quaternion_from_rotation(truth);
    EXPECT_NEAR(cv::norm(q), 1.0, 1e-12) << degrees;

    // Rebuild the matrix from the quaternion and compare. The 179-degree cases are
    // the point of this test: the textbook w = sqrt(1 + trace)/2 branch loses all
    // its precision there and takes the root of a small negative number at 180.
    const double x = q[0];
    const double y = q[1];
    const double z = q[2];
    const double w = q[3];
    const cv::Matx33d back(
      1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y));
    EXPECT_LT(angle_between(truth, back), 1e-7) << degrees << " degrees";
  }
}

TEST(Quaternion, IdentityIsWOne)
{
  const cv::Vec4d q = quaternion_from_rotation(cv::Matx33d::eye());
  EXPECT_NEAR(q[3], 1.0, 1e-12);
  EXPECT_NEAR(q[0], 0.0, 1e-12);
  EXPECT_NEAR(q[1], 0.0, 1e-12);
  EXPECT_NEAR(q[2], 0.0, 1e-12);
}
