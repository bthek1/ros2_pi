// The rotation core, driven by synthetic ray bundles.
//
// This is the file that justified pulling the geometry out of the node. A
// rotation estimator inside a subscription callback can only be tested by
// pointing a camera at a room and squinting at RViz; as free functions over
// ray bundles it can be handed a KNOWN rotation and asked to find it back,
// which is a real test with a real answer.
//
// Nothing here needs a camera, the Pi, the network, or a ROS context.

#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "pimesh_perception/rotation.hpp"

using pimesh_perception::CameraMatrix;
using pimesh_perception::Rays;
using pimesh_perception::RotationGates;
using pimesh_perception::RotationReject;

namespace
{

// The predecessor's approximate C922 intrinsics at 720p, used here as a
// plausible K for synthetic data only. The real camera publishes zeros.
constexpr CameraMatrix kApproxK{907.0, 0.0, 640.0, 0.0, 907.0, 360.0, 0.0, 0.0, 1.0};

/// A repeatable bundle of unit rays spread over a camera's field of view.
Rays sample_rays(int count, unsigned seed = 7)
{
  std::mt19937 rng(seed);
  // +/- 0.5 in normalised image coordinates is roughly a 53 degree FOV — the
  // C922's, near enough for a bundle that has to be well spread rather than
  // physically exact.
  std::uniform_real_distribution<double> spread(-0.5, 0.5);
  Rays rays(3, count);
  for (int i = 0; i < count; ++i) {
    rays.col(i) = Eigen::Vector3d(spread(rng), spread(rng), 1.0).normalized();
  }
  return rays;
}

Eigen::Matrix3d rotation_about(const Eigen::Vector3d & axis, double radians)
{
  return Eigen::AngleAxisd(radians, axis.normalized()).toRotationMatrix();
}

}  // namespace

// ---------------------------------------------------------------- rays ----

TEST(RaysFromPixels, PrincipalPointLooksStraightAhead)
{
  // The pixel at (cx, cy) is by definition on the optical axis, so its ray is
  // +Z exactly. If this ever fails, cx/cy have been read out of the wrong slots
  // of K — the failure mode that puts a small constant bias into every pose.
  const std::vector<cv::Point2f> pixels{cv::Point2f(640.0f, 360.0f)};
  const Rays rays = pimesh_perception::rays_from_pixels(pixels, kApproxK);

  ASSERT_EQ(rays.cols(), 1);
  EXPECT_NEAR(rays(0, 0), 0.0, 1e-12);
  EXPECT_NEAR(rays(1, 0), 0.0, 1e-12);
  EXPECT_NEAR(rays(2, 0), 1.0, 1e-12);
}

TEST(RaysFromPixels, RaysAreUnitLengthAndPointForward)
{
  const std::vector<cv::Point2f> pixels{
    cv::Point2f(0.0f, 0.0f), cv::Point2f(1279.0f, 719.0f), cv::Point2f(300.0f, 500.0f)};
  const Rays rays = pimesh_perception::rays_from_pixels(pixels, kApproxK);

  for (Eigen::Index i = 0; i < rays.cols(); ++i) {
    EXPECT_NEAR(rays.col(i).norm(), 1.0, 1e-12);
    // Everything the camera can see is in front of it. A negative Z would mean
    // the unprojection flipped, which no amount of downstream fitting recovers.
    EXPECT_GT(rays(2, i), 0.0);
  }
}

TEST(RaysFromPixels, MovingRightTiltsTheRayRight)
{
  const std::vector<cv::Point2f> pixels{
    cv::Point2f(640.0f, 360.0f), cv::Point2f(740.0f, 360.0f)};
  const Rays rays = pimesh_perception::rays_from_pixels(pixels, kApproxK);
  EXPECT_GT(rays(0, 1), rays(0, 0));
}

TEST(IsCalibrated, ZeroFocalLengthIsTheUncalibratedSignal)
{
  // camera_node publishes K all zeros and warns. Detecting that is what keeps
  // the odometer honest instead of confidently wrong.
  EXPECT_FALSE(pimesh_perception::is_calibrated(CameraMatrix{}));
  EXPECT_TRUE(pimesh_perception::is_calibrated(kApproxK));

  CameraMatrix half_zero = kApproxK;
  half_zero[4] = 0.0;   // fy missing
  EXPECT_FALSE(pimesh_perception::is_calibrated(half_zero));
}

// -------------------------------------------------------------- kabsch ----

TEST(Kabsch, RecoversAKnownRotationExactly)
{
  const Rays prev = sample_rays(50);
  const Eigen::Matrix3d truth = rotation_about(Eigen::Vector3d(0.2, 1.0, -0.3), 0.09);
  const Rays curr = truth * prev;

  const Eigen::Matrix3d found = pimesh_perception::kabsch(prev, curr);
  EXPECT_LT((found - truth).norm(), 1e-9);
}

TEST(Kabsch, NeverReturnsAReflection)
{
  // For cleanly rotated data the SVD cannot reflect — det(P * (R P)^T) is
  // always positive — so a test built from a rotation proves nothing about the
  // determinant guard. The guard fires on data whose best-fit ORTHOGONAL
  // transform genuinely is a reflection: a degenerate ray bundle plus a few
  // false matches can produce exactly that, and the estimator must answer with
  // the nearest real rotation rather than a motion no camera can perform.
  //
  // Reflecting the bundle through a plane is the deterministic way to build
  // that case.
  const Rays prev = sample_rays(40);
  Eigen::Matrix3d mirror = Eigen::Matrix3d::Identity();
  mirror(0, 0) = -1.0;                    // flip x: det = -1
  const Rays curr = mirror * prev;

  const Eigen::Matrix3d found = pimesh_perception::kabsch(prev, curr);
  EXPECT_NEAR(found.determinant(), 1.0, 1e-9) << "kabsch returned a reflection";
  EXPECT_LT((found * found.transpose() - Eigen::Matrix3d::Identity()).norm(), 1e-9);
}

TEST(Kabsch, HandlesANearDegenerateBundleWithoutFallingApart)
{
  // A flat wall filling the frame: the rays are nearly coplanar, so the
  // cross-covariance is close to rank-deficient and its smallest singular
  // value is decided by noise. The fit is allowed to be imprecise here; what
  // it may never be is a non-rotation.
  Rays prev(3, 12);
  for (Eigen::Index i = 0; i < prev.cols(); ++i) {
    prev.col(i) = Eigen::Vector3d(0.0, -0.4 + 0.07 * static_cast<double>(i), 1.0).normalized();
  }
  const Eigen::Matrix3d truth = rotation_about(Eigen::Vector3d::UnitX(), 0.05);
  const Rays curr = truth * prev;

  const Eigen::Matrix3d found = pimesh_perception::kabsch(prev, curr);
  EXPECT_NEAR(found.determinant(), 1.0, 1e-9);
  EXPECT_LT((found * found.transpose() - Eigen::Matrix3d::Identity()).norm(), 1e-9);
}

TEST(ResidualAngles, AreZeroForAPerfectFitAndPositiveOtherwise)
{
  const Rays prev = sample_rays(20);
  const Eigen::Matrix3d truth = rotation_about(Eigen::Vector3d::UnitY(), 0.04);
  const Rays curr = truth * prev;

  const Eigen::VectorXd exact = pimesh_perception::residual_angles(truth, prev, curr);
  EXPECT_LT(exact.maxCoeff(), 1e-7);

  // A deliberately wrong rotation must show up as roughly the angle it is out
  // by — the residual is the quantity the pose gate thresholds on, so it has
  // to mean something.
  const Eigen::Matrix3d wrong = rotation_about(Eigen::Vector3d::UnitY(), 0.14);
  const Eigen::VectorXd off = pimesh_perception::residual_angles(wrong, prev, curr);
  EXPECT_NEAR(off.mean(), 0.10, 0.02);
}

// ---------------------------------------------------- estimate_rotation ----

TEST(EstimateRotation, AcceptsACleanBundleAndFindsTheRotation)
{
  const Rays prev = sample_rays(200);
  const Eigen::Matrix3d truth = rotation_about(Eigen::Vector3d(0.1, 1.0, 0.05), 0.06);
  const Rays curr = truth * prev;

  const auto estimate = pimesh_perception::estimate_rotation(prev, curr);
  ASSERT_TRUE(estimate.ok);
  EXPECT_EQ(estimate.reject, RotationReject::kNone);
  EXPECT_LT((estimate.rotation - truth).norm(), 1e-8);
  EXPECT_LT(estimate.mean_residual_rad, 1e-8);
  EXPECT_EQ(estimate.pairs_in, 200u);
}

TEST(EstimateRotation, RejectsTooFewPairsRatherThanFittingNoise)
{
  const Rays prev = sample_rays(4);
  const Eigen::Matrix3d truth = rotation_about(Eigen::Vector3d::UnitZ(), 0.02);
  const Rays curr = truth * prev;

  // Three pairs determine a rotation, so this WOULD produce an answer. The
  // gate refuses it anyway: at four pairs one bad match is 25% of the data.
  const auto estimate = pimesh_perception::estimate_rotation(prev, curr);
  EXPECT_FALSE(estimate.ok);
  EXPECT_EQ(estimate.reject, RotationReject::kTooFewPairs);
  EXPECT_EQ(estimate.pairs_used, 0u);
}

TEST(EstimateRotation, RefitDropsAFewOutliersAndStillFindsTheTruth)
{
  Rays prev = sample_rays(60);
  const Eigen::Matrix3d truth = rotation_about(Eigen::Vector3d::UnitX(), 0.05);
  Rays curr = truth * prev;

  // Six of sixty pairs are false matches — 10%, which is worse than descriptor
  // matching with crossCheck actually delivers. They land in the residual tail
  // and the reject-worst-and-refit rounds should remove them.
  std::mt19937 rng(11);
  std::uniform_real_distribution<double> spread(-0.5, 0.5);
  for (int i = 0; i < 6; ++i) {
    curr.col(i * 7) = Eigen::Vector3d(spread(rng), spread(rng), 1.0).normalized();
  }

  const auto estimate = pimesh_perception::estimate_rotation(prev, curr);
  ASSERT_TRUE(estimate.ok) << "reject: " << pimesh_perception::to_string(estimate.reject);
  EXPECT_LT(pimesh_perception::rotation_angle(estimate.rotation * truth.transpose()), 0.005);
  EXPECT_LT(estimate.pairs_used, estimate.pairs_in);
}

TEST(EstimateRotation, RejectsABundleThatAgreesOnNoRotation)
{
  // Every pair is garbage: this is what a frame across a motion blur, or a
  // scene where everything moved independently, actually looks like. Returning
  // a confident rotation here is the failure that smears a mesh, so the gate
  // must fire.
  const Rays prev = sample_rays(80, 3);
  const Rays curr = sample_rays(80, 4);

  const auto estimate = pimesh_perception::estimate_rotation(prev, curr);
  EXPECT_FALSE(estimate.ok);
  EXPECT_EQ(estimate.reject, RotationReject::kResidual);
  EXPECT_GT(estimate.mean_residual_rad, 0.03);
}

TEST(EstimateRotation, MismatchedBundleSizesAreRejectedNotUndefined)
{
  const auto estimate =
    pimesh_perception::estimate_rotation(sample_rays(30), sample_rays(20));
  EXPECT_FALSE(estimate.ok);
  EXPECT_EQ(estimate.reject, RotationReject::kTooFewPairs);
}

TEST(EstimateRotation, GatesAreConfigurable)
{
  const Rays prev = sample_rays(80, 3);
  const Rays curr = sample_rays(80, 4);

  RotationGates loose;
  loose.max_residual_rad = 3.14;   // accept anything
  const auto estimate = pimesh_perception::estimate_rotation(prev, curr, loose);
  EXPECT_TRUE(estimate.ok);
}

// ------------------------------------------------------------- helpers ----

TEST(RotationAngle, MeasuresTheAxisAngleMagnitude)
{
  EXPECT_NEAR(pimesh_perception::rotation_angle(Eigen::Matrix3d::Identity()), 0.0, 1e-12);
  const auto r = rotation_about(Eigen::Vector3d(1.0, 2.0, 3.0), 0.37);
  EXPECT_NEAR(pimesh_perception::rotation_angle(r), 0.37, 1e-9);
}

TEST(Orthonormalize, RepairsDriftWithoutMovingACleanRotation)
{
  const auto clean = rotation_about(Eigen::Vector3d::UnitZ(), 0.3);
  EXPECT_LT((pimesh_perception::orthonormalize(clean) - clean).norm(), 1e-12);

  // Composing thousands of per-frame rotations accumulates exactly this kind
  // of error, and a not-quite-rotation is a slightly-scaling transform that
  // would quietly grow the map.
  Eigen::Matrix3d drifted = clean;
  drifted(0, 1) += 1e-4;
  drifted(2, 2) -= 2e-4;
  const auto fixed = pimesh_perception::orthonormalize(drifted);
  EXPECT_LT((fixed * fixed.transpose() - Eigen::Matrix3d::Identity()).norm(), 1e-12);
  EXPECT_NEAR(fixed.determinant(), 1.0, 1e-12);
}
