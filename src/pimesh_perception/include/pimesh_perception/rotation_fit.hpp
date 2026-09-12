#ifndef PIMESH_PERCEPTION__ROTATION_FIT_HPP_
#define PIMESH_PERCEPTION__ROTATION_FIT_HPP_

#include <cstddef>
#include <vector>

#include "opencv2/core.hpp"

namespace pimesh_perception
{

/// The unit bearing ray through a pixel, in the optical frame.
///
/// `(x, y)` is a pixel, `K` the 3x3 intrinsic matrix from `/camera_info`. The ray
/// is `normalise(K^-1 [x y 1])`, written out rather than done with a matrix
/// inverse because the form is so simple that the inverse obscures it.
///
/// **This is where the calibration enters the geometry**, and the only place it
/// does in P3. With nominal intrinsics the rays are wrong in a way that grows
/// towards the frame edges — where plenty of ORB corners live — and the error
/// lands directly in the residual the pose gate is checked against. P9 is
/// upstream of this function for that reason: fx=953.4, fy=957.6, cx=627.7,
/// cy=334.6 on this camera at 720p, held-out reprojection 0.4955 px.
cv::Vec3d bearing(const cv::Matx33d & k, double x, double y);

/// What a rotation fit came back with, and whether it should be believed.
struct RotationFit
{
  /// True only if every gate passed. A false here means **hold the last pose** —
  /// not publish this rotation, and not publish identity either. A wrong pose is
  /// worse than a stale one: the mesh folds around it and there is nothing in the
  /// output that says which frames were guesses.
  bool ok {false};
  cv::Matx33d rotation {cv::Matx33d::eye()};
  /// Mean angle between a rotated source ray and its target, in radians, over the
  /// pairs that survived rejection.
  double residual_rad {0.0};
  std::size_t pairs_in {0};
  std::size_t pairs_used {0};
  std::size_t refits {0};
};

/// The rotation that best maps `from` onto `to` — Kabsch, one SVD, no iteration.
///
/// Both vectors are unit bearing rays in the same frame, `from[i]` corresponding
/// to `to[i]`. The answer is the R minimising the sum of squared distances between
/// `R * from[i]` and `to[i]`, which for unit vectors is the same as maximising
/// their alignment.
///
/// **Rotation only, and that is a scope decision rather than a simplification.**
/// Two views of the same points under pure rotation are related by a homography
/// that needs no depth at all — the rays are what move, not the points. Translation
/// cannot be recovered this way even in principle: with zero baseline the essential
/// matrix is degenerate, and with a real baseline these rays are still consistent
/// with any scale of scene. A hand pan carries ~0.9 m of arm arc, so the pose this
/// produces is honest about direction and silent about position, which is why P3
/// publishes a rotation and P7 exists.
///
/// The determinant correction is not optional: without it, a degenerate or
/// noise-dominated set of pairs yields a *reflection* — determinant -1, orthogonal,
/// and a perfectly valid-looking matrix that mirrors the world.
cv::Matx33d fit_rotation(const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to);

/// Mean angle, in radians, between `R * from[i]` and `to[i]`.
///
/// An angle rather than a chord length, so the number means something a person can
/// check: 0.03 rad is 1.7 degrees, which at fx≈950 is ~28 px of disagreement.
double mean_residual_rad(
  const cv::Matx33d & rotation,
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to);

/// Fit, then throw away the worst pairs and fit again, then decide.
///
/// ORB matching produces a few confidently wrong pairs — a repeated corner on a
/// window frame, a reflection — and a least-squares fit has no defence against
/// them: one pair 90 degrees out drags the whole rotation by degrees. Rejecting
/// the worst fraction and refitting is the cheapest robust estimator there is, and
/// at 100+ pairs it is enough.
///
/// The gates, and both of them are about refusing to answer:
///   - at least `min_pairs` surviving pairs (8 by default — below that the fit is
///     interpolating noise);
///   - mean residual under `max_residual_rad` (0.03 by default).
/// Failing either sets `ok = false`, which the caller must read as *hold*.
RotationFit fit_rotation_robust(
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to,
  std::size_t min_pairs = 8, double max_residual_rad = 0.03,
  double reject_fraction = 0.2, std::size_t refits = 2);

/// Change the frame a rotation is expressed in: `R_b = C * R_a * C^T`.
///
/// The fit happens in the optical frame, because that is where the rays are. The
/// pose is published in the body frame, because that is what REP-103 says a robot's
/// orientation means and what every later stage will compose with. `C` is the
/// rotation of `base_link` from `camera_optical_frame`, which this project
/// publishes exactly once as a static transform and reads from TF rather than
/// writing the quaternion into a second place.
///
/// Getting this wrong does not fail: a yaw becomes a roll, the TF frame spins about
/// the wrong axis, and the mesh is built from a consistently mis-oriented sweep.
cv::Matx33d change_basis(const cv::Matx33d & basis, const cv::Matx33d & rotation);

/// A unit quaternion `(x, y, z, w)` from a rotation matrix.
///
/// Shepperd's branch on the largest diagonal term, not the textbook
/// `w = sqrt(1 + trace) / 2` — which loses all precision as the rotation
/// approaches 180 degrees and takes the square root of a small negative number
/// when it gets there.
cv::Vec4d quaternion_from_rotation(const cv::Matx33d & rotation);

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__ROTATION_FIT_HPP_
