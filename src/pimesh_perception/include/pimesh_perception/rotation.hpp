// Rotation from matched bearing rays — the geometry half of P3's odometry,
// with no ROS and no camera in it.
//
// The idea, in one paragraph. A pixel plus the camera's intrinsic matrix K
// gives a *bearing ray*: the direction, in the optical frame, from the optical
// centre through that pixel. Depth would scale a ray but never turn it, so a
// ray is exactly what a rotation estimator needs and nothing more. Given the
// same physical point seen in two frames, its two rays differ by the camera's
// rotation between them — and finding the single R that best maps one bundle
// of rays onto the other is orthogonal Procrustes, solved by one SVD (Kabsch).
//
// Why rotation ONLY, when P7 will want six degrees of freedom: with no
// baseline the essential matrix is degenerate, and with no depth the
// translation is unobservable up to scale anyway. A hand-held pan is mostly
// rotation, so the honest scope at P3 is a compass. P7 back-fills translation
// once depth exists to give the rays a length.
//
// This header is deliberately ROS-free and header-light so `test_rotation`
// can drive it with synthetic rotations: build a ray bundle, rotate it by a
// known R, feed both, and assert the recovered R matches. That is a test the
// node itself could never be — it needs a camera and a room.

#ifndef PIMESH_PERCEPTION__ROTATION_HPP_
#define PIMESH_PERCEPTION__ROTATION_HPP_

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core.hpp>

namespace pimesh_perception
{

/// Rays are held 3xN, one column per feature: Eigen is column-major, and every
/// operation here is per-ray, so a column is the contiguous unit.
using Rays = Eigen::Matrix3Xd;

/// Row-major 3x3 camera matrix, the layout sensor_msgs/CameraInfo uses.
using CameraMatrix = std::array<double, 9>;

/// True when K is a usable calibration. `camera_node` publishes K all zeros
/// and says so — a fabricated focal length would let every stage downstream
/// compute confident nonsense, so an uncalibrated camera must be *detectable*
/// rather than silently papered over.
bool is_calibrated(const CameraMatrix & k);

/// Unproject pixels to unit bearing rays through K.
Rays rays_from_pixels(const std::vector<cv::Point2f> & pixels, const CameraMatrix & k);

/// Best-fit rotation with curr ~= R * prev. One SVD, no iteration.
///
/// The determinant guard is not decoration: for a degenerate ray bundle — all
/// features on one line, or a near-planar wall filling the frame — the raw SVD
/// answer can come back a reflection, which is not a motion any camera can
/// perform.
Eigen::Matrix3d kabsch(const Rays & prev, const Rays & curr);

/// Per-pair angle in radians between R*prev and curr. This is the fit error,
/// and its mean is what the pose gate thresholds on.
Eigen::VectorXd residual_angles(
  const Eigen::Matrix3d & rotation, const Rays & prev, const Rays & curr);

/// Why a frame produced no pose. Reported per-frame so `just gate-keypoints`
/// can print a reject rate broken down by cause: "too few pairs" means the
/// view is featureless or the motion was too fast, "residual" means the
/// matches disagree with any single rotation, and they call for different
/// fixes.
enum class RotationReject
{
  kNone,          ///< accepted
  kNoIntrinsics,  ///< K is zeros: uncalibrated camera, see is_calibrated()
  kTooFewPairs,   ///< fewer matched pairs than min_pairs, before or after refit
  kResidual,      ///< survived the refit but still disagrees with one rotation
};

const char * to_string(RotationReject reason);

/// The thresholds that decide whether a frame's estimate is trustworthy.
/// Defaults are the predecessor's, which were tuned against a real room.
struct RotationGates
{
  /// Below this many matched pairs the SVD is fitting noise. Three pairs
  /// determine a rotation; eight is the margin that makes it robust.
  std::size_t min_pairs{8};
  /// Mean residual above this and the pairs do not describe one rotation —
  /// usually a moving object in frame, or a burst of false matches.
  double max_residual_rad{0.03};
  /// Reject-worst-and-refit rounds, standing in for RANSAC. Descriptor
  /// matching leaves a few percent of false pairs; they land in the residual
  /// tail, so dropping the tail and refitting removes them without the
  /// machinery of a full consensus search.
  int refit_rounds{2};
  /// Fraction of the worst-fitting pairs dropped per refit round.
  double drop_fraction{0.2};
};

/// The outcome of one frame's estimate. `ok == false` means HOLD THE LAST
/// POSE — never publish a guess. A wrong pose is worse than a stale one:
/// downstream, fusion integrates depth at whatever pose it is handed, and a
/// garbage rotation smears the room permanently.
struct RotationEstimate
{
  bool ok{false};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  double mean_residual_rad{0.0};
  std::size_t pairs_in{0};    ///< pairs offered
  std::size_t pairs_used{0};  ///< pairs surviving the refit rounds
  RotationReject reject{RotationReject::kTooFewPairs};
};

/// Robust rotation from matched ray pairs, or a rejection with its reason.
RotationEstimate estimate_rotation(
  const Rays & prev, const Rays & curr, const RotationGates & gates = {});

/// Angle of a rotation in radians — the magnitude of its axis-angle form.
/// Used to report how far the camera turned without unpacking Euler angles,
/// which have an order convention to get wrong and a gimbal to fall into.
double rotation_angle(const Eigen::Matrix3d & rotation);

/// Re-orthonormalise a rotation. Composing thousands of per-frame rotations
/// accumulates floating-point error until the product is no longer quite
/// orthogonal, and a not-quite-rotation turns into a slightly-scaling
/// transform that quietly grows the map. Cheap insurance, once per frame.
Eigen::Matrix3d orthonormalize(const Eigen::Matrix3d & rotation);

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__ROTATION_HPP_
