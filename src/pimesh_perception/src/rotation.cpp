#include "pimesh_perception/rotation.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <Eigen/SVD>

namespace pimesh_perception
{

bool is_calibrated(const CameraMatrix & k)
{
  // fx and fy are the two entries nothing can work without, and both are
  // exactly 0.0 in the uncalibrated CameraInfo `camera_node` publishes.
  return k[0] > 0.0 && k[4] > 0.0;
}

Rays rays_from_pixels(const std::vector<cv::Point2f> & pixels, const CameraMatrix & k)
{
  const double fx = k[0], cx = k[2], fy = k[4], cy = k[5];

  Rays rays(3, static_cast<Eigen::Index>(pixels.size()));
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    // The pinhole model, run backwards. Forward it is u = fx*X/Z + cx; with Z
    // unknown, fixing Z = 1 gives the direction and normalising drops the
    // arbitrary scale. What survives is pure bearing.
    const double x = (static_cast<double>(pixels[i].x) - cx) / fx;
    const double y = (static_cast<double>(pixels[i].y) - cy) / fy;
    rays.col(static_cast<Eigen::Index>(i)) = Eigen::Vector3d(x, y, 1.0).normalized();
  }
  return rays;
}

Eigen::Matrix3d kabsch(const Rays & prev, const Rays & curr)
{
  // H = sum_i prev_i * curr_i^T, the 3x3 cross-covariance of the two bundles.
  // Everything about the fit is in these nine numbers — the pair count only
  // affects how well-conditioned they are.
  const Eigen::Matrix3d h = prev * curr.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix3d v = svd.matrixV();
  Eigen::Matrix3d rotation = v * svd.matrixU().transpose();
  if (rotation.determinant() < 0.0) {
    // det = -1 is a reflection. Flipping the singular vector belonging to the
    // SMALLEST singular value is the minimal repair: it is the direction the
    // data constrains least, so this changes the fit as little as possible.
    v.col(2) *= -1.0;
    rotation = v * svd.matrixU().transpose();
  }
  return rotation;
}

Eigen::VectorXd residual_angles(
  const Eigen::Matrix3d & rotation, const Rays & prev, const Rays & curr)
{
  // Both bundles are unit vectors, so the dot product is the cosine directly.
  // Clamped because rounding can push it a hair outside [-1, 1] and acos would
  // return NaN, which then poisons the mean and passes the gate by being
  // neither greater nor less than the threshold.
  const Eigen::VectorXd cosines =
    (curr.array() * (rotation * prev).array()).colwise().sum().transpose();
  return cosines.array().min(1.0).max(-1.0).acos();
}

const char * to_string(RotationReject reason)
{
  switch (reason) {
    case RotationReject::kNone: return "ok";
    case RotationReject::kNoIntrinsics: return "no_intrinsics";
    case RotationReject::kTooFewPairs: return "too_few_pairs";
    case RotationReject::kResidual: return "residual";
  }
  return "unknown";
}

RotationEstimate estimate_rotation(
  const Rays & prev, const Rays & curr, const RotationGates & gates)
{
  RotationEstimate out;
  out.pairs_in = static_cast<std::size_t>(prev.cols());

  if (prev.cols() != curr.cols() ||
    static_cast<std::size_t>(prev.cols()) < gates.min_pairs)
  {
    out.reject = RotationReject::kTooFewPairs;
    return out;
  }

  Rays p = prev;
  Rays c = curr;
  Eigen::Matrix3d rotation = kabsch(p, c);

  for (int round = 0; round < gates.refit_rounds; ++round) {
    const Eigen::VectorXd residuals = residual_angles(rotation, p, c);
    const auto keep_count = static_cast<Eigen::Index>(
      std::ceil(static_cast<double>(p.cols()) * (1.0 - gates.drop_fraction)));
    if (keep_count < static_cast<Eigen::Index>(gates.min_pairs)) {
      break;    // refitting below the gate would only manufacture a fit
    }

    // Partial sort: we want the keep_count smallest residuals, not a full
    // ordering of all of them.
    std::vector<Eigen::Index> order(static_cast<std::size_t>(p.cols()));
    std::iota(order.begin(), order.end(), 0);
    std::nth_element(
      order.begin(), order.begin() + keep_count, order.end(),
      [&residuals](Eigen::Index a, Eigen::Index b) {return residuals(a) < residuals(b);});

    Rays kept_p(3, keep_count);
    Rays kept_c(3, keep_count);
    for (Eigen::Index i = 0; i < keep_count; ++i) {
      kept_p.col(i) = p.col(order[static_cast<std::size_t>(i)]);
      kept_c.col(i) = c.col(order[static_cast<std::size_t>(i)]);
    }
    p = std::move(kept_p);
    c = std::move(kept_c);
    rotation = kabsch(p, c);
  }

  const Eigen::VectorXd residuals = residual_angles(rotation, p, c);
  out.pairs_used = static_cast<std::size_t>(p.cols());
  out.mean_residual_rad = residuals.mean();
  out.rotation = rotation;

  if (out.pairs_used < gates.min_pairs) {
    out.reject = RotationReject::kTooFewPairs;
    return out;
  }
  if (out.mean_residual_rad > gates.max_residual_rad) {
    out.reject = RotationReject::kResidual;
    return out;
  }

  out.ok = true;
  out.reject = RotationReject::kNone;
  return out;
}

double rotation_angle(const Eigen::Matrix3d & rotation)
{
  return Eigen::AngleAxisd(rotation).angle();
}

Eigen::Matrix3d orthonormalize(const Eigen::Matrix3d & rotation)
{
  // The nearest orthogonal matrix in the Frobenius sense: take the SVD and
  // throw the singular VALUES away, keeping only the rotation they scaled.
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
    rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d u = svd.matrixU();
  Eigen::Matrix3d fixed = u * svd.matrixV().transpose();
  if (fixed.determinant() < 0.0) {
    u.col(2) *= -1.0;
    fixed = u * svd.matrixV().transpose();
  }
  return fixed;
}

}  // namespace pimesh_perception
