#include "pimesh_perception/rotation_fit.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

namespace pimesh_perception
{

cv::Vec3d bearing(const cv::Matx33d & k, double x, double y)
{
  const double fx = k(0, 0);
  const double fy = k(1, 1);
  const double cx = k(0, 2);
  const double cy = k(1, 2);
  // A zero focal length would be a CameraInfo that never arrived; returning the
  // optical axis is the one answer that is wrong in a visible way rather than NaN,
  // which propagates silently through an SVD.
  if (fx == 0.0 || fy == 0.0) {return cv::Vec3d(0.0, 0.0, 1.0);}

  cv::Vec3d ray((x - cx) / fx, (y - cy) / fy, 1.0);
  return cv::normalize(ray);
}

cv::Matx33d fit_rotation(const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to)
{
  if (from.size() != to.size() || from.size() < 3) {return cv::Matx33d::eye();}

  // The 3x3 correlation of the two ray sets. No centroid subtraction: these are
  // directions from a common origin, not points in space, so there is no centroid
  // to remove — subtracting their mean would be fitting the rotation of a cloud
  // that does not exist.
  cv::Matx33d h = cv::Matx33d::zeros();
  for (std::size_t i = 0; i < from.size(); ++i) {
    h += cv::Matx33d(to[i] * from[i].t());
  }

  cv::Matx33d u;
  cv::Matx31d w;
  cv::Matx33d vt;
  cv::SVD::compute(h, w, u, vt);

  cv::Matx33d rotation = u * vt;

  // The reflection guard. u * vt is orthogonal but not necessarily a rotation; on
  // a degenerate or noise-dominated set its determinant comes out -1, which is a
  // mirror. Flipping the sign of the column matching the smallest singular value
  // is the least-damaging correction — it is the direction the data constrains
  // least.
  if (cv::determinant(rotation) < 0.0) {
    cv::Matx33d flip = cv::Matx33d::eye();
    flip(2, 2) = -1.0;
    rotation = u * flip * vt;
  }

  return rotation;
}

double mean_residual_rad(
  const cv::Matx33d & rotation,
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to)
{
  if (from.empty() || from.size() != to.size()) {return 0.0;}

  double sum = 0.0;
  for (std::size_t i = 0; i < from.size(); ++i) {
    const cv::Vec3d rotated = rotation * from[i];
    // Clamped before acos: a dot product of 1.0000000002 is ordinary floating
    // point and acos of it is NaN, which then poisons the mean and every gate
    // reading it.
    const double dot = std::max(-1.0, std::min(1.0, rotated.dot(to[i])));
    sum += std::acos(dot);
  }
  return sum / static_cast<double>(from.size());
}

RotationFit fit_rotation_robust(
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to,
  std::size_t min_pairs, double max_residual_rad, double reject_fraction, std::size_t refits)
{
  RotationFit fit;
  fit.pairs_in = from.size();

  if (from.size() != to.size() || from.size() < min_pairs || from.size() < 3) {
    return fit;   // ok stays false: too few pairs to answer at all
  }

  std::vector<cv::Vec3d> a = from;
  std::vector<cv::Vec3d> b = to;

  cv::Matx33d rotation = fit_rotation(a, b);

  for (std::size_t round = 0; round < refits; ++round) {
    // How far each pair is from the current answer, largest first.
    std::vector<std::size_t> order(a.size());
    std::iota(order.begin(), order.end(), 0U);
    std::vector<double> error(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
      const cv::Vec3d rotated = rotation * a[i];
      const double dot = std::max(-1.0, std::min(1.0, rotated.dot(b[i])));
      error[i] = std::acos(dot);
    }
    std::sort(
      order.begin(), order.end(),
      [&error](std::size_t l, std::size_t r) {return error[l] < error[r];});

    const std::size_t keep = static_cast<std::size_t>(
      static_cast<double>(a.size()) * (1.0 - reject_fraction));
    // Never reject below the gate's own floor: dropping pairs until a fit looks
    // good is how a robust estimator becomes a way of manufacturing agreement.
    if (keep < std::max<std::size_t>(min_pairs, 3)) {break;}

    std::vector<cv::Vec3d> a_keep;
    std::vector<cv::Vec3d> b_keep;
    a_keep.reserve(keep);
    b_keep.reserve(keep);
    for (std::size_t i = 0; i < keep; ++i) {
      a_keep.push_back(a[order[i]]);
      b_keep.push_back(b[order[i]]);
    }
    a = std::move(a_keep);
    b = std::move(b_keep);
    rotation = fit_rotation(a, b);
    ++fit.refits;
  }

  fit.rotation = rotation;
  fit.pairs_used = a.size();
  fit.residual_rad = mean_residual_rad(rotation, a, b);
  fit.ok = (fit.pairs_used >= min_pairs) && (fit.residual_rad < max_residual_rad);
  return fit;
}

cv::Matx33d change_basis(const cv::Matx33d & basis, const cv::Matx33d & rotation)
{
  return basis * rotation * basis.t();
}

cv::Vec4d quaternion_from_rotation(const cv::Matx33d & m)
{
  const double trace = m(0, 0) + m(1, 1) + m(2, 2);
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;

  if (trace > 0.0) {
    const double s = std::sqrt(trace + 1.0) * 2.0;
    w = 0.25 * s;
    x = (m(2, 1) - m(1, 2)) / s;
    y = (m(0, 2) - m(2, 0)) / s;
    z = (m(1, 0) - m(0, 1)) / s;
  } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
    const double s = std::sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2.0;
    w = (m(2, 1) - m(1, 2)) / s;
    x = 0.25 * s;
    y = (m(0, 1) + m(1, 0)) / s;
    z = (m(0, 2) + m(2, 0)) / s;
  } else if (m(1, 1) > m(2, 2)) {
    const double s = std::sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2.0;
    w = (m(0, 2) - m(2, 0)) / s;
    x = (m(0, 1) + m(1, 0)) / s;
    y = 0.25 * s;
    z = (m(1, 2) + m(2, 1)) / s;
  } else {
    const double s = std::sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2.0;
    w = (m(1, 0) - m(0, 1)) / s;
    x = (m(0, 2) + m(2, 0)) / s;
    y = (m(1, 2) + m(2, 1)) / s;
    z = 0.25 * s;
  }

  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm == 0.0) {return cv::Vec4d(0.0, 0.0, 0.0, 1.0);}
  return cv::Vec4d(x / norm, y / norm, z / norm, w / norm);
}

}  // namespace pimesh_perception
