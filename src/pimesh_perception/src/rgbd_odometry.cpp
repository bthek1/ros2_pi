#include "pimesh_perception/rgbd_odometry.hpp"

// calib3d, for solvePnPRansac. Present and identical in OpenCV 4.6 on the Pi and
// 4.10 here — checked, because this package's rule is that nothing in it touches
// an API that differs between the two, and cv::aruco has already cost this
// project an afternoon for exactly that.
#include "opencv2/calib3d.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numeric>
#include <vector>

namespace pimesh_perception
{

cv::Vec3d unproject(const cv::Matx33d & k, double x, double y, double z)
{
  const double fx = k(0, 0);
  const double fy = k(1, 1);
  // A zero focal length is a CameraInfo that never arrived. The origin is the one
  // answer that is wrong in a visible way rather than NaN, which propagates
  // silently through an SVD and poisons every pair beside it.
  if (fx == 0.0 || fy == 0.0) {return cv::Vec3d(0.0, 0.0, 0.0);}
  return cv::Vec3d((x - k(0, 2)) / fx * z, (y - k(1, 2)) / fy * z, z);
}

bool sample_depth(
  const cv::Mat & depth_32f, double x, double y, int patch,
  double min_m, double max_m, double max_spread, double & metres)
{
  if (depth_32f.empty() || depth_32f.type() != CV_32FC1) {return false;}

  const int half = std::max(0, patch / 2);
  const int cx = static_cast<int>(std::lround(x));
  const int cy = static_cast<int>(std::lround(y));
  // The whole patch has to be on the image, not merely its centre. Clamping the
  // window instead would quietly read a smaller, off-centre neighbourhood at the
  // frame edge — and the frame edge is where a wide-angle lens puts the geometry
  // that constrains rotation best.
  if (cx - half < 0 || cy - half < 0 ||
    cx + half >= depth_32f.cols || cy + half >= depth_32f.rows)
  {
    return false;
  }

  // At patch 3 this is nine doubles; a fixed upper bound keeps the hot path free
  // of allocation without pretending the patch size is a compile-time constant.
  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(2 * half + 1) * static_cast<std::size_t>(2 * half + 1));
  for (int row = cy - half; row <= cy + half; ++row) {
    const float * line = depth_32f.ptr<float>(row);
    for (int col = cx - half; col <= cx + half; ++col) {
      const double value = static_cast<double>(line[col]);
      if (!std::isfinite(value) || value < min_m || value > max_m) {continue;}
      values.push_back(value);
    }
  }

  // Better than half the window has to be usable. A corner where most of the patch
  // is past the clip range is a corner on the sky, and the few readings that
  // survive there are the ones nearest the boundary rather than a sample of the
  // surface.
  const std::size_t window =
    static_cast<std::size_t>(2 * half + 1) * static_cast<std::size_t>(2 * half + 1);
  if (values.size() * 2 <= window) {return false;}

  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<long>(mid), values.end());
  const double median = values[mid];
  if (median <= 0.0) {return false;}

  // The flatness gate, on the *surviving* samples. min and max rather than a
  // standard deviation: a step is what is being excluded, and one pixel on the far
  // side of it is exactly the case a variance would average away.
  const auto extremes = std::minmax_element(values.begin(), values.end());
  if ((*extremes.second - *extremes.first) / median > max_spread) {return false;}

  metres = median;
  return true;
}

RigidFit fit_rigid(
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to, ScaleHandling scale)
{
  RigidFit fit;
  fit.pairs_in = from.size();
  if (from.size() != to.size() || from.size() < 3) {return fit;}

  const double n = static_cast<double>(from.size());
  cv::Vec3d centre_from(0.0, 0.0, 0.0);
  cv::Vec3d centre_to(0.0, 0.0, 0.0);
  for (std::size_t i = 0; i < from.size(); ++i) {
    centre_from += from[i];
    centre_to += to[i];
  }
  centre_from /= n;
  centre_to /= n;

  // The 3x3 correlation of the two clouds about their own centroids, and the
  // spread of the source cloud about its own. Removing the centroid is the one
  // line that separates this from the ray fit: it is what leaves a translation to
  // solve for afterwards. `spread` is only needed for the scale, and it is
  // accumulated in the same pass because it is the same subtraction.
  cv::Matx33d h = cv::Matx33d::zeros();
  double spread = 0.0;
  for (std::size_t i = 0; i < from.size(); ++i) {
    const cv::Vec3d a = from[i] - centre_from;
    h += cv::Matx33d((to[i] - centre_to) * a.t());
    spread += a.dot(a);
  }

  cv::Matx33d u;
  cv::Matx31d w;
  cv::Matx33d vt;
  cv::SVD::compute(h, w, u, vt);

  // The reflection guard, as in fit_rotation(). A degenerate cloud — every
  // landmark on one wall, which is an ordinary thing to be looking at — yields
  // determinant -1, an orthogonal matrix that mirrors the room and looks entirely
  // valid. The sign lands in the smallest singular value, which is why it has to
  // be carried into the scale below too.
  double smallest_sign = 1.0;
  cv::Matx33d rotation = u * vt;
  if (cv::determinant(rotation) < 0.0) {
    cv::Matx33d flip = cv::Matx33d::eye();
    flip(2, 2) = -1.0;
    rotation = u * flip * vt;
    smallest_sign = -1.0;
  }

  fit.rotation = rotation;
  if (scale == ScaleHandling::DivideOut && spread > 0.0) {
    // Umeyama's scale: the correlation the rotation actually explains, over the
    // source cloud's own spread. `trace(S)` with the reflection sign on its
    // smallest term — the same correction the rotation just took, and leaving it
    // out gives a scale that is wrong by twice the smallest singular value on
    // exactly the degenerate clouds the guard above is for.
    const double explained = w(0, 0) + w(1, 0) + smallest_sign * w(2, 0);
    fit.scale = explained / spread;
    // A non-positive or absurd scale is a cloud that does not support the
    // question. 1.0 is the refusal, and the residual gate then sees the fit for
    // what it is rather than being handed a number that hides it.
    if (!(fit.scale > 0.1 && fit.scale < 10.0)) {fit.scale = 1.0;}
  }
  fit.translation = centre_to - fit.scale * (rotation * centre_from);
  return fit;
}

double mean_residual_m(
  const RigidFit & fit,
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to)
{
  if (from.empty() || from.size() != to.size()) {return 0.0;}
  double sum = 0.0;
  for (std::size_t i = 0; i < from.size(); ++i) {
    sum += cv::norm(fit.map(from[i]) - to[i]);
  }
  return sum / static_cast<double>(from.size());
}

RigidFit fit_rigid_robust(
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to,
  std::size_t min_pairs, double max_residual_m, double reject_fraction, std::size_t refits,
  ScaleHandling scale)
{
  RigidFit fit;
  fit.pairs_in = from.size();

  if (from.size() != to.size() || from.size() < min_pairs || from.size() < 3) {
    return fit;   // ok stays false: too few pairs to answer at all
  }

  std::vector<cv::Vec3d> a = from;
  std::vector<cv::Vec3d> b = to;
  RigidFit current = fit_rigid(a, b, scale);

  for (std::size_t round = 0; round < refits; ++round) {
    std::vector<double> error(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
      error[i] = cv::norm(current.map(a[i]) - b[i]);
    }
    std::vector<std::size_t> order(a.size());
    std::iota(order.begin(), order.end(), 0U);
    std::sort(
      order.begin(), order.end(),
      [&error](std::size_t l, std::size_t r) {return error[l] < error[r];});

    const std::size_t keep = static_cast<std::size_t>(
      static_cast<double>(a.size()) * (1.0 - reject_fraction));
    // Never below the gate's own floor. Rejecting until a fit looks good is how a
    // robust estimator becomes a way of manufacturing agreement — the same note
    // fit_rotation_robust() carries, and the same arithmetic.
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
    current = fit_rigid(a, b, scale);
    ++fit.refits;
  }

  const std::size_t refits_done = fit.refits;
  fit = current;
  fit.pairs_in = from.size();
  fit.pairs_used = a.size();
  fit.refits = refits_done;
  fit.residual_m = mean_residual_m(fit, a, b);
  fit.ok = (fit.pairs_used >= min_pairs) && (fit.residual_m < max_residual_m);
  return fit;
}

TranslationFit fit_translation_robust(
  const cv::Matx33d & rotation,
  const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to,
  std::size_t min_pairs, double max_residual_m, double reject_fraction, std::size_t refits,
  ScaleHandling scale)
{
  TranslationFit fit;
  fit.pairs_in = from.size();
  if (from.size() != to.size() || from.size() < min_pairs || from.empty()) {return fit;}

  std::vector<cv::Vec3d> a = from;
  std::vector<cv::Vec3d> b = to;

  auto solve = [&rotation, scale](
    const std::vector<cv::Vec3d> & p, const std::vector<cv::Vec3d> & q, TranslationFit & out) {
      const double n = static_cast<double>(p.size());
      cv::Vec3d cp(0.0, 0.0, 0.0);
      cv::Vec3d cq(0.0, 0.0, 0.0);
      for (std::size_t i = 0; i < p.size(); ++i) {
        cp += p[i];
        cq += q[i];
      }
      cp /= n;
      cq /= n;

      out.scale = 1.0;
      if (scale == ScaleHandling::DivideOut) {
        // The least-squares scale with the rotation already fixed: how much of the
        // target cloud's spread the rotated source explains, over the source's own.
        double explained = 0.0;
        double spread = 0.0;
        for (std::size_t i = 0; i < p.size(); ++i) {
          const cv::Vec3d u = rotation * (p[i] - cp);
          explained += u.dot(q[i] - cq);
          spread += u.dot(u);
        }
        if (spread > 0.0) {out.scale = explained / spread;}
        if (!(out.scale > 0.1 && out.scale < 10.0)) {out.scale = 1.0;}
      }
      out.translation = cq - out.scale * (rotation * cp);
    };

  solve(a, b, fit);

  for (std::size_t round = 0; round < refits; ++round) {
    std::vector<double> error(a.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
      error[i] = cv::norm(fit.map(rotation, a[i]) - b[i]);
    }
    std::vector<std::size_t> order(a.size());
    std::iota(order.begin(), order.end(), 0U);
    std::sort(
      order.begin(), order.end(),
      [&error](std::size_t l, std::size_t r) {return error[l] < error[r];});

    const std::size_t keep = static_cast<std::size_t>(
      static_cast<double>(a.size()) * (1.0 - reject_fraction));
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
    solve(a, b, fit);
    ++fit.refits;
  }

  fit.pairs_used = a.size();
  double sum = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {sum += cv::norm(fit.map(rotation, a[i]) - b[i]);}
  fit.residual_m = sum / static_cast<double>(a.size());
  fit.ok = (fit.pairs_used >= min_pairs) && (fit.residual_m < max_residual_m);
  return fit;
}

PnpFit fit_pose_pnp(
  const cv::Matx33d & k,
  const std::vector<cv::Vec3d> & object, const std::vector<cv::Point2f> & image,
  std::size_t min_inliers, double reprojection_px, double max_residual_px, int iterations)
{
  PnpFit fit;
  fit.pairs_in = object.size();
  // Four is solvePnPRansac's own floor; min_inliers is this project's, and it is
  // higher for the reason the landmark fit's is: a pose that exactly explains a
  // handful of points has fitted their noise and reports no error at all.
  if (object.size() != image.size() || object.size() < std::max<std::size_t>(min_inliers, 4)) {
    return fit;
  }

  std::vector<cv::Point3f> points;
  points.reserve(object.size());
  for (const cv::Vec3d & p : object) {
    points.emplace_back(
      static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2]));
  }

  cv::Mat rvec;
  cv::Mat tvec;
  std::vector<int> inliers;
  const cv::Mat camera = cv::Mat(cv::Matx33d(k));
  bool solved = false;
  try {
    solved = cv::solvePnPRansac(
      points, image, camera, cv::Mat(), rvec, tvec, false, iterations,
      static_cast<float>(reprojection_px), 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
  } catch (const cv::Exception &) {
    // A degenerate configuration throws rather than returning false. It is a
    // refusal either way, and the caller reads both as *hold*.
    return fit;
  }
  if (!solved || inliers.size() < min_inliers) {
    fit.inliers = inliers.size();
    return fit;
  }

  cv::Matx33d rotation;
  cv::Mat rotation_mat;
  cv::Rodrigues(rvec, rotation_mat);
  rotation_mat.convertTo(rotation_mat, CV_64F);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {rotation(r, c) = rotation_mat.at<double>(r, c);}
  }
  fit.rotation = rotation;
  fit.translation = cv::Vec3d(
    tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
  fit.inliers = inliers.size();

  // The residual over the **inliers only**, which is the honest denominator: an
  // outlier's reprojection error is unbounded, so averaging it in would make the
  // number say more about how many outliers there were than about how well the
  // pose fits what it was fitted to.
  double sum = 0.0;
  for (int index : inliers) {
    const cv::Vec3d camera_point = fit.rotation * object[static_cast<std::size_t>(index)] +
      fit.translation;
    if (camera_point[2] <= 0.0) {continue;}
    const double u = k(0, 0) * camera_point[0] / camera_point[2] + k(0, 2);
    const double v = k(1, 1) * camera_point[1] / camera_point[2] + k(1, 2);
    const cv::Point2f & seen = image[static_cast<std::size_t>(index)];
    sum += std::hypot(u - seen.x, v - seen.y);
  }
  fit.residual_px = sum / static_cast<double>(inliers.size());
  fit.ok = (fit.residual_px < max_residual_px);
  return fit;
}

cv::Affine3d camera_step(const cv::Affine3d & point_motion)
{
  return point_motion.inv();
}

cv::Matx33d camera_step(const cv::Matx33d & point_rotation)
{
  return point_rotation.t();
}

cv::Affine3d change_basis(const cv::Affine3d & basis, const cv::Affine3d & motion)
{
  return basis * motion * basis.inv();
}

std::array<double, 36> unconstrained_covariance()
{
  // Diagonal only. An off-diagonal term is a *correlation*, which is a claim
  // about the estimator's error structure, and this one makes none.
  std::array<double, 36> covariance {};
  for (int i = 0; i < 6; ++i) {
    covariance[static_cast<std::size_t>(i * 6 + i)] = kOdomUnconstrainedVariance;
  }
  return covariance;
}

}  // namespace pimesh_perception
