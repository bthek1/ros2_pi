#include "pimesh_backend/triangulation.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace pimesh_backend
{

bool project(
  const cv::Affine3d & map_from_camera, const cv::Matx33d & k,
  const cv::Vec3d & point, cv::Point2d & pixel)
{
  const cv::Vec3d c = map_from_camera.inv() * point;
  // A point on or behind the image plane has no pixel. Not a small epsilon about
  // zero: 1 mm is already a projection thousands of pixels off the sensor.
  if (!(c[2] > 1e-3)) {return false;}
  pixel.x = k(0, 0) * c[0] / c[2] + k(0, 2);
  pixel.y = k(1, 1) * c[1] / c[2] + k(1, 2);
  return true;
}

Triangulation triangulate(
  const std::vector<View> & views, double min_parallax_rad, double max_error_px)
{
  Triangulation out;
  if (views.size() < 2) {
    out.refusal = "one view";
    return out;
  }

  // --- Parallax, from the rays in the map frame ------------------------------
  //
  // The widest pair, not the first two: a point seen by five keyframes on a slow
  // pan has tiny parallax between neighbours and a usable amount between the ends.
  std::vector<cv::Vec3d> rays;
  rays.reserve(views.size());
  for (const View & view : views) {
    const cv::Vec3d normalised(
      (view.pixel.x - view.k(0, 2)) / view.k(0, 0),
      (view.pixel.y - view.k(1, 2)) / view.k(1, 1),
      1.0);
    rays.push_back(cv::normalize(cv::Vec3d(view.map_from_camera.rotation() * normalised)));
  }
  double widest_cos = 1.0;
  for (std::size_t i = 0; i < rays.size(); ++i) {
    for (std::size_t j = i + 1; j < rays.size(); ++j) {
      widest_cos = std::min(widest_cos, rays[i].dot(rays[j]));
    }
  }
  // Clamped before the acos, for angle_between()'s reason: 1.0000000002 is ordinary
  // floating point, and acos of it is NaN, which compares false against the
  // threshold below and would *accept* the point.
  out.parallax_rad = std::acos(std::max(-1.0, std::min(1.0, widest_cos)));
  if (out.parallax_rad < min_parallax_rad) {
    out.refusal = "parallax";
    return out;
  }

  // --- The linear solve ----------------------------------------------------------
  //
  // Each view contributes x * P3 - P1 and y * P3 - P2, with P = [R | t] the
  // camera-from-map transform and (x, y) the normalised pixel. The solution is the
  // right singular vector of the smallest singular value, in homogeneous form.
  cv::Mat a(static_cast<int>(2 * views.size()), 4, CV_64F);
  for (std::size_t i = 0; i < views.size(); ++i) {
    const View & view = views[i];
    const cv::Affine3d camera_from_map = view.map_from_camera.inv();
    const cv::Matx33d r = camera_from_map.rotation();
    const cv::Vec3d t = camera_from_map.translation();
    const double x = (view.pixel.x - view.k(0, 2)) / view.k(0, 0);
    const double y = (view.pixel.y - view.k(1, 2)) / view.k(1, 1);
    const int row = static_cast<int>(2 * i);
    for (int c = 0; c < 3; ++c) {
      a.at<double>(row, c) = x * r(2, c) - r(0, c);
      a.at<double>(row + 1, c) = y * r(2, c) - r(1, c);
    }
    a.at<double>(row, 3) = x * t[2] - t[0];
    a.at<double>(row + 1, 3) = y * t[2] - t[1];
  }
  cv::Mat w;
  cv::Mat u;
  cv::Mat vt;
  cv::SVD::compute(a, w, u, vt, cv::SVD::FULL_UV);
  const double h = vt.at<double>(3, 3);
  if (std::abs(h) < 1e-12) {
    // A point at infinity: the rays are parallel after all, in a way the angle
    // test above did not catch (it cannot happen past that test with real views,
    // and a division by it would be an inf that every later check passes).
    out.refusal = "at infinity";
    return out;
  }
  out.point = cv::Vec3d(vt.at<double>(3, 0) / h, vt.at<double>(3, 1) / h, vt.at<double>(3, 2) / h);

  // --- Cheirality and reprojection, per view ------------------------------------
  double sum = 0.0;
  for (const View & view : views) {
    cv::Point2d pixel;
    if (!project(view.map_from_camera, view.k, out.point, pixel)) {
      out.refusal = "behind a camera";
      return out;
    }
    const double error = std::hypot(pixel.x - view.pixel.x, pixel.y - view.pixel.y);
    out.max_error_px = std::max(out.max_error_px, error);
    sum += error;
  }
  out.mean_error_px = sum / static_cast<double>(views.size());
  if (out.max_error_px > max_error_px) {
    out.refusal = "reprojection";
    return out;
  }
  out.ok = true;
  return out;
}

}  // namespace pimesh_backend
