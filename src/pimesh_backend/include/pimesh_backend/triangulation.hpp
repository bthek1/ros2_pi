#ifndef PIMESH_BACKEND__TRIANGULATION_HPP_
#define PIMESH_BACKEND__TRIANGULATION_HPP_

#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"

namespace pimesh_backend
{

/// One sighting of a point: which camera, where it was, and where on its sensor.
struct View
{
  /// `map <- camera_optical_frame` for the camera that saw it.
  cv::Affine3d map_from_camera {cv::Affine3d::Identity()};
  cv::Matx33d k {cv::Matx33d::eye()};
  cv::Point2f pixel;
};

/// What a triangulation came back with, and whether to believe it.
struct Triangulation
{
  /// True only if every refusal below passed. False means **keep the position the
  /// point already has** — which for a point in this map is the depth network's
  /// reading, and is a better answer than a triangulation that failed a check.
  bool ok {false};
  cv::Vec3d point {0.0, 0.0, 0.0};
  /// The widest angle between any two of the rays, in radians. Reported whether or
  /// not it passed, because it is the number that says *why* a point was not
  /// triangulated — and on a pan it is most of them.
  double parallax_rad {0.0};
  /// Reprojection error of the solved point in the view that fits it worst, and the
  /// mean over all of them, in pixels.
  double max_error_px {0.0};
  double mean_error_px {0.0};
  /// Which refusal fired, if one did. A string rather than an enum because it is
  /// only ever counted and printed.
  const char * refusal {""};
};

/// A 3D point from two or more sightings of it, by linear least squares (DLT) over
/// every view at once.
///
/// **This is the first opinion about depth in this project that does not come from
/// the depth network.** Every landmark P7 poses against is a pixel pushed out along
/// its ray by however far Depth Anything V2 said the surface was; the network's
/// error is a smooth warp over the frame, it does not average down, and nothing
/// downstream could see it. A triangulated point depends on the camera poses and the
/// pixels only. It is not *independent* of the network — the poses were solved
/// against network-depth landmarks — but it is a second route to the same number,
/// which is what makes the ratio between the two worth printing.
///
/// **Three refusals, and the first is the one that matters.**
///
///   - **Parallax.** Two rays that are nearly parallel meet somewhere along a long
///     thin region, so a sub-pixel error in either moves the answer by metres along
///     the ray while the reprojection error stays tiny — a triangulation is least
///     trustworthy in exactly the case where its own residual looks best. The
///     depth error per pixel of noise is roughly `d / (f * parallax)`: at f = 525
///     and 1 degree that is ~11% of the distance per pixel, worse than the network
///     it would replace. So below `min_parallax_rad` this refuses, and the caller
///     keeps the network's reading.
///   - **Cheirality.** The solved point must be in front of every camera that saw
///     it. DLT is happy to return the mirror image behind the cameras, where every
///     reprojection is perfect.
///   - **Reprojection.** Every view, not the mean — a point that fits three views
///     and misses the fourth by 20 px is a mismatch in the fourth, and a mean over
///     four would call it 5 px and accept it.
///
/// Pixels are converted to normalised coordinates before the solve, so the linear
/// system is conditioned on the unit sphere and not on pixel magnitudes of ~10^3.
Triangulation triangulate(
  const std::vector<View> & views, double min_parallax_rad, double max_error_px);

/// Where `point` (map frame) lands on the sensor of the camera at `map_from_camera`,
/// and whether it is in front of it at all.
bool project(
  const cv::Affine3d & map_from_camera, const cv::Matx33d & k,
  const cv::Vec3d & point, cv::Point2d & pixel);

}  // namespace pimesh_backend

#endif  // PIMESH_BACKEND__TRIANGULATION_HPP_
