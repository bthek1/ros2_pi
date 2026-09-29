#include "pimesh_frontend/local_map_match.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "pimesh_backend/triangulation.hpp"

namespace pimesh_frontend
{

ProjectionSearch search_by_projection(
  const std::vector<pimesh_backend::PointView> & points,
  const std::vector<std::uint8_t> & skip_point,
  const cv::Affine3d & map_from_camera, const cv::Matx33d & k, const cv::Size & image,
  const std::vector<cv::Point2f> & corners, const cv::Mat & corner_descriptors,
  const std::vector<std::uint8_t> & corner_taken,
  const ProjectionConfig & config)
{
  ProjectionSearch out;
  if (corners.empty() || image.width <= 0 || image.height <= 0 ||
    static_cast<std::size_t>(corner_descriptors.rows) != corners.size() ||
    corner_taken.size() != corners.size() || skip_point.size() != points.size())
  {
    return out;
  }

  // --- A grid over the free corners, so each point looks at a handful of them ----
  const double cell = std::max(1.0, config.radius_px);
  const int columns = static_cast<int>(std::ceil(image.width / cell));
  const int rows = static_cast<int>(std::ceil(image.height / cell));
  std::vector<std::vector<std::size_t>> grid(static_cast<std::size_t>(columns * rows));
  auto cell_of = [&](double x, double y, int & cx, int & cy) {
      cx = std::clamp(static_cast<int>(x / cell), 0, columns - 1);
      cy = std::clamp(static_cast<int>(y / cell), 0, rows - 1);
    };
  for (std::size_t c = 0; c < corners.size(); ++c) {
    if (corner_taken[c]) {continue;}
    int cx = 0;
    int cy = 0;
    cell_of(corners[c].x, corners[c].y, cx, cy);
    grid[static_cast<std::size_t>(cy * columns + cx)].push_back(c);
  }

  // --- Each point's best candidate, and whether it is unambiguous ---------------
  std::vector<ProjectionMatch> candidates;
  const double radius_sq = config.radius_px * config.radius_px;
  for (std::size_t p = 0; p < points.size(); ++p) {
    if (skip_point[p] || points[p].descriptor.empty()) {continue;}
    cv::Point2d pixel;
    if (!pimesh_backend::project(map_from_camera, k, points[p].position, pixel)) {continue;}
    if (pixel.x < 0.0 || pixel.y < 0.0 || pixel.x >= image.width || pixel.y >= image.height) {
      continue;
    }
    out.in_view.push_back(p);

    int best = std::numeric_limits<int>::max();
    int second = std::numeric_limits<int>::max();
    std::size_t best_corner = 0;
    int cx = 0;
    int cy = 0;
    cell_of(pixel.x, pixel.y, cx, cy);
    for (int gy = std::max(0, cy - 1); gy <= std::min(rows - 1, cy + 1); ++gy) {
      for (int gx = std::max(0, cx - 1); gx <= std::min(columns - 1, cx + 1); ++gx) {
        for (std::size_t c : grid[static_cast<std::size_t>(gy * columns + gx)]) {
          const double dx = corners[c].x - pixel.x;
          const double dy = corners[c].y - pixel.y;
          if (dx * dx + dy * dy > radius_sq) {continue;}
          const int d = pimesh_backend::hamming(
            points[p].descriptor, corner_descriptors.row(static_cast<int>(c)));
          if (d < best) {
            second = best;
            best = d;
            best_corner = c;
          } else if (d < second) {
            second = d;
          }
        }
      }
    }
    if (best > config.max_hamming) {continue;}
    if (second != std::numeric_limits<int>::max() &&
      static_cast<double>(best) >= config.ratio * static_cast<double>(second))
    {
      continue;
    }
    candidates.push_back(ProjectionMatch{p, best_corner, best});
  }

  // --- One claim per corner, closest first --------------------------------------
  std::stable_sort(
    candidates.begin(), candidates.end(),
    [](const ProjectionMatch & a, const ProjectionMatch & b) {return a.distance < b.distance;});
  std::vector<std::uint8_t> claimed(corners.size(), 0);
  for (const ProjectionMatch & m : candidates) {
    if (claimed[m.corner]) {continue;}
    claimed[m.corner] = 1;
    out.matches.push_back(m);
  }
  return out;
}

}  // namespace pimesh_frontend
