#ifndef PIMESH_DEPTH__DEPTH_PATCH_HPP_
#define PIMESH_DEPTH__DEPTH_PATCH_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "opencv2/core.hpp"
#include "pimesh_core/stats.hpp"

namespace pimesh_depth
{

/// What a centred region of one depth map says, and how much of it said nothing.
///
/// **The counters are not bookkeeping.** `usable` being small, or `clipped` being
/// large, each turns the median into a number about something other than the
/// surface — and both failures produce a median that looks like a distance.
struct PatchStats
{
  /// Pixels the patch covers.
  std::size_t total {0};
  /// Finite, positive, and below the far clip — the ones the quartiles are over.
  std::size_t usable {0};
  /// At or past `max_m`. **These are excluded and counted, never averaged in.**
  /// `max_range_m` is the model's "far away or no idea", written into the map as
  /// a real-looking distance, so a patch that is half clip reports a median
  /// somewhere between the surface and 6 m and nothing about it looks wrong.
  std::size_t clipped {0};
  /// Not finite, or not positive.
  std::size_t bad {0};
  /// Quartiles over the usable samples. All zero when `usable == 0` — which is
  /// why a caller must branch on `usable` and not on `median`: 0 m is a
  /// perfectly plausible-looking distance, and this project has already shipped
  /// one gate that asserted 0.00 against a budget and printed PASS.
  double q1 {0.0};
  double median {0.0};
  double q3 {0.0};

  /// Spatial spread across the patch, in metres. On a flat surface square-on
  /// this is the depth map's own noise; large means the patch is not on one
  /// surface.
  double iqr() const {return q3 - q1;}

  /// `clipped` as a fraction of the patch. The number that decides whether an
  /// implied scale computed from `median` means anything — see below.
  double clipped_fraction() const
  {
    return (total == 0) ? 0.0 : static_cast<double>(clipped) / static_cast<double>(total);
  }
};

/// Quartiles of a centred, axis-aligned block of a 32FC1 depth map.
///
/// **Why a patch and not the pixel at the principal point.** `test_tsdf_volume`
/// already pins that a depth map carries *z*, not distance along the ray, so the
/// principal point is the one place where the two agree exactly — it is the best
/// case, and a single reading there means one hot pixel would set this project's
/// unit for good. A patch with its spread reported cannot do that quietly.
///
/// **Why the far clip is excluded rather than included.** `depth_to_metres`
/// writes `max_range` wherever the model's inverse depth falls below its floor,
/// so a clipped pixel is not a large distance, it is the absence of one — and
/// depth is linear in `depth_scale` *only below the clip*. An implied scale
/// derived from a median that has clip values in it is wrong in the direction
/// that makes the scale look smaller than it is, with nothing in the output
/// saying so. That is what `clipped` is for, and why the caller is expected to
/// refuse on it rather than read around it.
///
/// \param fraction  side length of the patch as a fraction of each dimension,
///                  clamped to (0, 1]. 0.25 of 1280x720 is 320x180.
/// \param max_m     the far clip the map was written with. A reading at or past
///                  it is counted as clipped, never as a distance.
inline PatchStats centred_patch_stats(
  const cv::Mat & depth_32f, double fraction, double max_m)
{
  PatchStats out;
  if (depth_32f.empty() || depth_32f.type() != CV_32FC1) {return out;}

  const double f = std::min(1.0, std::max(1e-6, fraction));
  // At least one pixel in each dimension, however small the fraction: a patch
  // that rounded to zero would report `usable == 0` and read as "the surface was
  // out of range" rather than as "you asked for nothing".
  const int w = std::max(1, static_cast<int>(depth_32f.cols * f));
  const int h = std::max(1, static_cast<int>(depth_32f.rows * f));
  // Centred. The `(cols - w) / 2` form leaves the patch one pixel left of centre
  // when the leftover is odd, which is the same convention cv::Rect centring uses
  // and is pinned by a test so that nobody "fixes" it into being off-centre the
  // other way.
  const int x0 = (depth_32f.cols - w) / 2;
  const int y0 = (depth_32f.rows - h) / 2;

  std::vector<double> usable;
  usable.reserve(static_cast<std::size_t>(w) * static_cast<std::size_t>(h));
  for (int y = y0; y < y0 + h; ++y) {
    const float * row = depth_32f.ptr<float>(y);
    for (int x = x0; x < x0 + w; ++x) {
      ++out.total;
      const float v = row[x];
      if (!std::isfinite(v) || !(v > 0.0F)) {
        ++out.bad;
      } else if (static_cast<double>(v) >= max_m) {
        ++out.clipped;
      } else {
        usable.push_back(static_cast<double>(v));
      }
    }
  }

  out.usable = usable.size();
  // One sort for the three, rather than three `percentile` calls — that function
  // takes its vector by value on purpose, and three copies of a 320x180 patch at
  // the depth rate is work this instrument has no reason to do inside the
  // container it is measuring. `quartiles` indexes with `percentile`'s own
  // `percentile_index`, and `test_stats` asserts the two agree.
  const pimesh_core::Quartiles q = pimesh_core::quartiles(std::move(usable));
  out.q1 = q.q1;
  out.median = q.median;
  out.q3 = q.q3;
  return out;
}

}  // namespace pimesh_depth

#endif  // PIMESH_DEPTH__DEPTH_PATCH_HPP_
