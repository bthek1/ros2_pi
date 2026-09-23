#include "pimesh_mapping/scale_aligner.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace pimesh_mapping
{
namespace
{

/// Median by partial sort, taking the vector by value because it reorders it.
///
/// The lower of the two middles on an even count rather than their average, the
/// same convention `percentile` takes and for the same reason:
/// every value this returns is one some pixel actually had.
double median_of(std::vector<double> values)
{
  if (values.empty()) {return 0.0;}
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
  return values[mid];
}

/// Walk two 32FC1 images together, calling `f(expected, incoming)` where both are
/// finite and positive, and report the valid fraction.
///
/// Both callers need exactly this and the pixel validity rule is the part worth
/// having in one place: a zero from the ray-caster means "nothing there" and a
/// zero or a NaN from the depth map means "no reading", and treating either as a
/// distance is how a ratio of 0 gets into a median.
template<typename Fn>
bool walk_overlap(
  const cv::Mat & expected_m, const cv::Mat & incoming_m,
  double min_overlap, double & overlap, Fn && f)
{
  overlap = 0.0;
  if (expected_m.empty() || incoming_m.empty()) {return false;}
  if (expected_m.size() != incoming_m.size()) {return false;}
  if (expected_m.type() != CV_32FC1 || incoming_m.type() != CV_32FC1) {return false;}

  std::size_t valid = 0;
  const std::size_t total = static_cast<std::size_t>(expected_m.total());
  for (int v = 0; v < expected_m.rows; ++v) {
    const float * e = expected_m.ptr<float>(v);
    const float * i = incoming_m.ptr<float>(v);
    for (int u = 0; u < expected_m.cols; ++u) {
      if (!std::isfinite(e[u]) || !std::isfinite(i[u]) || e[u] <= 0.0F || i[u] <= 0.0F) {
        continue;
      }
      ++valid;
      f(static_cast<double>(e[u]), static_cast<double>(i[u]));
    }
  }
  overlap = total ? static_cast<double>(valid) / static_cast<double>(total) : 0.0;
  return overlap >= min_overlap && valid > 0;
}

}  // namespace

bool depth_ratio(
  const cv::Mat & expected_m, const cv::Mat & incoming_m,
  double min_overlap, double & ratio, double & overlap)
{
  std::vector<double> ratios;
  ratios.reserve(static_cast<std::size_t>(expected_m.total()));
  const bool enough = walk_overlap(
    expected_m, incoming_m, min_overlap, overlap,
    [&ratios](double e, double i) {ratios.push_back(e / i);});
  if (!enough) {
    ratio = 1.0;
    return false;
  }
  ratio = median_of(std::move(ratios));
  return true;
}

bool surface_gap(
  const cv::Mat & expected_m, const cv::Mat & incoming_m,
  double min_overlap, double & gap_m, double & overlap)
{
  std::vector<double> gaps;
  gaps.reserve(static_cast<std::size_t>(expected_m.total()));
  const bool enough = walk_overlap(
    expected_m, incoming_m, min_overlap, overlap,
    [&gaps](double e, double i) {gaps.push_back(std::fabs(e - i));});
  if (!enough) {
    gap_m = 0.0;
    return false;
  }
  gap_m = median_of(std::move(gaps));
  return true;
}

bool surface_agreement(
  const cv::Mat & expected_m, const cv::Mat & incoming_m,
  double tolerance, double & fraction)
{
  fraction = 0.0;
  if (expected_m.empty() || incoming_m.empty()) {return false;}
  if (expected_m.size() != incoming_m.size()) {return false;}
  if (expected_m.type() != CV_32FC1 || incoming_m.type() != CV_32FC1) {return false;}

  std::size_t valid = 0;
  std::size_t agree = 0;
  for (int v = 0; v < expected_m.rows; ++v) {
    const float * e = expected_m.ptr<float>(v);
    const float * i = incoming_m.ptr<float>(v);
    for (int u = 0; u < expected_m.cols; ++u) {
      // The denominator: every pixel the *incoming frame* has a reading for. A
      // pixel where the map holds nothing counts as a disagreement, which is the
      // honest reading of "the map does not yet agree with this view".
      if (!std::isfinite(i[u]) || i[u] <= 0.0F) {continue;}
      ++valid;
      if (!std::isfinite(e[u]) || e[u] <= 0.0F) {continue;}
      if (std::fabs(static_cast<double>(e[u] - i[u])) <= tolerance * static_cast<double>(i[u])) {
        ++agree;
      }
    }
  }
  if (valid == 0) {return false;}
  fraction = static_cast<double>(agree) / static_cast<double>(valid);
  return true;
}

ScaleResult ScaleAligner::scale_for(const cv::Mat & expected_m, const cv::Mat & incoming_m)
{
  ScaleResult result;
  double ratio = 1.0;
  double overlap = 0.0;
  const bool ok = depth_ratio(expected_m, incoming_m, options_.min_overlap, ratio, overlap);
  result.overlap = overlap;
  if (!ok || !std::isfinite(ratio) || ratio <= 0.0) {
    // Scale 1.0 and `aligned` false: the frame goes in unmodified. **The history
    // is deliberately not updated here** — a frame with no overlap has no opinion
    // about the scale, and letting it push a 1.0 into the window would drag the
    // baseline towards the identity every time the camera looked somewhere new.
    return result;
  }

  result.ratio = ratio;
  ratios_.push_back(ratio);
  while (ratios_.size() > options_.window) {ratios_.pop_front();}

  const double baseline = median_of(std::vector<double>(ratios_.begin(), ratios_.end()));
  result.baseline = baseline;
  if (!(baseline > 0.0)) {return result;}

  // The high-pass: what this frame says divided by what the recent frames have
  // been saying. A frame that agrees with the baseline gets exactly 1.
  const double raw = ratio / baseline;
  const double low = 1.0 - options_.max_correction;
  const double high = 1.0 + options_.max_correction;
  result.scale = std::min(high, std::max(low, raw));
  result.clamped = raw < low || raw > high;
  result.aligned = true;
  return result;
}

}  // namespace pimesh_mapping
