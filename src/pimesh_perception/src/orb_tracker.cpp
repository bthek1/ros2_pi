#include "pimesh_perception/orb_tracker.hpp"

#include <algorithm>
#include <numeric>

namespace pimesh_perception
{

OrbTracker::OrbTracker(const Options & options)
: options_(options)
{
  orb_ = cv::ORB::create(
    options_.max_features, static_cast<float>(options_.scale_factor),
    options_.pyramid_levels);
  // Brute force is the right matcher at 500 features: an index structure costs
  // more to build per frame than the scan it saves. NORM_HAMMING because ORB
  // descriptors are 256-bit strings, not float vectors — an L2 distance over
  // them is meaningless.
  //
  // crossCheck keeps only MUTUAL best matches: a pair survives only if each is
  // the other's nearest neighbour. That is one-to-one by construction and
  // prunes most false pairings before any geometry is involved — it is the
  // cheap alternative to Lowe's ratio test, and it is what lets the refit
  // rounds in estimate_rotation() get away with only two passes.
  matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, /*crossCheck=*/true);
}

void OrbTracker::reset()
{
  prev_points_.clear();
  prev_descriptors_.release();
  prev_track_id_.clear();
  window_.clear();
  // Track ids are NOT reset: an id must never be reused for a different
  // physical point, or a consumer holding an old id silently follows the
  // wrong thing after a reset.
}

TrackedFrame OrbTracker::track(const cv::Mat & gray)
{
  TrackedFrame out;

  // detectAndCompute, not detect: matching needs descriptors, and computing
  // them separately can silently drop keypoints whose 31x31 descriptor patch
  // falls off the image edge — leaving the keypoint and descriptor arrays
  // misaligned, which is a wrong-answer bug rather than a crash.
  orb_->detectAndCompute(gray, cv::noArray(), out.keypoints, out.descriptors);

  const std::size_t n = out.keypoints.size();
  out.match_index.assign(n, -1);
  out.track_id.assign(n, -1);
  out.recently_seen.assign(n, 0);

  std::vector<cv::Point2f> points(n);
  for (std::size_t i = 0; i < n; ++i) {
    points[i] = out.keypoints[i].pt;
  }

  const bool have_descriptors = !out.descriptors.empty();

  // ---- pooled match: "seen recently?" -------------------------------------
  //
  // The window's blocks are concatenated into one train matrix. The same
  // physical feature appears once per frame in there, but crossCheck still
  // yields at most one match per query keypoint — which is all a yes/no
  // question needs.
  if (have_descriptors && !window_.empty()) {
    cv::Mat train;
    cv::vconcat(std::vector<cv::Mat>(window_.begin(), window_.end()), train);

    std::vector<cv::DMatch> matches;
    matcher_->match(out.descriptors, train, matches);
    for (const auto & m : matches) {
      if (m.distance <= static_cast<float>(options_.max_distance)) {
        out.recently_seen[static_cast<std::size_t>(m.queryIdx)] = 1;
      }
    }
    out.recent_match_count = static_cast<std::size_t>(
      std::count(out.recently_seen.begin(), out.recently_seen.end(), uint8_t{1}));
  }

  // ---- strict match: "where did this point move?" -------------------------
  if (have_descriptors && !prev_descriptors_.empty()) {
    std::vector<cv::DMatch> pairs;
    matcher_->match(out.descriptors, prev_descriptors_, pairs);
    out.prev_matched.reserve(pairs.size());
    out.curr_matched.reserve(pairs.size());
    for (const auto & m : pairs) {
      if (m.distance > static_cast<float>(options_.max_distance)) {
        continue;
      }
      const auto q = static_cast<std::size_t>(m.queryIdx);
      const auto t = static_cast<std::size_t>(m.trainIdx);
      out.match_index[q] = static_cast<int32_t>(t);
      // A matched feature IS the previous one, so it keeps that identity.
      out.track_id[q] = prev_track_id_[t];
      out.prev_matched.push_back(prev_points_[t]);
      out.curr_matched.push_back(points[q]);
    }
  }

  for (std::size_t i = 0; i < n; ++i) {
    if (out.track_id[i] < 0) {
      out.track_id[i] = next_track_id_++;
    }
  }

  // ---- advance the state --------------------------------------------------
  //
  // No clone. cv::Mat is a refcounted header over shared pixel data, so both
  // the window and the previous-frame slot hold the SAME 16 kB of descriptors
  // the returned TrackedFrame carries — and it stays alive exactly as long as
  // something still refers to it. detectAndCompute writes into a fresh Mat on
  // the next call, so nothing here is ever overwritten underneath us.
  if (have_descriptors) {
    window_.push_back(out.descriptors);
    while (window_.size() > static_cast<std::size_t>(std::max(1, options_.window))) {
      window_.pop_front();
    }
  }
  prev_points_ = std::move(points);
  prev_descriptors_ = out.descriptors;
  prev_track_id_ = out.track_id;

  return out;
}

}  // namespace pimesh_perception
