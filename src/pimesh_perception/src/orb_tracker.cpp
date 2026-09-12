#include "pimesh_perception/orb_tracker.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <unordered_set>
#include <utility>
#include <vector>

namespace pimesh_perception
{
namespace
{

/// One candidate match, before any of them have been accepted.
struct Candidate
{
  int query {0};        ///< index into this frame's keypoints
  int target {0};       ///< index into the pooled descriptor rows
  float distance {0.0F};
};

}  // namespace

OrbTracker::OrbTracker(const Config & config)
: config_(config)
{
  // The defaults, spelled out rather than left to ORB::create()'s signature,
  // because two of them are decisions.
  //
  // `nlevels = 8` with `scaleFactor = 1.2`: the pyramid spans a factor of ~3.6 in
  // scale, which is what makes a corner still matchable after the camera has
  // moved towards it. `WTA_K = 2` is what makes the descriptor comparable by
  // Hamming distance at all — the 3 and 4 variants produce descriptors that need
  // a different metric, and a matcher configured for Hamming on one of those
  // returns confident nonsense.
  orb_ = cv::ORB::create(
    config_.max_features,
    1.2F,     // scaleFactor
    8,        // nlevels
    31,       // edgeThreshold
    0,        // firstLevel
    2,        // WTA_K
    cv::ORB::HARRIS_SCORE,
    31,       // patchSize
    20);      // fastThreshold

  // NORM_HAMMING because an ORB descriptor is 256 bits, not a vector of numbers:
  // the distance between two of them is how many bits differ. NORM_L2 on these
  // bytes computes something real and meaningless.
  //
  // crossCheck stays *off* here, and that is deliberate. BFMatcher's crossCheck
  // is incompatible with knnMatch, and the one-to-one guarantee this class needs
  // is stronger than cross-checking anyway: it is enforced below, over the whole
  // pooled window at once, where cross-check would only relate one pair of frames.
  matcher_ = cv::makePtr<cv::BFMatcher>(cv::NORM_HAMMING, false);
}

void OrbTracker::reset()
{
  window_.clear();
  // next_id_ is *not* reset: ids are unique within a session, and a reset that
  // reused them would make two unrelated tracks indistinguishable to anything
  // that had recorded the first.
}

TrackedFrame OrbTracker::process(const cv::Mat & gray)
{
  TrackedFrame frame;

  const auto t0 = std::chrono::steady_clock::now();
  orb_->detectAndCompute(gray, cv::noArray(), frame.keypoints, frame.descriptors);
  const auto t1 = std::chrono::steady_clock::now();
  frame.detect_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  const std::size_t count = frame.keypoints.size();
  frame.ids.assign(count, -1);
  frame.is_new.assign(count, true);

  // --- The pooled window, as one matrix ---------------------------------------
  //
  // Concatenating the window's descriptors and matching once is equivalent to
  // matching against each frame and keeping the best, and it is one BFMatcher
  // call instead of ten. `pooled_ids` maps a row back to the track it belongs to.
  cv::Mat pooled;
  std::vector<std::int32_t> pooled_ids;
  if (!frame.descriptors.empty()) {
    for (const TrackedFrame & past : window_) {
      if (past.descriptors.empty()) {continue;}
      pooled.push_back(past.descriptors);
      pooled_ids.insert(pooled_ids.end(), past.ids.begin(), past.ids.end());
    }
  }

  if (!pooled.empty()) {
    std::vector<std::vector<cv::DMatch>> knn;
    matcher_->knnMatch(frame.descriptors, pooled, knn, 1);

    std::vector<Candidate> candidates;
    candidates.reserve(knn.size());
    for (std::size_t i = 0; i < knn.size(); ++i) {
      if (knn[i].empty()) {continue;}
      const cv::DMatch & m = knn[i][0];
      if (m.distance > static_cast<float>(config_.max_distance)) {continue;}
      candidates.push_back({static_cast<int>(i), m.trainIdx, m.distance});
    }

    // Closest first, and each track claimed once. Taking them in arrival order
    // instead would hand a track to whichever feature happened to be detected
    // earlier, which is an ordering decided by the detector's scan pattern.
    std::sort(
      candidates.begin(), candidates.end(),
      [](const Candidate & a, const Candidate & b) {return a.distance < b.distance;});

    // Claimed by *track*, not by row: one track appears in as many pooled rows as
    // it has frames in the window, so de-duplicating rows would still let two
    // features inherit one track through two different sightings of it.
    std::unordered_set<std::int32_t> claimed;
    for (const Candidate & candidate : candidates) {
      if (!frame.is_new[static_cast<std::size_t>(candidate.query)]) {continue;}
      const std::int32_t track = pooled_ids[static_cast<std::size_t>(candidate.target)];
      if (!claimed.insert(track).second) {continue;}

      frame.ids[static_cast<std::size_t>(candidate.query)] = track;
      frame.is_new[static_cast<std::size_t>(candidate.query)] = false;
    }
  }

  // Every unmatched feature starts a track of its own. It may never be seen
  // again, which costs one int — and is what lets the *next* frame match it.
  for (std::size_t i = 0; i < count; ++i) {
    if (frame.is_new[i]) {frame.ids[i] = next_id_++;}
  }

  // --- Consecutive pairs, for the geometry ------------------------------------
  //
  // Mutual best match against the previous frame alone: each of the two has to be
  // the other's nearest. That is stricter than the pooled pass on purpose — a
  // wrong pair here does not cost a track id, it tilts a rotation estimate, and
  // the residual gate downstream cannot tell a wrong pair from real motion.
  if (!window_.empty() && !frame.descriptors.empty() && !window_.back().descriptors.empty()) {
    const cv::Mat & prev = window_.back().descriptors;

    std::vector<std::vector<cv::DMatch>> forward;
    std::vector<std::vector<cv::DMatch>> backward;
    matcher_->knnMatch(frame.descriptors, prev, forward, 1);
    matcher_->knnMatch(prev, frame.descriptors, backward, 1);

    for (std::size_t i = 0; i < forward.size(); ++i) {
      if (forward[i].empty()) {continue;}
      const cv::DMatch & f = forward[i][0];
      if (f.distance > static_cast<float>(config_.max_distance)) {continue;}
      const std::size_t back = static_cast<std::size_t>(f.trainIdx);
      if (back >= backward.size() || backward[back].empty()) {continue;}
      if (backward[back][0].trainIdx != static_cast<int>(i)) {continue;}
      frame.consecutive_pairs.push_back(
        PixelPair{
          window_.back().keypoints[back].pt,
          frame.keypoints[i].pt});
    }
  }

  frame.match_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t1).count();

  window_.push_back(frame);
  while (window_.size() > config_.match_window) {window_.pop_front();}

  return frame;
}

}  // namespace pimesh_perception
