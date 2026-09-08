// ORB detection and two different kinds of matching, with no ROS in it.
//
// One frame in, one TrackedFrame out. The class holds the only state that
// makes tracking possible — the previous frame, and a window of recent
// descriptors — so the node above it stays a thin shell that moves messages.
//
// **There are two matchings here and they answer different questions.** Getting
// them confused is the subtle bug this file exists to prevent:
//
//   * **Strict, against the PREVIOUS frame only.** "Where did this exact point
//     move between these two frames?" That is the only question motion can be
//     estimated from, so it feeds `match_index`, the track ids, and the
//     rotation estimate. A two-frame question needs a two-frame answer.
//
//   * **Pooled, against a window of the last N frames.** "Have I seen this
//     point recently?" Frame-to-frame matching alone loses ~25% of keypoints
//     to *detection churn*: whether a corner lands at rank #95 or #105 against
//     a 500-feature cap is decided by sensor noise, so features flicker out
//     for a frame and back. Pooling over ten frames forgives the flicker — the
//     toy version of what SLAM calls matching against a local map — and the
//     predecessor measured it holding ~90% matched where consecutive-only sat
//     around 65%. It drives the preview colours and the matched fraction.
//
// Using the pooled result for odometry would be the real trap: a "match" into
// a frame six frames back carries six frames of motion, and feeding that to an
// estimator that assumes one frame of it produces a confident wrong answer.

#ifndef PIMESH_PERCEPTION__ORB_TRACKER_HPP_
#define PIMESH_PERCEPTION__ORB_TRACKER_HPP_

#include <cstdint>
#include <deque>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace pimesh_perception
{

/// One frame's features and everything matching learned about them.
/// The vectors are parallel and all of length `keypoints.size()`.
struct TrackedFrame
{
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;                  ///< count x 32, CV_8U; empty if none

  /// Index into the PREVIOUS frame's arrays, or -1 when unmatched there.
  std::vector<int32_t> match_index;
  /// Persistent id for a point followed across frames. A feature that matches
  /// the previous frame inherits its id; anything else mints a new one, so an
  /// id's lifetime is exactly how long the tracker held on to that point.
  std::vector<int32_t> track_id;
  /// Matched somewhere in the pooled window — "seen recently", not "new".
  std::vector<uint8_t> recently_seen;

  /// Pixel coordinates of the strict consecutive pairs, parallel to each
  /// other: prev_matched[i] in the previous frame is curr_matched[i] here.
  /// This is the rotation estimator's entire input.
  std::vector<cv::Point2f> prev_matched;
  std::vector<cv::Point2f> curr_matched;

  std::size_t recent_match_count{0};    ///< how many were seen in the window

  std::size_t count() const {return keypoints.size();}
  /// Fraction of this frame's features already seen in the window, in [0, 1].
  /// The number `just gate-keypoints` compares against the predecessor's ~0.90.
  double matched_fraction() const
  {
    return keypoints.empty() ?
           0.0 : static_cast<double>(recent_match_count) / static_cast<double>(keypoints.size());
  }
};

class OrbTracker
{
public:
  struct Options
  {
    /// Feature cap. 500 is enough for a dense-enough match set and cheap
    /// enough to run on every frame at camera rate; the cost is roughly
    /// linear in this, and the pooled match is quadratic-ish in it.
    int max_features{500};
    /// Image-pyramid levels ORB detects on, and the scale step between them.
    ///
    /// This is the cheapest real knob in the stage. Eight levels (OpenCV's
    /// default) cost 9.6 ms/frame at 1280x720 on this box; four cost 6.8 ms
    /// (measured 2026-09-07). What the extra levels buy is SCALE invariance —
    /// recognising the same corner when it is twice as big — and between two
    /// consecutive frames 30 ms apart there is almost no scale change to be
    /// invariant to. Four levels at 1.2 still span 1.73x, which covers a
    /// hand-held sweep's approach and retreat.
    ///
    /// It is a knob and not a constant because the calculation changes the day
    /// something matches across a wide baseline — P7's keyframes, or loop
    /// closure — where scale invariance is the whole point.
    int pyramid_levels{4};
    double scale_factor{1.2};
    /// How many recent frames the pooled match sees. 1 would be
    /// consecutive-only and would show the churn described above.
    int window{10};
    /// Hamming bits (of 256) above which a match is rejected as "a
    /// similar-looking corner" rather than "the same physical point". 64 is
    /// the predecessor's, and is a quarter of the descriptor.
    int max_distance{64};
  };

  explicit OrbTracker(const Options & options);

  /// Detect, match both ways, and advance the state by one frame.
  /// `gray` must be single-channel: ORB detects on intensity, and handing it
  /// BGR costs a conversion inside OpenCV on every call.
  TrackedFrame track(const cv::Mat & gray);

  /// Forget everything. The next frame becomes a first frame — no previous
  /// frame, an empty window, and therefore no matches and no rotation.
  void reset();

  const Options & options() const {return options_;}

private:
  Options options_;
  cv::Ptr<cv::ORB> orb_;
  cv::Ptr<cv::BFMatcher> matcher_;

  /// The previous frame, kept in pixel+descriptor form because the strict
  /// match needs the two aligned and cv::KeyPoint carries baggage we do not.
  std::vector<cv::Point2f> prev_points_;
  cv::Mat prev_descriptors_;
  std::vector<int32_t> prev_track_id_;

  /// The pooled window. Descriptor blocks, oldest first.
  std::deque<cv::Mat> window_;

  int32_t next_track_id_{0};
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__ORB_TRACKER_HPP_
