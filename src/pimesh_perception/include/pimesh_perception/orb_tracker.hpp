#ifndef PIMESH_PERCEPTION__ORB_TRACKER_HPP_
#define PIMESH_PERCEPTION__ORB_TRACKER_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"

namespace pimesh_perception
{

/// One corner, seen twice: where it was in the previous frame and where it is now.
struct PixelPair
{
  cv::Point2f previous;
  cv::Point2f current;
};

/// One frame's worth of ORB output, plus who each feature turned out to be.
struct TrackedFrame
{
  std::vector<cv::KeyPoint> keypoints;
  /// Row i is the 32-byte descriptor of keypoints[i]. CV_8U, 32 columns.
  cv::Mat descriptors;

  /// The track each feature belongs to. Always >= 0 — a feature seen for the
  /// first time is given a fresh id here, because the *next* frame has to be able
  /// to inherit it. This is not what goes on the wire; see published_track_ids().
  std::vector<std::int32_t> ids;
  /// True where this frame is the first sighting of that track.
  std::vector<bool> is_new;

  /// Where the time went, in milliseconds on a steady clock: detection, then both
  /// matching passes. Reported rather than inferred because the two are very
  /// different costs with very different fixes — detection scales with the feature
  /// cap, matching with the cap *times* the window — and a single 7 ms number
  /// cannot tell you which knob to turn.
  double detect_ms {0.0};
  double match_ms {0.0};

  /// Pixel pairs matched against the *immediately previous* frame: where a corner
  /// was, and where it is now. Separate from the ids above on purpose — see
  /// OrbTracker's class comment.
  ///
  /// Pixels rather than a pair of indices, and that is not a convenience. An index
  /// into the previous frame is only meaningful to a holder of that frame, so the
  /// index form obliges every caller to keep its own copy of the last
  /// TrackedFrame and to keep it in step with the tracker's window. Two copies of
  /// the same bookkeeping drift, and when they do the pairs silently refer to the
  /// wrong corners — a rotation fit on mismatched pairs does not fail, it returns
  /// a wrong answer with a plausible residual.
  std::vector<PixelPair> consecutive_pairs;

  std::size_t matched() const
  {
    std::size_t n = 0;
    for (bool fresh : is_new) {if (!fresh) {++n;}}
    return n;
  }

  /// The fraction of this frame's features that were already being followed. The
  /// headline number for "is the tracker working": it collapses when the camera
  /// moves too fast, when the room is blank, and when the descriptor threshold is
  /// wrong, and it is stable across all three when they are right.
  double matched_fraction() const
  {
    return keypoints.empty() ?
           0.0 : static_cast<double>(matched()) / static_cast<double>(keypoints.size());
  }

  /// Track ids in the convention pimesh_msgs/Keypoints documents: the track's id
  /// for a feature that was already being followed, **-1 for one seen here for the
  /// first time**. A consumer drawing tracks needs that distinction; the internal
  /// `ids` vector cannot express it.
  std::vector<std::int32_t> published_track_ids() const
  {
    std::vector<std::int32_t> out(ids.size());
    for (std::size_t i = 0; i < ids.size(); ++i) {out[i] = is_new[i] ? -1 : ids[i];}
    return out;
  }
};

/// ORB detection plus two kinds of matching, which is the part worth explaining.
///
/// **Pooled matching, for tracks.** A feature is matched against every frame in a
/// window of the last N, not just against the previous one. Strict frame-to-frame
/// matching loses ~25% of keypoints to *detection churn* rather than to motion:
/// at a 500-feature cap, whether a corner comes in at rank 495 or 505 is decided
/// by sensor noise, so features flicker out for a frame and back. A window
/// forgives the flicker, and is the toy version of what a SLAM system calls
/// matching against a local map.
///
/// **Consecutive-pair matching, for geometry.** The rotation estimate wants pairs
/// that are one frame apart and nothing else: a match to a frame five back spans
/// five times the motion and would be weighted as if it spanned one. So the same
/// descriptors are matched a second time against the previous frame alone — and
/// mutually, each the other's best — and those pairs, not the pooled ones, go to
/// the fit. Conflating the two is a tempting saving that quietly makes the
/// residual gate meaningless.
///
/// A lookalike corner is worse than no corner, so both matchers reject anything
/// past a Hamming distance of 64 bits in 256. That threshold is absolute rather
/// than a ratio test: descriptors of genuinely unrelated corners sit near 128 bits
/// apart, so 64 is already generous.
///
/// **Matching is made one-to-one.** A plain nearest-neighbour search is
/// many-to-one: two corners in this frame can both name the same older feature as
/// their best match, and both would then inherit the same track id — one track
/// with two simultaneous positions, which is a contradiction the geometry has no
/// way to notice. Candidates are taken in order of distance and each older track
/// is claimed once.
///
/// **Nothing in this class touches an OpenCV API that differs between 4.6 and
/// 4.10.** `ORB::create`, `BFMatcher` and `knnMatch` are identical on both; the
/// APIs that are not, and which bit this project once already, are in `cv::aruco`.
/// An API that exists at both ends and means different things is worse than one
/// that is missing at one end, because the missing one fails loudly.
class OrbTracker
{
public:
  struct Config
  {
    int max_features {500};
    /// Frames in the pooled window, the current one excluded.
    std::size_t match_window {10};
    /// Hamming bits, of 256.
    int max_distance {64};
  };

  explicit OrbTracker(const Config & config);

  /// Detect, match, assign ids. `gray` must be CV_8UC1.
  ///
  /// Call it once per frame, in order: the window is this object's state.
  TrackedFrame process(const cv::Mat & gray);

  /// Ids handed out so far, which is also the number of distinct features this
  /// session has ever seen. Ids are never reused.
  std::int32_t tracks_created() const {return next_id_;}

  std::size_t window_size() const {return window_.size();}

  void reset();

private:
  Config config_;
  cv::Ptr<cv::ORB> orb_;
  cv::Ptr<cv::BFMatcher> matcher_;
  std::deque<TrackedFrame> window_;
  std::int32_t next_id_ {0};
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__ORB_TRACKER_HPP_
