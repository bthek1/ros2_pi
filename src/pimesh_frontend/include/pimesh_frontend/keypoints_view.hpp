#ifndef PIMESH_PERCEPTION__KEYPOINTS_VIEW_HPP_
#define PIMESH_PERCEPTION__KEYPOINTS_VIEW_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include "opencv2/core.hpp"
#include "pimesh_msgs/msg/keypoints.hpp"
#include "pimesh_frontend/orb_tracker.hpp"

namespace pimesh_frontend
{

/// Reading a `pimesh_msgs/Keypoints` back into the things geometry works with.
///
/// **This exists as a header because it was three loops inside a subscription
/// callback, and that is a place no test can reach.** Every distance the pose
/// estimator fits comes through here — the corner positions, the track ids they
/// are matched by, the descriptors a keyframe keeps, and the one-frame-apart pairs
/// a rotation is fitted to. It is the `image_buffer.hpp` lesson arriving for a
/// fourth time: the question to ask of a helper is not whether it is interesting
/// but whether a test could call it if it wanted to.
///
/// **Two bugs were in the callback version when it moved here, 2026-09-23, and
/// both are the same shape:** a loop bounded by one array's length while indexing
/// a *different* array. `x` was the bound and `y` was indexed; `prev_x` was the
/// bound and `prev_y` was indexed. A message whose parallel arrays are not all the
/// same length therefore read past the end of a `std::vector` — and since the
/// message shape is a contract nothing enforces at runtime, the only thing keeping
/// that from happening was that one publisher in this workspace happens to fill
/// them consistently.
///
/// The functions below are **total**: every one clamps to the shortest array it
/// touches, so none of them can read out of bounds however malformed the input.
/// That is deliberately *not* the whole answer — silently truncating is how you
/// get a plausible wrong number — so `well_formed()` is the predicate a caller
/// checks first, and `odometry_node` counts and reports what it refuses. Two lines
/// of defence, because the clamp alone would hide the very thing the predicate
/// exists to announce.

/// Every per-feature array the message documents as parallel is the same length,
/// and the descriptor blob is exactly `descriptor_bytes` per feature.
///
/// `descriptor_bytes == 0` with an empty blob is well formed: it is what a frame
/// in which ORB found nothing looks like.
inline bool keypoints_well_formed(const pimesh_msgs::msg::Keypoints & msg)
{
  const std::size_t n = msg.x.size();
  if (msg.y.size() != n || msg.size.size() != n || msg.angle.size() != n ||
    msg.response.size() != n || msg.track_id.size() != n || msg.is_new.size() != n ||
    msg.prev_x.size() != n || msg.prev_y.size() != n)
  {
    return false;
  }
  return msg.descriptors.size() == n * static_cast<std::size_t>(msg.descriptor_bytes);
}

/// The number of features every per-feature accessor below is safe to index.
inline std::size_t keypoints_count(const pimesh_msgs::msg::Keypoints & msg)
{
  return std::min(msg.x.size(), msg.y.size());
}

/// Where each corner is, in the image the message names.
inline std::vector<cv::Point2f> keypoint_pixels(const pimesh_msgs::msg::Keypoints & msg)
{
  const std::size_t n = keypoints_count(msg);
  std::vector<cv::Point2f> pixels(n);
  for (std::size_t i = 0; i < n; ++i) {
    pixels[i] = cv::Point2f(msg.x[i], msg.y[i]);
  }
  return pixels;
}

/// The descriptors, in a `cv::Mat` over **its own storage**.
///
/// Not `mat_over`-style aliasing, and the difference is a lifetime rather than a
/// copy count. The message arrives as a shared const pointer that the subscription
/// releases when the callback returns; a Mat header over its bytes outlives them
/// in the history deque, which is a use-after-free that reads as *plausible
/// descriptors* for as long as the allocator leaves the page alone. 16 kB a frame
/// is the price of that not being a possibility.
///
/// Empty when the blob does not match the feature count, because a descriptor
/// matrix built from a wrong-length blob would be rows of somebody else's bytes.
inline cv::Mat copy_keypoint_descriptors(const pimesh_msgs::msg::Keypoints & msg)
{
  const std::size_t n = msg.x.size();
  const std::size_t stride = static_cast<std::size_t>(msg.descriptor_bytes);
  if (stride == 0 || n == 0 || msg.descriptors.size() != n * stride) {
    return cv::Mat();
  }
  cv::Mat out(static_cast<int>(n), static_cast<int>(stride), CV_8U);
  std::memcpy(out.data, msg.descriptors.data(), msg.descriptors.size());
  return out;
}

/// The pairs a rotation is fitted to: where a corner was one frame ago, and where
/// it is now.
///
/// **This is the mutual-best pairing the detector reported, not the pooled
/// window's.** A consumer that rebuilt it by intersecting two frames' `track_id`
/// arrays would get the looser pairing, and a rotation fit on looser pairs does
/// not fail — it returns a wrong answer with a plausible residual. Features whose
/// `prev_*` is the NaN hole were not mutually matched and are skipped.
inline std::vector<PixelPair> keypoint_consecutive_pairs(
  const pimesh_msgs::msg::Keypoints & msg)
{
  const std::size_t n = std::min(
    keypoints_count(msg), std::min(msg.prev_x.size(), msg.prev_y.size()));
  std::vector<PixelPair> pairs;
  pairs.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const cv::Point2f previous(msg.prev_x[i], msg.prev_y[i]);
    if (!TrackedFrame::matched_previous(previous)) {continue;}
    pairs.push_back(PixelPair{previous, cv::Point2f(msg.x[i], msg.y[i])});
  }
  return pairs;
}

}  // namespace pimesh_frontend

#endif  // PIMESH_PERCEPTION__KEYPOINTS_VIEW_HPP_
