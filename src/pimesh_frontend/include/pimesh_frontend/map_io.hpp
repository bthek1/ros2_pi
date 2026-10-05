#ifndef PIMESH_FRONTEND__MAP_IO_HPP_
#define PIMESH_FRONTEND__MAP_IO_HPP_

#include <cstdint>
#include <deque>
#include <string>
#include <unordered_map>

#include "opencv2/core/affine.hpp"

#include "pimesh_frontend/keyframe_store.hpp"

namespace pimesh_frontend
{

/// #13's P20: a room's keyframes on disk, so a later session can recognise it.
///
/// **What is saved is the keyframe store, and each keyframe's pose is its pose in
/// the `map` frame** — written into `Keyframe::odom_from_camera`, because that is
/// the field place recognition reads a candidate's pose from. In the session that
/// saves, `map` is the pose graph's corrected frame (identity onto odom without loop
/// closure); in the session that loads, `map` *is* the saved session's frame, and
/// relocalising means finding where this session's odom sits inside it.
///
/// **The map points are not saved**, and that is a decision rather than an omission:
/// relocalisation verifies against each keyframe's own depth landmarks, which travel
/// in the keyframe, and the one reader the map points have — local-map tracking —
/// defaults off because P14 measured it worse than P7. A file section nothing reads
/// is where a wrong number lives undisturbed. See milestone-i-future.md.
///
/// The layout, all host byte order (both machines are little-endian):
///
///     "PIMESHKF"                       8 bytes
///     uint64 count
///     count x {
///       int64  stamp_ns
///       double rotation[9] (row-major), translation[3]
///       int32  descriptor rows, cols, OpenCV type (CV_8U only)
///       uint8  descriptors[rows * cols]
///       uint64 n, double bearings[n * 3]
///       uint64 n, int32 track_ids[n]
///       uint64 n, double landmarks[n * 3]
///       uint64 n, int32 landmark_row[n]
///     }
///
/// **No version field**, per milestone-i-future.md: one writer and one reader that
/// ship together, and a version branch nobody has taken is a branch nobody has
/// tested. What it does have is refusal: a missing file, a wrong magic, a short
/// read anywhere, an implausible count and **trailing bytes** each fail the load
/// with a reason and leave the caller's store untouched — because half a map loads
/// as a smaller map, and a smaller map is a plausible one.

/// The keyframe store as it is saved: each keyframe's `odom_from_camera` replaced by
/// its pose in the map frame — the pose graph's corrected pose where `corrected` has
/// one for its stamp, else `map_from_odom * odom_from_camera`.
///
/// **The two sources disagree exactly when loop closure has moved a keyframe**, and
/// that is the case the save exists for: a map saved at the odometry poses would be
/// the drifted room, relocalised into with confidence.
std::deque<Keyframe> keyframes_in_map(
  const std::deque<Keyframe> & frames, const cv::Affine3d & map_from_odom,
  const std::unordered_map<std::int64_t, cv::Affine3d> & corrected);

/// Write `frames` to `path`, through `path + ".partial"` and a rename so a session
/// killed mid-write leaves the previous map rather than half of this one. Returns ""
/// on success, else why not.
std::string save_keyframes(const std::string & path, const std::deque<Keyframe> & frames);

/// Read `path` into `out`, replacing it only if the whole file reads. Returns "" on
/// success, else why not.
std::string load_keyframes(const std::string & path, std::deque<Keyframe> & out);

}  // namespace pimesh_frontend

#endif  // PIMESH_FRONTEND__MAP_IO_HPP_
