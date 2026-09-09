#ifndef PIMESH_CAMERA__STAMP_HPP_
#define PIMESH_CAMERA__STAMP_HPP_

#include <cstdint>

namespace pimesh_camera
{

/// Where a published stamp came from, so the caller can say so and a test can
/// assert on it.
enum class StampSource
{
  /// The kernel's capture time, carried across to the ROS clock. The good case,
  /// and what this node exists to produce.
  kCaptureTime,
  /// The driver did not report a monotonic buffer timestamp, so the stamp is
  /// the moment of dequeue. Still honest to within a frame, but it no longer
  /// carries the capture time and the node says so.
  kNoMonotonicClock,
  /// The buffer's timestamp implied an age that cannot be real. Falls back to
  /// dequeue rather than publishing a confidently wrong number.
  kImplausibleAge,
};

struct Stamp
{
  std::int64_t nanoseconds {0};
  StampSource source {StampSource::kCaptureTime};
};

/// Convert a V4L2 buffer's CLOCK_MONOTONIC capture time to a ROS-clock stamp.
///
/// ============================================================
/// This function is the reason `pimesh_camera` exists instead of `usb_cam`.
/// ============================================================
///
/// The frame was captured at `frame_monotonic_ns` on CLOCK_MONOTONIC. The stamp
/// has to be on the ROS clock, which is the system clock. Those are two
/// different epochs, and converting between them is where `usb_cam` 0.8.1 goes
/// wrong: it computes the offset between the two clocks **once per process**
/// and adds it to every frame forever. Every stamp in a session is then
/// displaced by the same random sub-second amount — measured at 0.223, 0.362
/// and 0.979 s on three launches of the same binary — redrawn at each launch.
/// That is what makes a stamp-age freshness gate drop 100% of frames.
///
/// The fix is to stop converting epochs at all and convert an *interval*
/// instead:
///
///     stamp = ros_now - (monotonic_now - frame_monotonic)
///
/// Both "now" readings are taken microseconds apart on the same machine, so the
/// clocks' relative drift over that gap is irrelevant, and — the load-bearing
/// part — **the result does not depend on the offset between the two epochs at
/// all.** There is no per-process constant left that could be wrong. That
/// property is what `tools/gates/capture.sh` checks by launching the node twice,
/// and what `test_stamp.cpp` checks directly by feeding in wildly different
/// epoch relationships and asserting the answer does not move.
///
/// It is a free function taking every clock reading as an argument, rather than
/// a method that calls `now()`, for exactly one reason: a claim this central
/// should be testable without a camera, a network or a ROS context.
///
/// \param ros_now_ns          the ROS clock, read at dequeue
/// \param monotonic_now_ns    CLOCK_MONOTONIC, read at the same moment
/// \param frame_monotonic_ns  the buffer's own timestamp
/// \param monotonic_valid     did the driver flag it as CLOCK_MONOTONIC?
/// \param max_plausible_age_ns  above this, distrust the buffer's clock
inline Stamp stamp_from_capture(
  std::int64_t ros_now_ns,
  std::int64_t monotonic_now_ns,
  std::int64_t frame_monotonic_ns,
  bool monotonic_valid,
  std::int64_t max_plausible_age_ns = 500000000LL)
{
  if (!monotonic_valid) {
    return Stamp{ros_now_ns, StampSource::kNoMonotonicClock};
  }

  const std::int64_t age_ns = monotonic_now_ns - frame_monotonic_ns;

  // A negative age means the buffer is stamped in the future; an age of seconds
  // means this is not the clock the flags claim. Either way the subtraction
  // would produce a confidently wrong stamp, which is worse than a slightly
  // late one. Half a second is far outside anything a 4-buffer pool at 47 Hz
  // can produce (~85 ms at the very worst).
  if (age_ns < 0 || age_ns > max_plausible_age_ns) {
    return Stamp{ros_now_ns, StampSource::kImplausibleAge};
  }

  return Stamp{ros_now_ns - age_ns, StampSource::kCaptureTime};
}

}  // namespace pimesh_camera

#endif  // PIMESH_CAMERA__STAMP_HPP_
