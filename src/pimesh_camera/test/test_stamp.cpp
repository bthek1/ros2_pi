// The stamp arithmetic, which is the one claim pimesh_camera really makes.
//
// gates/capture.sh tests this end to end by launching the node twice against a
// real camera and comparing the two offsets. That test is the one that counts,
// and it needs a Pi, a C922 and a working LAN. These tests need none of those,
// so they run on both machines in `colcon test` and they fail in seconds rather
// than minutes — and, unlike the gate, they can construct the pathological
// clock relationships that would otherwise only turn up on somebody else's
// hardware.

#include <cstdint>

#include "gtest/gtest.h"
#include "pimesh_camera/stamp.hpp"

using pimesh_camera::Stamp;
using pimesh_camera::StampSource;
using pimesh_camera::stamp_from_capture;

namespace
{
constexpr std::int64_t kMs = 1000000LL;
constexpr std::int64_t kSecond = 1000000000LL;
}  // namespace

// The ordinary case: a frame dequeued 12 ms after the kernel captured it is
// stamped 12 ms before now.
TEST(Stamp, SubtractsTheFrameAgeFromNow)
{
  const Stamp stamp = stamp_from_capture(
    /* ros_now */ 5000 * kMs, /* monotonic_now */ 900 * kMs,
    /* frame_monotonic */ 888 * kMs, /* monotonic_valid */ true);

  EXPECT_EQ(stamp.nanoseconds, 4988 * kMs);
  EXPECT_EQ(stamp.source, StampSource::kCaptureTime);
}

// ============================================================
// This is the regression test for the bug the node exists to avoid.
// ============================================================
//
// usb_cam 0.8.1 converts the monotonic capture clock to the ROS clock through
// an offset it computes once per process, so every stamp in a session is
// displaced by the same random sub-second amount and the amount is redrawn at
// every launch — measured at 0.223, 0.362 and 0.979 s on three launches.
//
// The property that makes that impossible here is that only the *difference*
// between the two monotonic readings is ever used. So: take one frame, and
// evaluate it under four completely unrelated relationships between the
// monotonic epoch and the ROS epoch — a machine up for 15 minutes, one up for
// 200 days, and two arbitrary ones in between. The frame's age is 8 ms in every
// case. Every answer must be identical.
//
// An implementation that carried an epoch offset would return four different
// stamps here, which is exactly the observable symptom on the real node: two
// launches disagreeing. If this test ever fails, gates/capture.sh's
// "launch delta" assertion is about to fail too, and this one says why.
TEST(Stamp, DoesNotDependOnTheOffsetBetweenTheTwoClockEpochs)
{
  constexpr std::int64_t kAge = 8 * kMs;
  constexpr std::int64_t kRosNow = 1788934501LL * kSecond;

  const std::int64_t uptimes[] = {
    900 * kSecond,                  // up 15 minutes
    17280000LL * kSecond,           // up 200 days
    1LL,                            // pathological: monotonic barely started
    1788934501LL * kSecond,         // pathological: the two epochs coincide
  };

  for (std::int64_t monotonic_now : uptimes) {
    const Stamp stamp = stamp_from_capture(kRosNow, monotonic_now, monotonic_now - kAge, true);
    EXPECT_EQ(stamp.nanoseconds, kRosNow - kAge)
      << "the stamp moved when the monotonic epoch moved (uptime " << monotonic_now
      << " ns) — that is a per-process epoch, and it is the usb_cam bug";
    EXPECT_EQ(stamp.source, StampSource::kCaptureTime);
  }
}

// A driver that reports V4L2_BUF_FLAG_TIMESTAMP_COPY, or no clock at all, has
// not given us a capture time. Stamping at dequeue is a weaker claim and the
// caller has to be able to tell, so it comes back labelled rather than silently
// substituted.
TEST(Stamp, FallsBackToDequeueWhenTheClockIsNotMonotonic)
{
  const Stamp stamp = stamp_from_capture(5000 * kMs, 900 * kMs, 888 * kMs, false);

  EXPECT_EQ(stamp.nanoseconds, 5000 * kMs);
  EXPECT_EQ(stamp.source, StampSource::kNoMonotonicClock);
  // And specifically: the buffer's timestamp must not have been used at all.
  EXPECT_NE(stamp.nanoseconds, 4988 * kMs);
}

// A buffer stamped in the future is not a slightly-late frame, it is a clock we
// have misidentified. Publishing `now + 12 ms` would be confidently wrong.
TEST(Stamp, RefusesANegativeAge)
{
  const Stamp stamp = stamp_from_capture(5000 * kMs, 888 * kMs, 900 * kMs, true);

  EXPECT_EQ(stamp.nanoseconds, 5000 * kMs);
  EXPECT_EQ(stamp.source, StampSource::kImplausibleAge);
}

// The case this guard is really for: the flags say MONOTONIC and the field is
// on some other epoch entirely, so the implied age is enormous. Without the
// bound, a frame would be stamped days in the past and every consumer that
// reasoned about it would be wrong in a way no log mentions.
TEST(Stamp, RefusesAnAgeThatCannotBeReal)
{
  const Stamp stamp = stamp_from_capture(5000 * kMs, 900 * kSecond, 0, true);

  EXPECT_EQ(stamp.nanoseconds, 5000 * kMs);
  EXPECT_EQ(stamp.source, StampSource::kImplausibleAge);
}

// The boundary, both sides of it. A 4-buffer pool at 47 Hz cannot produce more
// than ~85 ms of age, so the default 500 ms is generous — but a threshold that
// is off by one at the edge is a threshold nobody can reason about.
TEST(Stamp, AcceptsExactlyTheMaximumPlausibleAge)
{
  constexpr std::int64_t kLimit = 500 * kMs;
  constexpr std::int64_t kRosNow = 10 * kSecond;

  const Stamp at_limit = stamp_from_capture(kRosNow, kLimit, 0, true, kLimit);
  EXPECT_EQ(at_limit.source, StampSource::kCaptureTime);
  EXPECT_EQ(at_limit.nanoseconds, kRosNow - kLimit);

  const Stamp past_limit = stamp_from_capture(kRosNow, kLimit + 1, 0, true, kLimit);
  EXPECT_EQ(past_limit.source, StampSource::kImplausibleAge);
}

// Zero age is the frame that arrived the instant it was captured. It is not a
// special case and must not be treated as one — an implementation that guarded
// with `age <= 0` would reject a perfectly good frame roughly never, which is
// the worst possible frequency for a bug.
TEST(Stamp, AcceptsAZeroAge)
{
  const Stamp stamp = stamp_from_capture(5000 * kMs, 900 * kMs, 900 * kMs, true);

  EXPECT_EQ(stamp.nanoseconds, 5000 * kMs);
  EXPECT_EQ(stamp.source, StampSource::kCaptureTime);
}
