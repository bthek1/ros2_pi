// Unit tests for the capture layer's testable half.
//
// A camera cannot be assumed present — these run on the dev box, which has
// none — so what is tested here is everything that does NOT need a device:
// the timestamp arithmetic (the most consequential logic in this package) and
// the failure paths, which are the ones that matter most and get exercised
// least.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "pimesh_camera/v4l2_capture.hpp"

using pimesh_camera::TimestampSource;
using pimesh_camera::to_string;
using pimesh_camera::to_system_clock_ns;
using pimesh_camera::V4l2Capture;

namespace
{
constexpr int64_t kSecond = 1000000000LL;
}

// ---------------------------------------------------------------------------
// Timestamp conversion
// ---------------------------------------------------------------------------

TEST(ToSystemClock, AddsTheOffsetBetweenTheTwoClocks)
{
  // The two clocks read 100 s apart; a buffer captured at monotonic 50 s is
  // therefore at system time 150 s.
  EXPECT_EQ(to_system_clock_ns(50 * kSecond, 60 * kSecond, 160 * kSecond), 150 * kSecond);
}

TEST(ToSystemClock, PreservesHowLongAgoTheFrameWasCaptured)
{
  // A frame captured 16 ms before the clocks were sampled must land 16 ms
  // before "now" on the system clock — this is the property every downstream
  // latency measurement rests on.
  const int64_t mono_now = 1234 * kSecond;
  const int64_t real_now = 1788000000LL * kSecond;
  const int64_t age_ns = 16 * 1000000LL;
  EXPECT_EQ(to_system_clock_ns(mono_now - age_ns, mono_now, real_now), real_now - age_ns);
}

TEST(ToSystemClock, IsIndependentOfWhenTheOffsetWasSampled)
{
  // Sample the clock pair at two different instants, with both clocks having
  // advanced by the same amount (they are both real clocks; they tick
  // together). The same buffer must convert to the same system time.
  //
  // This is the anti-usb_cam property stated as a test: the conversion carries
  // no epoch of its own, so nothing about *when* it ran can leak into the
  // answer.
  const int64_t buffer = 500 * kSecond;
  const int64_t mono_a = 600 * kSecond;
  const int64_t real_a = 1000 * kSecond;
  const int64_t advance = 37 * kSecond;
  EXPECT_EQ(
    to_system_clock_ns(buffer, mono_a, real_a),
    to_system_clock_ns(buffer, mono_a + advance, real_a + advance));
}

TEST(ToSystemClock, DoesNotReproduceTheUsbCamEpochBug)
{
  // usb_cam 0.8.1 builds its offset once per process as
  //     tv_sec * 1000000 + tv_usec / 1000.0
  // which mixes microseconds and milliseconds, leaving the offset short by
  // roughly the microsecond field of the wall clock at node start: uniform in
  // 0-1 s, a different draw every launch (measured 0.223 / 0.362 / 0.979 s).
  //
  // Reproduce that arithmetic, show it is wrong by a sub-second amount, and
  // show ours is exact for the same inputs.
  const int64_t real_now = 1788000000LL * kSecond + 723456000LL;  // .723456 s
  const int64_t mono_now = 4242 * kSecond;
  const int64_t buffer = mono_now - 5 * 1000000LL;  // captured 5 ms ago

  const int64_t truth = to_system_clock_ns(buffer, mono_now, real_now);
  EXPECT_EQ(truth, real_now - 5 * 1000000LL);

  // The buggy epoch, in the same units.
  const int64_t sec = real_now / kSecond;
  const int64_t usec = (real_now % kSecond) / 1000;
  const int64_t buggy_epoch_us =
    sec * 1000000LL + static_cast<int64_t>(static_cast<double>(usec) / 1000.0);
  const int64_t buggy = buffer + (buggy_epoch_us * 1000LL - mono_now);

  const int64_t error_ns = truth - buggy;
  EXPECT_GT(error_ns, 700 * 1000000LL);   // ~0.72 s late, this draw
  EXPECT_LT(error_ns, kSecond);           // always under a second, which is why it hid
}

TEST(ToSystemClock, HandlesABufferOlderThanTheOffsetSample)
{
  // Buffers are always in the past relative to the sample; make sure nothing
  // wraps or saturates for a frame that has been sitting in the queue.
  const int64_t mono_now = 10 * kSecond;
  const int64_t real_now = 2000 * kSecond;
  EXPECT_EQ(to_system_clock_ns(1 * kSecond, mono_now, real_now), 1991 * kSecond);
}

// ---------------------------------------------------------------------------
// Timestamp source reporting
// ---------------------------------------------------------------------------

TEST(TimestampSource, ReadsTheDriverFlags)
{
  // The values are the kernel's, restated here so a change to <videodev2.h>
  // shows up as a test failure rather than as silently wrong provenance.
  EXPECT_EQ(pimesh_camera::timestamp_source_from_flags(0x00002000), TimestampSource::Monotonic);
  EXPECT_EQ(pimesh_camera::timestamp_source_from_flags(0x00004000), TimestampSource::Copy);
  EXPECT_EQ(pimesh_camera::timestamp_source_from_flags(0x00000000), TimestampSource::Unknown);
}

TEST(TimestampSource, SaysWhichClockInWordsForTheLog)
{
  // The node prints this when the source is not what we expect, so it has to
  // be legible rather than an enum number.
  EXPECT_NE(std::string(to_string(TimestampSource::Monotonic)).find("CLOCK_MONOTONIC"),
            std::string::npos);
  EXPECT_STRNE(to_string(TimestampSource::Copy), to_string(TimestampSource::Monotonic));
  EXPECT_STRNE(to_string(TimestampSource::Unknown), to_string(TimestampSource::Monotonic));
}

// ---------------------------------------------------------------------------
// Failure paths — the node must fail loudly, never idle
// ---------------------------------------------------------------------------

TEST(Open, ThrowsNamingTheDeviceThatIsNotThere)
{
  try {
    V4l2Capture capture("/dev/video-does-not-exist", 1280, 720, 30, 4);
    FAIL() << "opening a missing device must throw";
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    // The message has to name the path and the errno: a camera failure at 3 am
    // should not require reading the source.
    EXPECT_NE(what.find("/dev/video-does-not-exist"), std::string::npos) << what;
    EXPECT_NE(what.find("errno"), std::string::npos) << what;
  }
}

TEST(Open, RejectsAPathThatIsNotACharacterDevice)
{
  // A regular file passes `stat` but is not a device. Catching this here turns
  // a confusing ioctl error into a sentence.
  const std::string path = std::string(::testing::TempDir()) + "/pimesh_not_a_device";
  { std::ofstream f(path); f << "not a camera"; }

  try {
    V4l2Capture capture(path, 1280, 720, 30, 4);
    std::remove(path.c_str());
    FAIL() << "a regular file must not be accepted as a capture device";
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    std::remove(path.c_str());
    EXPECT_NE(what.find("character device"), std::string::npos) << what;
  }
}

TEST(Open, RejectsACharacterDeviceThatIsNotV4L2)
{
  // /dev/null is a character device, so it gets past the stat check and fails
  // at VIDIOC_QUERYCAP instead. This is the path a wrong /dev/videoN takes.
  try {
    V4l2Capture capture("/dev/null", 1280, 720, 30, 4);
    FAIL() << "/dev/null is not a V4L2 device";
  } catch (const std::runtime_error & e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("V4L2"), std::string::npos) << what;
  }
}
