// The failure paths of V4l2Capture, all of which run without a camera.
//
// That is the point of them: the dev box has no capture device at all, so these
// are the only camera tests that can run on both machines — and they cover the
// behaviour P1 is most emphatic about, which is that a device that cannot
// deliver frames produces a *refusal*, not a node that comes up and idles.
// `usb_cam` 0.8.1 logs one ERROR and idles forever, and a process that is up,
// discoverable and subscribed to while publishing nothing is the hardest kind
// of failure to see.
//
// The busy-device path — the one that actually bites, when a leaked process
// holds /dev/video0 — cannot be constructed without hardware and is covered by
// gates/capture.sh, which holds the real camera with v4l2-ctl and asserts the
// node refuses in under two seconds.

#include <unistd.h>

#include <cstdio>
#include <string>

#include "gtest/gtest.h"
#include "pimesh_camera/v4l2_capture.hpp"

using pimesh_camera::CaptureError;
using pimesh_camera::V4l2Capture;

namespace
{
V4l2Capture::Config config_for(const std::string & device)
{
  V4l2Capture::Config config;
  config.device = device;
  return config;
}

/// Assert that constructing throws CaptureError *and* that the message says
/// something a person could act on. The second half is not decoration: the
/// whole difference between this node and usb_cam is that its failures are
/// legible, and a test that only checks the exception type would pass over a
/// message that said "error".
void expect_capture_error(const std::string & device, const std::string & expected_substring)
{
  try {
    V4l2Capture capture{config_for(device)};
    FAIL() << device << " constructed successfully — it should have thrown";
  } catch (const CaptureError & e) {
    const std::string what = e.what();
    EXPECT_NE(what.find(expected_substring), std::string::npos)
      << "message was: " << what << "\n  expected it to mention: " << expected_substring;
    EXPECT_NE(what.find(device), std::string::npos)
      << "message does not name the device it failed on: " << what;
  }
}
}  // namespace

// The replugged-camera case, and the commonest one. It is separated from "the
// device exists and will not open" on purpose — the two have different fixes,
// and a single "cannot open" message sends people to look at permissions when
// the camera is simply not plugged in.
TEST(V4l2Capture, RefusesADeviceThatDoesNotExist)
{
  expect_capture_error("/dev/video-does-not-exist", "cannot stat");
}

// A path that is a file rather than a device node. Worth its own message
// because a typo'd device parameter lands here, and "not a character device"
// says which half of the path is wrong.
TEST(V4l2Capture, RefusesSomethingThatIsNotADeviceNode)
{
  char path[] = "/tmp/pimesh_capture_test_XXXXXX";
  const int fd = ::mkstemp(path);
  ASSERT_NE(fd, -1) << "could not create a temporary file to test against";
  ::close(fd);

  expect_capture_error(path, "not a character device");

  ::unlink(path);
}

// A real character device that is not a V4L2 device at all. This is the check
// that /dev/video1 — the C922's UVC metadata node — has to fail, and the reason
// V4l2Capture reads `device_caps` rather than `capabilities`: the latter is the
// union over every node a driver owns, so a metadata node reports its sibling's
// capture bit and sails through a naive check, failing later at REQBUFS with a
// far worse message.
TEST(V4l2Capture, RefusesACharacterDeviceThatIsNotV4L2)
{
  expect_capture_error("/dev/null", "VIDIOC_QUERYCAP");
}

// Whatever went wrong, nothing is left open. A test process that leaked a file
// descriptor per failed construction would be a small problem; the node doing
// it while retrying would be a real one, and the constructor's catch-and-rethrow
// is easy to lose in a refactor.
TEST(V4l2Capture, LeaksNoDescriptorOnFailure)
{
  // A fresh descriptor number is the observable: if the failed constructions
  // below leaked, this second probe would come back higher than the first.
  const int before = ::dup(0);
  ASSERT_NE(before, -1);
  ::close(before);

  for (int i = 0; i < 8; ++i) {
    EXPECT_THROW(V4l2Capture{config_for("/dev/null")}, CaptureError);
  }

  const int after = ::dup(0);
  ASSERT_NE(after, -1);
  ::close(after);

  EXPECT_EQ(after, before) << "eight failed constructions leaked descriptors";
}
