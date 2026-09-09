// The V4L2 half of capture: ioctls, an mmap'd buffer pool, and a poll loop.
//
// Nothing here knows about ROS. What it knows about is the order the kernel
// insists on — QUERYCAP, S_FMT, S_PARM, REQBUFS, QUERYBUF+mmap, QBUF, STREAMON
// — and that every one of those steps has a failure that means "somebody else
// has this camera" rather than "this camera is broken".

#include "pimesh_camera/v4l2_capture.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace pimesh_camera
{
namespace
{

/// ioctl, retried across EINTR.
///
/// A signal arriving mid-ioctl is not an error, and on a node that is expected
/// to be shut down with Ctrl-C it is a routine event. Without this retry, a
/// SIGINT that lands inside VIDIOC_DQBUF surfaces as a capture failure and the
/// node reports the camera died when what happened is that it was asked to
/// stop.
int xioctl(int fd, unsigned long request, void * arg)
{
  int result;
  do {
    result = ::ioctl(fd, request, arg);
  } while (result == -1 && errno == EINTR);
  return result;
}

std::string errno_text(const std::string & what)
{
  return what + ": " + std::strerror(errno) + " (errno " + std::to_string(errno) + ")";
}

/// The one message a person actually needs when capture will not start.
///
/// EBUSY from a V4L2 device means another process is streaming it, and that is
/// by far the most common way this node fails in practice: a leaked camera
/// process from an earlier session holds /dev/video0 exclusively and every
/// later run dies. Saying so, with the command that finds the culprit, is worth
/// more than the errno.
std::string busy_hint(const std::string & device)
{
  return " — another process is streaming " + device +
         ". Find it with: fuser -v " + device;
}

}  // namespace

V4l2Capture::V4l2Capture(const Config & config)
: device_(config.device)
{
  open_device();
  try {
    check_capabilities();
    negotiate_format(config);
    request_buffers(config.buffer_count);
    start_streaming();
  } catch (...) {
    // Half-configured devices stay half-configured until the fd closes, and a
    // process that throws out of a constructor never runs its destructor. The
    // controls this node sets do not persist, but the mmaps and the fd do.
    for (auto & buffer : buffers_) {
      if (buffer.start) {::munmap(buffer.start, buffer.length);}
    }
    buffers_.clear();
    if (fd_ >= 0) {::close(fd_); fd_ = -1;}
    throw;
  }
}

V4l2Capture::~V4l2Capture()
{
  if (streaming_) {stop_streaming();}
  for (auto & buffer : buffers_) {
    if (buffer.start) {::munmap(buffer.start, buffer.length);}
  }
  if (fd_ >= 0) {::close(fd_);}
}

void V4l2Capture::open_device()
{
  // stat first, so "there is no such device" and "there is a device and it will
  // not open" are different messages. They have different fixes: the first is a
  // replugged camera or the wrong node (/dev/video1 is the C922's UVC metadata
  // node, not a capture device), the second is almost always permissions.
  struct stat st {};
  if (::stat(device_.c_str(), &st) == -1) {
    throw CaptureError(errno_text("cannot stat " + device_));
  }
  if (!S_ISCHR(st.st_mode)) {
    throw CaptureError(device_ + " is not a character device");
  }

  // Blocking, and polled. O_NONBLOCK would turn every quiet moment into a busy
  // loop; poll() gives the same responsiveness for no CPU.
  fd_ = ::open(device_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ == -1) {
    std::string message = errno_text("cannot open " + device_);
    if (errno == EACCES) {
      message += " — is this user in the 'video' group?";
    }
    throw CaptureError(message);
  }
}

void V4l2Capture::check_capabilities()
{
  struct v4l2_capability cap {};
  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) == -1) {
    throw CaptureError(errno_text("VIDIOC_QUERYCAP on " + device_) + " — not a V4L2 device");
  }

  // device_caps, not capabilities: the latter is the union over every node the
  // driver owns, so a metadata node reports its sibling's capture bit and
  // passes this check. That is exactly the /dev/video1 trap, and checking the
  // wrong field here means failing later, at REQBUFS, with a worse message.
  const std::uint32_t caps =
    (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ? cap.device_caps : cap.capabilities;

  if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) {
    throw CaptureError(
            device_ + " is a V4L2 device but not a video capture node (driver: " +
            reinterpret_cast<const char *>(cap.driver) +
            ") — on the C922, capture is /dev/video0 and /dev/video1 is its metadata node");
  }
  if (!(caps & V4L2_CAP_STREAMING)) {
    throw CaptureError(device_ + " does not support streaming I/O (mmap)");
  }
}

void V4l2Capture::negotiate_format(const Config & config)
{
  struct v4l2_format fmt {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = config.width;
  fmt.fmt.pix.height = config.height;
  // MJPEG, and no decode anywhere on this machine. The Pi is a sensor head: the
  // JPEG the camera produced is the JPEG that goes on the wire. Asking for
  // YUYV at 720p instead would be ~1.3 MB a frame across Wi-Fi, which the
  // network budget does not have and never will.
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;

  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) == -1) {
    std::string message = errno_text("VIDIOC_S_FMT on " + device_);
    if (errno == EBUSY) {message += busy_hint(device_);}
    throw CaptureError(message);
  }

  // S_FMT *negotiates*. It fills the struct back in with what the driver will
  // actually do and returns success, so a device that cannot do 1280x720 MJPEG
  // silently hands back 640x480 YUYV and every downstream assumption is wrong
  // with nothing having failed. Check what came back, not what went in.
  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
    throw CaptureError(
            device_ + " will not do MJPEG — the driver chose a different format. "
            "Check `v4l2-ctl -d " + device_ + " --list-formats-ext`");
  }
  width_ = fmt.fmt.pix.width;
  height_ = fmt.fmt.pix.height;
  if (width_ != config.width || height_ != config.height) {
    throw CaptureError(
            device_ + " gave " + std::to_string(width_) + "x" + std::to_string(height_) +
            " instead of " + std::to_string(config.width) + "x" +
            std::to_string(config.height) + " — that resolution is not available in MJPEG");
  }

  // The frame interval is a request, not a setting, and a great many UVC
  // cameras quietly ignore it. Worse, the C922 will *agree* to 60 fps and then
  // deliver 18-21 because exposure_dynamic_framerate=1 trades rate for exposure
  // in indoor light — a control that persists inside the camera across
  // processes and reboots. So this is best-effort: not being able to set it is
  // not fatal, and the number reported back is not a promise either. Never
  // quote a frame rate from here; measure it. tools/camera-reset.sh is what
  // clears the control that actually costs the frames.
  struct v4l2_streamparm parm {};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_G_PARM, &parm) != -1 &&
    (parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME))
  {
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = config.fps;
    if (xioctl(fd_, VIDIOC_S_PARM, &parm) != -1) {
      const auto & tpf = parm.parm.capture.timeperframe;
      fps_ = (tpf.numerator > 0) ? (tpf.denominator / tpf.numerator) : config.fps;
    }
  }
  if (fps_ == 0) {fps_ = config.fps;}
}

void V4l2Capture::request_buffers(std::uint32_t count)
{
  struct v4l2_requestbuffers req {};
  req.count = count;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (xioctl(fd_, VIDIOC_REQBUFS, &req) == -1) {
    std::string message = errno_text("VIDIOC_REQBUFS on " + device_);
    if (errno == EBUSY) {message += busy_hint(device_);}
    throw CaptureError(message);
  }
  if (req.count < 2) {
    throw CaptureError(
            device_ + " granted only " + std::to_string(req.count) +
            " buffer(s); streaming needs at least 2");
  }

  buffers_.resize(req.count);
  for (std::uint32_t i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) {
      throw CaptureError(errno_text("VIDIOC_QUERYBUF " + std::to_string(i)));
    }
    // The kernel's buffer, mapped into this process. No copy happens here and
    // none happens on capture either — the driver DMAs into this page and the
    // only copy in the whole path is the one into the outgoing message.
    void * start = ::mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
    if (start == MAP_FAILED) {
      throw CaptureError(errno_text("mmap of buffer " + std::to_string(i)));
    }
    buffers_[i] = MappedBuffer{start, buf.length};
  }
}

void V4l2Capture::start_streaming()
{
  // Every buffer to the driver before STREAMON, or the first frames go nowhere.
  for (std::uint32_t i = 0; i < buffers_.size(); ++i) {
    struct v4l2_buffer buf {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QBUF, &buf) == -1) {
      throw CaptureError(errno_text("VIDIOC_QBUF " + std::to_string(i)));
    }
  }

  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
    std::string message = errno_text("VIDIOC_STREAMON on " + device_);
    if (errno == EBUSY) {message += busy_hint(device_);}
    throw CaptureError(message);
  }
  streaming_ = true;
}

void V4l2Capture::stop_streaming()
{
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(fd_, VIDIOC_STREAMOFF, &type);   // shutting down; nothing to do on failure
  streaming_ = false;
  held_ = -1;
}

bool V4l2Capture::grab(Frame & out, int timeout_ms)
{
  // Give back the buffer the previous call handed out. Doing it here rather
  // than at the end of that call is what makes the returned Frame's pointer
  // valid for as long as the caller holds it: the driver cannot start writing
  // into the buffer again until we say so.
  if (held_ >= 0) {
    struct v4l2_buffer buf {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = static_cast<std::uint32_t>(held_);
    if (xioctl(fd_, VIDIOC_QBUF, &buf) == -1) {
      throw CaptureError(errno_text("VIDIOC_QBUF (requeue)"));
    }
    held_ = -1;
  }

  struct pollfd pfd {};
  pfd.fd = fd_;
  pfd.events = POLLIN;
  int ready;
  do {
    ready = ::poll(&pfd, 1, timeout_ms);
  } while (ready == -1 && errno == EINTR);

  if (ready == -1) {
    throw CaptureError(errno_text("poll on " + device_));
  }
  if (ready == 0) {
    return false;             // quiet, not dead
  }
  // POLLERR on a V4L2 fd is the unplug. There is no recovering from it in this
  // process — the device node is gone — so it has to reach the caller as a
  // failure rather than as an empty poll that looks like a slow camera.
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    throw CaptureError(device_ + " reported a device error (poll revents) — unplugged?");
  }

  struct v4l2_buffer buf {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_DQBUF, &buf) == -1) {
    // EAGAIN after a successful poll is a race the kernel is allowed to lose;
    // everything else means the stream is over.
    if (errno == EAGAIN) {return false;}
    throw CaptureError(errno_text("VIDIOC_DQBUF on " + device_));
  }
  held_ = static_cast<int>(buf.index);

  out.data = static_cast<const std::uint8_t *>(buffers_[buf.index].start);
  out.size = buf.bytesused;
  out.sequence = buf.sequence;

  // The timestamp, and the flag that says what clock it is on.
  //
  // V4L2 reports its clock in the buffer's flags, and it genuinely varies:
  // TIMESTAMP_MONOTONIC is CLOCK_MONOTONIC and is what UVC gives; TIMESTAMP_COPY
  // means the driver copied whatever userspace put in the buffer, which is not
  // a capture time at all. Reading the field without checking the flag is how a
  // node ends up confidently stamping frames from a clock it has never
  // identified. The caller is told which it got and decides.
  const std::uint32_t clock_kind = buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK;
  out.monotonic_valid = (clock_kind == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC);
  out.monotonic_ns =
    static_cast<std::int64_t>(buf.timestamp.tv_sec) * 1000000000LL +
    static_cast<std::int64_t>(buf.timestamp.tv_usec) * 1000LL;

  return true;
}

}  // namespace pimesh_camera
