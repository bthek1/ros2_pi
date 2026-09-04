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
#include <ctime>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace pimesh_camera
{

namespace
{

/// ioctl retried across signals. Every V4L2 call goes through this: a stray
/// SIGCHLD returning EINTR is not a camera failure.
int xioctl(int fd, unsigned long request, void * arg)
{
  int result = 0;
  do {
    result = ::ioctl(fd, request, arg);
  } while (result == -1 && errno == EINTR);
  return result;
}

[[noreturn]] void fail(const std::string & what, int err)
{
  throw std::runtime_error(what + ": " + std::strerror(err) + " (errno " + std::to_string(err) + ")");
}

std::string fourcc(uint32_t f)
{
  return std::string{
    static_cast<char>(f & 0xff), static_cast<char>((f >> 8) & 0xff),
    static_cast<char>((f >> 16) & 0xff), static_cast<char>((f >> 24) & 0xff)};
}

}  // namespace

const char * to_string(TimestampSource source)
{
  switch (source) {
    case TimestampSource::Monotonic: return "CLOCK_MONOTONIC (capture time)";
    case TimestampSource::Copy: return "copied from upstream";
    default: return "unknown";
  }
}

V4l2Capture::V4l2Capture(
  std::string device, uint32_t width, uint32_t height, uint32_t fps, uint32_t buffer_count)
: device_(std::move(device))
{
  open_device();
  try {
    set_format(width, height);
    set_framerate(fps);
    map_buffers(buffer_count);
  } catch (...) {
    unmap_buffers();
    if (fd_ >= 0) {::close(fd_); fd_ = -1;}
    throw;
  }
}

V4l2Capture::~V4l2Capture()
{
  stop();
  unmap_buffers();
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void V4l2Capture::open_device()
{
  struct stat st {};
  if (::stat(device_.c_str(), &st) == -1) {
    fail("cannot stat " + device_, errno);
  }
  if (!S_ISCHR(st.st_mode)) {
    throw std::runtime_error(device_ + " is not a character device");
  }

  // O_NONBLOCK: dequeue must never block indefinitely — poll() owns the
  // waiting, so a dead device shows up as a timeout we can act on rather than
  // a thread parked forever in an ioctl.
  fd_ = ::open(device_.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    // EBUSY here is the classic leaked-camera-process symptom.
    fail("cannot open " + device_, errno);
  }

  v4l2_capability cap{};
  if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) == -1) {
    fail(device_ + " is not a V4L2 device (VIDIOC_QUERYCAP)", errno);
  }
  const auto caps = (cap.capabilities & V4L2_CAP_DEVICE_CAPS) ?
    cap.device_caps : cap.capabilities;
  if (!(caps & V4L2_CAP_VIDEO_CAPTURE)) {
    // This is what /dev/video1 is on the C922: a metadata node, not capture.
    throw std::runtime_error(
            device_ + " has no video-capture capability — on the C922 this is the UVC "
            "metadata node (index1), not the camera (index0)");
  }
  if (!(caps & V4L2_CAP_STREAMING)) {
    throw std::runtime_error(device_ + " does not support streaming I/O (mmap)");
  }
}

void V4l2Capture::set_format(uint32_t width, uint32_t height)
{
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  // MJPEG, and nothing else. The point of this node is to move the camera's
  // own compressed bytes onto the wire untouched.
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;

  if (xioctl(fd_, VIDIOC_S_FMT, &fmt) == -1) {
    fail("VIDIOC_S_FMT failed", errno);
  }

  // The driver is allowed to hand back something other than what was asked
  // for. Silently publishing a different resolution than the parameters claim
  // is how a pipeline ends up with wrong intrinsics, so refuse.
  if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_MJPEG) {
    throw std::runtime_error(
            "driver refused MJPEG and chose " + fourcc(fmt.fmt.pix.pixelformat) +
            " — this node publishes the camera's JPEG verbatim and cannot convert");
  }
  width_ = fmt.fmt.pix.width;
  height_ = fmt.fmt.pix.height;
  if (width_ != width || height_ != height) {
    throw std::runtime_error(
            "driver chose " + std::to_string(width_) + "x" + std::to_string(height_) +
            " instead of the requested " + std::to_string(width) + "x" + std::to_string(height) +
            " — check `v4l2-ctl --list-formats-ext` for the modes this camera actually has");
  }
}

void V4l2Capture::set_framerate(uint32_t fps)
{
  v4l2_streamparm parm{};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_G_PARM, &parm) == -1) {
    fail("VIDIOC_G_PARM failed", errno);
  }
  if (!(parm.parm.capture.capability & V4L2_CAP_TIMEPERFRAME)) {
    // Not fatal: the camera runs at whatever it runs at, and this node's job
    // is to publish frames as they arrive, not to pace them.
    actual_fps_ = 0.0;
    return;
  }

  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = fps;
  if (xioctl(fd_, VIDIOC_S_PARM, &parm) == -1) {
    fail("VIDIOC_S_PARM failed", errno);
  }
  const auto & tpf = parm.parm.capture.timeperframe;
  actual_fps_ = tpf.numerator > 0 ?
    static_cast<double>(tpf.denominator) / static_cast<double>(tpf.numerator) : 0.0;
}

void V4l2Capture::map_buffers(uint32_t count)
{
  v4l2_requestbuffers req{};
  req.count = count;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_REQBUFS, &req) == -1) {
    // EBUSY here means another process is already streaming from this device.
    // V4L2 lets two processes *open* the node; only one gets buffers.
    fail("VIDIOC_REQBUFS failed (another process may be streaming)", errno);
  }
  if (req.count < 2) {
    throw std::runtime_error(
            "driver granted only " + std::to_string(req.count) +
            " buffers; need at least 2 to keep one queued while the other is in flight");
  }

  buffers_.resize(req.count);
  for (uint32_t i = 0; i < req.count; ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) {
      fail("VIDIOC_QUERYBUF failed", errno);
    }
    void * start = ::mmap(
      nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);
    if (start == MAP_FAILED) {
      fail("mmap of buffer " + std::to_string(i) + " failed", errno);
    }
    buffers_[i].start = start;
    buffers_[i].length = buf.length;
  }
}

void V4l2Capture::unmap_buffers() noexcept
{
  for (auto & b : buffers_) {
    if (b.start != nullptr) {
      ::munmap(b.start, b.length);
      b.start = nullptr;
    }
  }
  buffers_.clear();
}

void V4l2Capture::start()
{
  for (uint32_t i = 0; i < buffers_.size(); ++i) {
    v4l2_buffer buf{};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (xioctl(fd_, VIDIOC_QBUF, &buf) == -1) {
      fail("VIDIOC_QBUF failed while priming buffer " + std::to_string(i), errno);
    }
  }

  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
    fail("VIDIOC_STREAMON failed", errno);
  }
  streaming_ = true;
}

void V4l2Capture::stop() noexcept
{
  if (!streaming_ || fd_ < 0) {
    return;
  }
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  xioctl(fd_, VIDIOC_STREAMOFF, &type);
  streaming_ = false;
}

bool V4l2Capture::wait_frame(Frame & frame, int timeout_ms)
{
  pollfd pfd{fd_, POLLIN, 0};
  int ready = 0;
  do {
    ready = ::poll(&pfd, 1, timeout_ms);
  } while (ready == -1 && errno == EINTR);

  if (ready == -1) {
    fail("poll on " + device_ + " failed", errno);
  }
  if (ready == 0) {
    return false;
  }
  // POLLERR/POLLHUP is what an unplugged USB camera looks like.
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    throw std::runtime_error(device_ + " reported a device error (unplugged?)");
  }

  v4l2_buffer buf{};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd_, VIDIOC_DQBUF, &buf) == -1) {
    if (errno == EAGAIN) {
      return false;  // poll raced us; no frame after all
    }
    fail("VIDIOC_DQBUF failed", errno);
  }

  // ---- the timestamp, which is the reason this node exists -----------------
  //
  // The driver stamps the buffer when the frame was captured, on
  // CLOCK_MONOTONIC. ROS publishes on the system clock, so the two are related
  // by an offset that is sampled HERE, once per frame, a few microseconds
  // after dequeue.
  //
  // usb_cam 0.8.1 computes that offset once per process and gets it wrong by
  // up to a second (a `tv_sec * 1000000 + tv_usec / 1000.0` mix-up), which is
  // why every stamp it emits sits a random sub-second amount in the past — a
  // different amount at every launch. Sampling per frame cannot drift and
  // cannot carry a startup error: there is no epoch to get wrong.
  timespec mono{}, real{};
  ::clock_gettime(CLOCK_MONOTONIC, &mono);
  ::clock_gettime(CLOCK_REALTIME, &real);

  const uint32_t stamp_flags = buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK;
  if (stamp_flags == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
    timestamp_source_ = TimestampSource::Monotonic;
  } else if (stamp_flags == V4L2_BUF_FLAG_TIMESTAMP_COPY) {
    timestamp_source_ = TimestampSource::Copy;
  } else {
    timestamp_source_ = TimestampSource::Unknown;
  }

  const int64_t mono_ns = static_cast<int64_t>(mono.tv_sec) * 1000000000LL + mono.tv_nsec;
  const int64_t real_ns = static_cast<int64_t>(real.tv_sec) * 1000000000LL + real.tv_nsec;
  const int64_t buf_ns =
    static_cast<int64_t>(buf.timestamp.tv_sec) * 1000000000LL +
    static_cast<int64_t>(buf.timestamp.tv_usec) * 1000LL;

  if (timestamp_source_ == TimestampSource::Monotonic) {
    frame.stamp_ns = buf_ns + (real_ns - mono_ns);
  } else {
    // No trustworthy capture time: stamp on arrival and let the node warn.
    frame.stamp_ns = real_ns;
  }

  frame.data = static_cast<const uint8_t *>(buffers_[buf.index].start);
  frame.size = buf.bytesused;
  frame.sequence = buf.sequence;
  frame.index = buf.index;
  return true;
}

void V4l2Capture::requeue(const Frame & frame)
{
  v4l2_buffer buf{};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.index = frame.index;
  if (xioctl(fd_, VIDIOC_QBUF, &buf) == -1) {
    fail("VIDIOC_QBUF failed returning buffer " + std::to_string(frame.index), errno);
  }
}

}  // namespace pimesh_camera
