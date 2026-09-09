#ifndef PIMESH_CAMERA__V4L2_CAPTURE_HPP_
#define PIMESH_CAMERA__V4L2_CAPTURE_HPP_

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace pimesh_camera
{

/// Thrown for anything that means "this device cannot give us frames".
///
/// A separate type because the node treats it differently from every other
/// error: it is not something to log and retry around. `usb_cam` 0.8.1 logs one
/// ERROR on a missing device and then idles forever, which is the worst
/// possible shape — a process that is up, subscribed to, and publishing
/// nothing, looking healthy to everything except a person watching an empty
/// RViz panel. Everything here that cannot produce frames throws, and the node
/// turns that into a non-zero exit.
class CaptureError : public std::runtime_error
{
public:
  using std::runtime_error::runtime_error;
};

/// One dequeued frame, valid until the next call to `grab()`.
///
/// A view, not an owner: `data` points straight into the mmap'd buffer the
/// kernel filled, and `grab()` gives that buffer back to the driver. Copy out
/// of it before then. That is the whole reason capture is cheap here — a 720p
/// MJPEG frame is ~120 kB and it is copied exactly once, into the message.
struct Frame
{
  const std::uint8_t * data {nullptr};
  std::size_t size {0};

  /// The kernel's own capture time, in nanoseconds on CLOCK_MONOTONIC.
  ///
  /// This is the number the whole node exists for. It is stamped by the driver
  /// when the frame was *dequeued from the hardware*, not when userspace got
  /// round to looking at it, and not when something published it.
  std::int64_t monotonic_ns {0};

  /// False when the driver did not give us a monotonic timestamp — some devices
  /// report V4L2_BUF_FLAG_TIMESTAMP_COPY or nothing at all. The caller has to
  /// know, because the fallback (stamp at dequeue) is a different and weaker
  /// claim, and quietly substituting it is how a timestamp bug hides.
  bool monotonic_valid {false};

  /// The driver's own frame counter. Gaps in it are frames the kernel dropped
  /// before we ever saw them, which is a different fault from frames lost on
  /// the wire and is worth being able to tell apart.
  std::uint32_t sequence {0};
};

/// An MJPEG V4L2 capture device: open, configure, mmap, stream.
///
/// Deliberately knows nothing about ROS. The V4L2 ioctl sequence is fiddly and
/// order-dependent enough to be worth reading on its own, and keeping it here
/// means the node above is about timestamps and QoS rather than about `struct
/// v4l2_buffer`.
class V4l2Capture
{
public:
  struct Config
  {
    std::string device {"/dev/video0"};
    std::uint32_t width {1280};
    std::uint32_t height {720};
    std::uint32_t fps {60};
    /// Four is the smallest pool that lets the driver fill one while userspace
    /// holds another with slack for a late wakeup. More would only add latency:
    /// a deeper queue on a live stream is not a buffer, it is stale video.
    std::uint32_t buffer_count {4};
  };

  /// Opens, configures and starts streaming. Throws CaptureError on anything
  /// that fails, which includes the device being missing, not being a capture
  /// device, not doing MJPEG, or already being streamed by somebody else.
  explicit V4l2Capture(const Config & config);
  ~V4l2Capture();

  V4l2Capture(const V4l2Capture &) = delete;
  V4l2Capture & operator=(const V4l2Capture &) = delete;

  /// Wait for and dequeue one frame. Blocks up to `timeout_ms`.
  ///
  /// Returns false only on a timeout with no frame — a survivable event, since
  /// a camera that has just had its exposure changed can go quiet for a beat.
  /// Everything that means the device is gone throws instead.
  bool grab(Frame & out, int timeout_ms);

  /// What the driver actually agreed to, which is not always what was asked
  /// for: VIDIOC_S_FMT negotiates, and it reports the nearest thing it can do
  /// rather than failing. A node that logs the requested size is lying about
  /// half its runs.
  std::uint32_t width() const {return width_;}
  std::uint32_t height() const {return height_;}
  std::uint32_t fps() const {return fps_;}
  const std::string & device() const {return device_;}

private:
  void open_device();
  void check_capabilities();
  void negotiate_format(const Config & config);
  void request_buffers(std::uint32_t count);
  void start_streaming();
  void stop_streaming();

  std::string device_;
  int fd_ {-1};
  bool streaming_ {false};

  struct MappedBuffer
  {
    void * start {nullptr};
    std::size_t length {0};
  };
  std::vector<MappedBuffer> buffers_;

  std::uint32_t width_ {0};
  std::uint32_t height_ {0};
  std::uint32_t fps_ {0};

  /// Index of the buffer handed out by the last `grab()`, still owned by
  /// userspace and requeued at the start of the next call. -1 when we hold
  /// none. Requeuing at the *start* of the next grab rather than at the end of
  /// this one is what makes Frame's pointer valid after grab() returns.
  int held_ {-1};
};

}  // namespace pimesh_camera

#endif  // PIMESH_CAMERA__V4L2_CAPTURE_HPP_
