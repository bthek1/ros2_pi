// V4L2 MJPEG capture, as thin as it can honestly be.
//
// The Pi is a sensor head: this class opens the device, hands out the JPEG
// bytes the camera produced, and does nothing else. No decode, no re-encode,
// no colour conversion — those all belong on the dev box, where there is a GPU
// and no Wi-Fi link in the way.
//
// Why raw ioctls rather than libv4l2: libv4l2's whole value is transparent
// format conversion, and converting is exactly what this must not do. Plain
// ioctls against <linux/videodev2.h> also mean the Pi needs no extra package
// beyond linux-libc-dev, which build-essential already pulls in.

#ifndef PIMESH_CAMERA__V4L2_CAPTURE_HPP_
#define PIMESH_CAMERA__V4L2_CAPTURE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace pimesh_camera
{

/// One dequeued frame. `data` points into a memory-mapped driver buffer and
/// stays valid only until requeue() — copy out of it before then.
struct Frame
{
  const uint8_t * data{nullptr};
  size_t size{0};
  /// Capture time, converted to the system clock. See V4l2Capture::wait_frame.
  int64_t stamp_ns{0};
  uint32_t sequence{0};
  uint32_t index{0};
};

/// How the driver timestamped the buffer. Anything but Monotonic means the
/// stamps are not capture times and the node must say so out loud.
enum class TimestampSource
{
  Monotonic,   ///< CLOCK_MONOTONIC at capture — what we want, and what UVC gives
  Copy,        ///< copied from an upstream source; provenance unknown
  Unknown,     ///< the driver did not say
};

const char * to_string(TimestampSource source);

/// Which clock the driver stamped a buffer against, from its flags.
TimestampSource timestamp_source_from_flags(uint32_t buffer_flags);

/// Convert a buffer's CLOCK_MONOTONIC capture time to the system clock, given
/// a pair of clock readings taken at (very nearly) the same instant.
///
/// This is the whole timestamp story in one line, and it is a free function so
/// it can be tested without a camera.
///
/// The offset is a property of the two clocks *now*, so it is sampled per
/// frame, microseconds after the buffer is dequeued. usb_cam 0.8.1 computes an
/// equivalent offset ONCE per process and gets it wrong besides — it writes
/// `tv_sec * 1000000 + tv_usec / 1000.0`, mixing microseconds and
/// milliseconds, so the result is short by roughly the microsecond field of the
/// wall clock at node start. That is uniform in 0-1 s, redrawn at every launch,
/// and then constant for the life of the process: the reason every stamp it
/// emits sits a random sub-second amount in the past.
///
/// Sampling per frame has no epoch to get wrong and nothing to drift.
constexpr int64_t to_system_clock_ns(
  int64_t buffer_monotonic_ns, int64_t monotonic_now_ns, int64_t realtime_now_ns)
{
  return buffer_monotonic_ns + (realtime_now_ns - monotonic_now_ns);
}

/// RAII: the device is opened in the constructor and released in the
/// destructor. Every failure throws std::runtime_error with the operation and
/// errno spelled out — a camera consumer that idles instead of failing is the
/// bug this project inherited and is not repeating.
class V4l2Capture
{
public:
  V4l2Capture(
    std::string device, uint32_t width, uint32_t height, uint32_t fps,
    uint32_t buffer_count);
  ~V4l2Capture();

  V4l2Capture(const V4l2Capture &) = delete;
  V4l2Capture & operator=(const V4l2Capture &) = delete;

  void start();
  void stop() noexcept;

  /// Block until a frame arrives. Returns false on timeout (no frame yet),
  /// throws if the device goes away. On success the frame must be handed back
  /// with requeue() or the driver runs out of buffers.
  bool wait_frame(Frame & frame, int timeout_ms);
  void requeue(const Frame & frame);

  uint32_t width() const {return width_;}
  uint32_t height() const {return height_;}
  /// What the driver actually granted, which is not always what was asked for.
  double actual_fps() const {return actual_fps_;}
  uint32_t buffer_count() const {return buffers_.size();}
  TimestampSource timestamp_source() const {return timestamp_source_;}
  const std::string & device() const {return device_;}

private:
  struct MappedBuffer
  {
    void * start{nullptr};
    size_t length{0};
  };

  void open_device();
  void set_format(uint32_t width, uint32_t height);
  void set_framerate(uint32_t fps);
  void map_buffers(uint32_t count);
  void unmap_buffers() noexcept;

  std::string device_;
  int fd_{-1};
  uint32_t width_{0};
  uint32_t height_{0};
  double actual_fps_{0.0};
  bool streaming_{false};
  TimestampSource timestamp_source_{TimestampSource::Unknown};
  std::vector<MappedBuffer> buffers_;
};

}  // namespace pimesh_camera

#endif  // PIMESH_CAMERA__V4L2_CAPTURE_HPP_
