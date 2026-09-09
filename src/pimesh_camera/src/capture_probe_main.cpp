// The instrument tools/gates/capture.sh measures with.
//
// It exists because the claims in P1 are numbers about a stream — a rate, and a
// stamp-versus-receipt offset — and the obvious way to get them, `ros2 topic
// hz`, cannot be trusted for either. It is Python subscribing to ~120 kB
// messages at 47 Hz, so its own scheduling shows up in the number it reports;
// and it has nothing to say about header stamps at all. A measurement whose
// error is the size of the thing being measured is not evidence.
//
// So: a C++ subscriber that does nothing but record two clocks and a hash, and
// prints `key=value` lines a shell can read. No ROS logging on the hot path, no
// allocation beyond the vector it fills.

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace
{

/// FNV-1a over the JPEG bytes, to count *distinct* frames.
///
/// A camera that is dropping to a lower rate internally, or a driver replaying
/// a buffer, produces messages at full rate with identical payloads — a rate
/// that is entirely real at the DDS layer and entirely fake as a statement
/// about the sensor. The predecessor's 42-60 fps figure was quoted with "0
/// duplicate payloads in 634 messages" for exactly this reason, and a frame
/// rate here is quoted the same way or not at all.
std::uint64_t fnv1a(const std::vector<std::uint8_t> & bytes)
{
  std::uint64_t hash = 1469598103934665603ULL;
  for (std::uint8_t byte : bytes) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) {return 0.0;}
  const std::size_t index =
    std::min(values.size() - 1, static_cast<std::size_t>(fraction * values.size()));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}

}  // namespace

class CaptureProbe : public rclcpp::Node
{
public:
  CaptureProbe()
  : Node("capture_probe")
  {
    topic_ = declare_parameter("topic", std::string("/image_raw/compressed"));
    duration_s_ = declare_parameter("duration_s", 30.0);

    // Must match the publisher: RELIABLE + KEEP_LAST(1). A mismatched
    // reliability makes the subscription silently never connect, which reads
    // downstream as a camera producing nothing.
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();

    sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      topic_, qos,
      [this](sensor_msgs::msg::CompressedImage::UniquePtr msg) {this->on_frame(std::move(msg));});

    // The window opens at the *first frame*, not at startup. Discovery across
    // the LAN takes a second or two, and counting that as dead time would put a
    // 47 Hz stream at 44 Hz and blame the camera.
    fprintf(stderr, "capture_probe: waiting for %s (%.1f s window)\n", topic_.c_str(), duration_s_);
  }

  /// Prints the measurement as `probe <key>=<value>` lines. Nothing else this
  /// program writes goes to stdout, so a gate can read it with awk and never
  /// see a log line.
  int report() const
  {
    if (offsets_ms_.empty()) {
      fprintf(stderr, "capture_probe: no frames on %s\n", topic_.c_str());
      printf("probe frames=0\n");
      return 1;
    }

    const double span_s = std::chrono::duration<double>(last_ - first_).count();
    // n-1 intervals between n frames. At 1400 frames the difference from n is
    // 0.07%, which is below the noise — but a rate is a count of *intervals*
    // and writing it the other way is the kind of small dishonesty that
    // compounds when someone later measures 10 frames.
    const double rate = (span_s > 0.0) ? (static_cast<double>(offsets_ms_.size()) - 1.0) / span_s : 0.0;
    const double unique_rate = (span_s > 0.0) ?
      (static_cast<double>(payloads_.size()) - 1.0) / span_s : 0.0;

    double sum = 0.0;
    for (double value : offsets_ms_) {sum += value;}

    printf("probe topic=%s\n", topic_.c_str());
    printf("probe frames=%zu\n", offsets_ms_.size());
    printf("probe unique_frames=%zu\n", payloads_.size());
    printf("probe span_s=%.3f\n", span_s);
    printf("probe rate_hz=%.2f\n", rate);
    printf("probe unique_rate_hz=%.2f\n", unique_rate);
    // The median, not the mean, is what the gate compares between two launches.
    // Wi-Fi gives this distribution a long right tail — a retransmitted frame
    // arrives tens of milliseconds late — and a mean over 1400 frames still
    // moves several ms run to run because of it. The median moves by
    // microseconds, so a difference between two launches is the node's
    // behaviour rather than the network's mood.
    printf("probe offset_median_ms=%.3f\n", percentile(offsets_ms_, 0.5));
    printf("probe offset_mean_ms=%.3f\n", sum / static_cast<double>(offsets_ms_.size()));
    printf("probe offset_p95_ms=%.3f\n", percentile(offsets_ms_, 0.95));
    printf("probe bytes_mean=%.0f\n", static_cast<double>(bytes_) / static_cast<double>(offsets_ms_.size()));
    return 0;
  }

  bool done() const {return done_;}

private:
  void on_frame(sensor_msgs::msg::CompressedImage::UniquePtr msg)
  {
    const auto arrival = std::chrono::steady_clock::now();
    const rclcpp::Time receipt = now();

    if (offsets_ms_.empty()) {
      first_ = arrival;
      offsets_ms_.reserve(4096);
    }
    last_ = arrival;

    if (std::chrono::duration<double>(arrival - first_).count() > duration_s_) {
      done_ = true;
      return;
    }

    // Receipt minus stamp, in milliseconds. Positive means the frame is stamped
    // in the past, which is what a capture-time stamp on a real network should
    // give. It contains three things and it is worth naming them, because the
    // gate's threshold has to accommodate all three: the true capture-to-arrival
    // latency, the offset between the two machines' NTP-disciplined system
    // clocks, and this probe's own wakeup delay. Only the first is the camera's.
    const rclcpp::Time stamp(msg->header.stamp, receipt.get_clock_type());
    offsets_ms_.push_back(static_cast<double>((receipt - stamp).nanoseconds()) / 1e6);

    bytes_ += msg->data.size();
    payloads_.insert(fnv1a(msg->data));
  }

  std::string topic_;
  double duration_s_ {30.0};
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;

  std::vector<double> offsets_ms_;
  std::unordered_set<std::uint64_t> payloads_;
  std::uint64_t bytes_ {0};
  std::chrono::steady_clock::time_point first_ {};
  std::chrono::steady_clock::time_point last_ {};
  bool done_ {false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto probe = std::make_shared<CaptureProbe>();

  // Spin until the window closes, with a hard ceiling so a probe pointed at a
  // dead topic ends by itself rather than hanging a gate. The ceiling is
  // generous: it is not the measurement, it is the backstop.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(probe);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
  while (rclcpp::ok() && !probe->done() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(50));
  }

  const int status = probe->report();
  rclcpp::shutdown();
  return status;
}
