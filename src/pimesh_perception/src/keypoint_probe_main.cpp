// The instrument tools/gates/keypoints.sh measures with.
//
// It reports three things off the published messages themselves: the rate they
// arrive at, how many features each carries, and what fraction of those features
// carry a track id rather than -1. That last one is the number P3 is judged on
// against the predecessor, and taking it from the message rather than from the
// node's own log is deliberate: a node reporting its own success is the one
// measurement that cannot catch a node that is wrong about it.
//
// Rate is measured on *this* program's steady clock. Not from header.stamp: the
// stamps in a bag are the Pi's capture times and under `ros2 bag play` they say
// nothing whatsoever about how fast anything is running now.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "pimesh_msgs/msg/keypoints.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

double percentile(std::vector<double> values, double fraction)
{
  if (values.empty()) {return 0.0;}
  const std::size_t index =
    std::min(values.size() - 1, static_cast<std::size_t>(fraction * values.size()));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}

}  // namespace

class KeypointProbe : public rclcpp::Node
{
public:
  KeypointProbe()
  : Node("keypoint_probe")
  {
    topic_ = declare_parameter("topic", std::string("/keypoints"));
    duration_s_ = declare_parameter("duration_s", 30.0);
    // Frames to ignore at the start. The first frame can match nothing at all and
    // the window takes ten to fill, so a matched fraction averaged from frame zero
    // is a measurement of the warm-up. The gate's claim is about the steady state.
    warmup_ = static_cast<std::size_t>(declare_parameter("warmup_frames", 15));

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    sub_ = create_subscription<pimesh_msgs::msg::Keypoints>(
      topic_, qos,
      [this](pimesh_msgs::msg::Keypoints::UniquePtr msg) {this->on_msg(std::move(msg));});

    fprintf(
      stderr, "keypoint_probe: waiting for %s (%.1f s window, %zu warm-up frames)\n",
      topic_.c_str(), duration_s_, warmup_);
  }

  int report() const
  {
    if (matched_.empty()) {
      fprintf(stderr, "keypoint_probe: no messages on %s\n", topic_.c_str());
      printf("probe frames=0\n");
      return 1;
    }

    const double span_s = std::chrono::duration<double>(last_ - first_).count();
    // n-1 intervals between n messages.
    const double rate = (span_s > 0.0 && matched_.size() > 1) ?
      (static_cast<double>(matched_.size()) - 1.0) / span_s : 0.0;

    double matched_sum = 0.0;
    double kp_sum = 0.0;
    for (std::size_t i = 0; i < matched_.size(); ++i) {
      matched_sum += matched_[i];
      kp_sum += counts_[i];
    }
    const double n = static_cast<double>(matched_.size());

    printf("probe topic=%s\n", topic_.c_str());
    printf("probe frames=%zu\n", matched_.size());
    printf("probe skipped_warmup=%zu\n", skipped_);
    printf("probe span_s=%.3f\n", span_s);
    printf("probe rate_hz=%.2f\n", rate);
    // The 5th percentile as well as the mean: a pipeline that keeps up on average
    // and stalls for 200 ms every second has a mean that says nothing. The gate
    // asserts on the sustained figure, which is what "sustained" has to mean.
    printf("probe interval_p95_ms=%.2f\n", percentile(intervals_ms_, 0.95));
    printf("probe keypoints_mean=%.1f\n", kp_sum / n);
    printf("probe matched_fraction=%.4f\n", matched_sum / n);
    printf("probe matched_p05=%.4f\n", percentile(matched_, 0.05));
    printf("probe descriptor_bytes=%u\n", descriptor_bytes_);
    printf("probe malformed=%zu\n", malformed_);
    return 0;
  }

  bool done() const {return done_;}

private:
  void on_msg(pimesh_msgs::msg::Keypoints::UniquePtr msg)
  {
    const auto arrival = std::chrono::steady_clock::now();
    if (seen_++ < warmup_) {++skipped_; return;}

    if (matched_.empty()) {
      first_ = arrival;
      matched_.reserve(4096);
    } else {
      intervals_ms_.push_back(std::chrono::duration<double, std::milli>(arrival - last_).count());
    }
    last_ = arrival;

    if (std::chrono::duration<double>(arrival - first_).count() > duration_s_) {
      done_ = true;
      return;
    }

    // The struct-of-arrays invariant, checked rather than assumed: every per-feature
    // array is the same length and the descriptor blob is exactly
    // descriptor_bytes * features. A message that breaks this is not a smaller
    // problem than a slow one — it is a consumer reading somebody else's bytes.
    const std::size_t n = msg->x.size();
    const bool consistent =
      msg->y.size() == n && msg->size.size() == n && msg->angle.size() == n &&
      msg->response.size() == n && msg->track_id.size() == n &&
      msg->descriptors.size() == n * msg->descriptor_bytes;
    if (!consistent) {++malformed_;}
    descriptor_bytes_ = msg->descriptor_bytes;

    std::size_t tracked = 0;
    for (std::int32_t id : msg->track_id) {if (id >= 0) {++tracked;}}

    counts_.push_back(static_cast<double>(n));
    matched_.push_back(n == 0 ? 0.0 : static_cast<double>(tracked) / static_cast<double>(n));
  }

  std::string topic_;
  double duration_s_ {30.0};
  std::size_t warmup_ {15};
  rclcpp::Subscription<pimesh_msgs::msg::Keypoints>::SharedPtr sub_;

  std::vector<double> matched_;
  std::vector<double> counts_;
  std::vector<double> intervals_ms_;
  std::size_t seen_ {0};
  std::size_t skipped_ {0};
  std::size_t malformed_ {0};
  std::uint32_t descriptor_bytes_ {0};
  std::chrono::steady_clock::time_point first_ {};
  std::chrono::steady_clock::time_point last_ {};
  bool done_ {false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto probe = std::make_shared<KeypointProbe>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(probe);
  // A hard ceiling so a probe pointed at a dead topic ends by itself rather than
  // hanging a gate. Not the measurement — the backstop.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(180);
  while (rclcpp::ok() && !probe->done() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(20));
  }

  const int status = probe->report();
  rclcpp::shutdown();
  return status;
}
