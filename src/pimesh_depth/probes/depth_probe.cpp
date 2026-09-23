// The instrument tools/gates/depth.sh measures with, as a component rather than a
// program — for the same reason ipc_probe is one, arrived at from a different
// direction.
//
// This probe has to watch three megabyte-class topics at once: `/image_raw` at
// 59 Hz (2.7 MB), `/depth` (3.7 MB) and `/depth/rgb` (2.7 MB) at whatever rate
// inference sustains. Out of process that is roughly **255 MB/s of
// serialisation**, all of it charged to the container this gate is trying to
// measure — the instrument would be the dominant load and the number it reported
// would be mostly about itself. Loaded into the container, every one of those is
// a pointer.
//
// It answers three questions the node cannot be trusted to answer about itself:
//
//  1. **Rate**, on this component's own steady clock. Not from `header.stamp`:
//     those are the Pi's capture times, and under `ros2 bag play` they say
//     nothing about how fast anything is running now.
//  2. **Is `/depth/rgb` byte-identical to the frame that depth was inferred on?**
//     This is P4's stated assertion and it is the one that catches a whole class
//     of silent wrongness — a depth map paired with a *different* frame produces
//     a coloured cloud that looks entirely plausible and is registered to the
//     wrong instant. Answered by hashing every `/image_raw` frame as it goes past,
//     keyed by stamp, and comparing the hash of each `/depth/rgb` against the
//     entry for its own stamp. A hash rather than a memcmp because holding the
//     source frames themselves would mean pinning 2.7 MB per in-flight stamp.
//  3. **Are the distances usable numbers?** Finite, positive, inside the clip. A
//     NaN or an infinity here survives every downstream comparison in silence and
//     poisons a TSDF several stages later.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pimesh_core/image_buffer.hpp"
#include "pimesh_core/stats.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

using pimesh_core::percentile;
using pimesh_core::fnv1a;

namespace pimesh_depth
{
namespace
{

std::int64_t stamp_key(const builtin_interfaces::msg::Time & t)
{
  return static_cast<std::int64_t>(t.sec) * 1000000000LL + t.nanosec;
}

}  // namespace

class DepthProbe : public rclcpp::Node
{
public:
  explicit DepthProbe(const rclcpp::NodeOptions & options)
  : Node("depth_probe", options)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Seconds to measure before printing the summary.";
    duration_s_ = declare_parameter("duration_s", 60.0, desc);

    desc.description = "Frames to ignore before averaging, so warm-up is not in the number.";
    warmup_ = declare_parameter("warmup_frames", 3, desc);

    desc.description = "The clip depth_node was configured with, for the range assertion.";
    max_range_ = declare_parameter("max_range_m", 6.0);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();

    // **ConstSharedPtr on every one of these, and it is not a style choice.**
    // rclcpp moves a buffer into the *last* ownership-taking subscription and
    // copies it for every other, so a probe that asked to own `/image_raw` would
    // silently add a 2.7 MB copy per frame to the very container it is measuring
    // — and would make tools/gates/ipc.sh start failing. See ipc_probe.cpp for
    // the measurement that settled this.
    source_sub_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter("source_topic", std::string("/image_raw")), qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_source(*msg);});

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter("depth_topic", std::string("/depth")), qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_depth(*msg);});

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      declare_parameter("rgb_topic", std::string("/depth/rgb")), qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_rgb(*msg);});

    started_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(duration_s_), [this] {report();});
  }

private:
  void on_source(const sensor_msgs::msg::Image & msg)
  {
    source_frames_++;
    // A bounded history keyed by stamp. `/image_raw` runs ~4x faster than depth,
    // so a few seconds of frames is more than enough to still hold the entry when
    // the matching /depth/rgb arrives; unbounded, this would be a 2.7 MB-per-frame
    // leak dressed as a cache.
    source_hashes_.emplace_back(stamp_key(msg.header.stamp), fnv1a(msg.data));
    while (source_hashes_.size() > 256) {source_hashes_.pop_front();}
  }

  void on_rgb(const sensor_msgs::msg::Image & msg)
  {
    rgb_frames_++;
    const auto key = stamp_key(msg.header.stamp);
    const auto found = std::find_if(
      source_hashes_.begin(), source_hashes_.end(),
      [key](const auto & entry) {return entry.first == key;});

    if (found == source_hashes_.end()) {
      // The source frame aged out of the window, or the stamp was rewritten.
      // Counted separately from a mismatch: "we could not check" and "we checked
      // and it differed" are different results and only one is a failure.
      rgb_unmatched_++;
      return;
    }
    if (found->second == fnv1a(msg.data)) {
      rgb_identical_++;
    } else {
      rgb_different_++;
    }
    if (msg.header.frame_id != "camera_optical_frame") {rgb_wrong_frame_++;}
  }

  void on_depth(const sensor_msgs::msg::Image & msg)
  {
    const auto now = std::chrono::steady_clock::now();
    depth_frames_++;
    if (depth_frames_ <= static_cast<std::uint64_t>(std::max(0, warmup_))) {
      last_arrival_ = now;
      return;
    }

    if (measured_ > 0) {
      intervals_.push_back(
        std::chrono::duration<double, std::milli>(now - last_arrival_).count());
    }
    last_arrival_ = now;
    measured_++;

    if (msg.encoding != "32FC1") {wrong_encoding_++; return;}
    if (msg.header.frame_id != "camera_optical_frame") {depth_wrong_frame_++;}

    // Does a /depth exist for a stamp we actually saw on the input? This is the
    // "carries the input frame's stamp" half of P4's claim, checked against the
    // source rather than against /depth/rgb — the two could agree with each other
    // and both be wrong.
    const auto key = stamp_key(msg.header.stamp);
    if (std::any_of(
        source_hashes_.begin(), source_hashes_.end(),
        [key](const auto & entry) {return entry.first == key;}))
    {
      stamp_matched_++;
    } else {
      stamp_unmatched_++;
    }

    // Sample the distances rather than scanning 921,600 of them per frame: the
    // probe shares a process with the node it is measuring, and a full scan at
    // 15 Hz is CPU taken from inference. Every 997th pixel is a prime stride, so
    // the sample walks the whole image rather than one column of it.
    const auto * pixels = reinterpret_cast<const float *>(msg.data.data());
    const std::size_t count = msg.data.size() / sizeof(float);
    for (std::size_t i = 0; i < count; i += 997) {
      const float v = pixels[i];
      sampled_++;
      if (!std::isfinite(v)) {
        non_finite_++;
      } else if (v <= 0.0F || v > max_range_ + 1e-3) {
        out_of_range_++;
      } else {
        depth_sum_ += v;
        depth_min_ = std::min(depth_min_, static_cast<double>(v));
        depth_max_ = std::max(depth_max_, static_cast<double>(v));
      }
    }
  }

  void report()
  {
    if (reported_) {return;}
    reported_ = true;

    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
    const double rate = elapsed > 0.0 ? static_cast<double>(measured_) / elapsed : 0.0;

    // stdout, as `probe key=value`, the same shape every instrument here uses so
    // a gate reads it with awk rather than by parsing prose.
    std::printf("probe source_frames=%lu\n", source_frames_);
    std::printf("probe depth_frames=%lu\n", depth_frames_);
    std::printf("probe measured=%lu\n", measured_);
    std::printf("probe rate_hz=%.2f\n", rate);
    std::printf("probe interval_p95_ms=%.2f\n", percentile(intervals_, 0.95));
    std::printf("probe rgb_frames=%lu\n", rgb_frames_);
    std::printf("probe rgb_identical=%lu\n", rgb_identical_);
    std::printf("probe rgb_different=%lu\n", rgb_different_);
    std::printf("probe rgb_unmatched=%lu\n", rgb_unmatched_);
    std::printf("probe stamp_matched=%lu\n", stamp_matched_);
    std::printf("probe stamp_unmatched=%lu\n", stamp_unmatched_);
    std::printf("probe wrong_encoding=%lu\n", wrong_encoding_);
    std::printf("probe wrong_frame=%lu\n", depth_wrong_frame_ + rgb_wrong_frame_);
    std::printf("probe sampled=%lu\n", sampled_);
    std::printf("probe non_finite=%lu\n", non_finite_);
    std::printf("probe out_of_range=%lu\n", out_of_range_);
    const std::uint64_t good = sampled_ - non_finite_ - out_of_range_;
    std::printf("probe depth_mean_m=%.3f\n", good ? depth_sum_ / static_cast<double>(good) : 0.0);
    std::printf("probe depth_min_m=%.3f\n", good ? depth_min_ : 0.0);
    std::printf("probe depth_max_m=%.3f\n", good ? depth_max_ : 0.0);
    std::fflush(stdout);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr source_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double duration_s_ {60.0};
  int warmup_ {3};
  double max_range_ {6.0};
  bool reported_ {false};

  std::deque<std::pair<std::int64_t, std::uint64_t>> source_hashes_;

  std::chrono::steady_clock::time_point started_;
  std::chrono::steady_clock::time_point last_arrival_;
  std::vector<double> intervals_;

  std::uint64_t source_frames_ {0};
  std::uint64_t depth_frames_ {0};
  std::uint64_t measured_ {0};
  std::uint64_t rgb_frames_ {0};
  std::uint64_t rgb_identical_ {0};
  std::uint64_t rgb_different_ {0};
  std::uint64_t rgb_unmatched_ {0};
  std::uint64_t stamp_matched_ {0};
  std::uint64_t stamp_unmatched_ {0};
  std::uint64_t wrong_encoding_ {0};
  std::uint64_t depth_wrong_frame_ {0};
  std::uint64_t rgb_wrong_frame_ {0};
  std::uint64_t sampled_ {0};
  std::uint64_t non_finite_ {0};
  std::uint64_t out_of_range_ {0};
  double depth_sum_ {0.0};
  double depth_min_ {1e9};
  double depth_max_ {-1e9};
};

}  // namespace pimesh_depth

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_depth::DepthProbe)
