// The instrument tools/gates/scale.sh measures this project's *unit* with.
//
// **What it is for.** `depth_scale` has been 10.0 since P4 because somebody typed
// it, and monocular depth is scale-ambiguous — the model says "twice as far",
// never "three metres". So every distance this pipeline has ever reported is
// plausibly shaped and in an unknown unit. P12 fixes that with a tape measure and
// a flat surface at a known distance, and this is what reads the map back.
//
// **A patch, not the pixel at the principal point**, which is the false green
// #10's P12 names by hand. `test_tsdf_volume` already pins that a depth map
// carries *z* and not distance along the ray, so the principal point is the one
// place where those agree exactly — the best case in the frame. A single reading
// there means one hot pixel sets this project's unit for good. The statistic and
// its refusals are in `pimesh_depth/depth_patch.hpp`, with `test_depth_patch`
// behind them, because a helper a gate's number comes out of has to be one a test
// can call.
//
// **In the container, like depth_probe.** `/depth` is 32FC1 — 3.7 MB a frame at
// 1280x720 — so an out-of-process subscriber would serialise ~63 MB/s and be a
// large part of the load on the thing it is reading. In the container it is a
// pointer.
//
// **Two spreads, and they are different failures.** The spatial IQR is how flat
// the patch reads within one frame: large means the camera is oblique to the
// surface, or the patch is over an edge. The temporal spread is how much the
// per-frame median moves across the clip: that is Depth Anything's own scale
// breathing, which `fusion_node`'s aligner exists for and which measured a few
// percent a frame. A unit derived from a clip with a large temporal spread is a
// unit with that spread in it, so the gate prints both and neither is hidden
// inside the median.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "pimesh_core/image_buffer.hpp"
#include "pimesh_core/stats.hpp"
#include "pimesh_depth/depth_patch.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace pimesh_depth
{

class ScaleProbe : public rclcpp::Node
{
public:
  explicit ScaleProbe(const rclcpp::NodeOptions & options)
  : Node("scale_probe", options)
  {
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = "Depth map to read. depth_node's own output topic.";
    const std::string topic = declare_parameter("depth_topic", std::string("/depth"), desc);

    desc.description =
      "Side of the centred patch as a fraction of each dimension. 0.25 of "
      "1280x720 is 320x180 — wide enough that one hot pixel cannot set the "
      "median, narrow enough to stay on a surface somebody pointed the camera at.";
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = 0.01;
    range.to_value = 1.0;
    desc.floating_point_range.push_back(range);
    patch_fraction_ = declare_parameter("patch_fraction", 0.25, desc);
    desc.floating_point_range.clear();

    desc.description =
      "The far clip the map was written with. **Must match depth_node's "
      "max_range_m**: a reading at or past it is the model's 'no idea' written as "
      "a real-looking distance, and counting those as distances is what makes an "
      "implied scale come out too small. test_transforms asserts the pair.";
    max_range_m_ = declare_parameter("max_range_m", 6.0, desc);

    desc.description =
      "Seconds to measure before printing the summary. The gate sets it from the "
      "clip's own metadata, so the window and the clip are the same seconds.";
    duration_s_ = declare_parameter("duration_s", 30.0, desc);

    desc.description =
      "Frames to ignore at the start. The first inference after a CUDA session "
      "warms is not representative and the clip's first moments are usually the "
      "camera still settling.";
    warmup_frames_ = static_cast<std::size_t>(declare_parameter("warmup_frames", 5, desc));

    // KeepLast(1), like everything that carries an image here: the freshest frame
    // is the only one anybody wants, and a backlog of 3.7 MB messages is the one
    // thing this probe must not create in the container it is measuring.
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {this->on_depth(std::move(msg));});

    started_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] {this->tick();});

    RCLCPP_INFO(
      get_logger(),
      "scale_probe watching %s for %.1fs, centred patch %.0f%% of frame, clip %.2f m",
      topic.c_str(), duration_s_, patch_fraction_ * 100.0, max_range_m_);
  }

private:
  void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    ++seen_;
    if (seen_ <= warmup_frames_) {return;}

    // A cv::Mat header over the message, not a copy — the same helper
    // fusion_node integrates through, so what is measured here is what is
    // integrated there.
    const cv::Mat depth = pimesh_core::depth_mat_over(*msg);
    if (depth.empty()) {
      ++malformed_;
      return;
    }

    const PatchStats s = centred_patch_stats(depth, patch_fraction_, max_range_m_);
    if (s.usable == 0) {
      // Counted, never folded in as a zero. A frame whose whole patch is clip or
      // NaN says "there was no surface in the middle of this frame", which is a
      // fact about the recording and belongs in the output rather than in the
      // average.
      ++empty_;
      return;
    }
    medians_.push_back(s.median);
    spreads_.push_back(s.iqr());
    clipped_.push_back(s.clipped_fraction());
    ++measured_;
  }

  void tick()
  {
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count();
    if (elapsed < duration_s_ || printed_) {return;}
    printed_ = true;

    if (medians_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "scale_probe result frames=0 — no frame had a usable centred patch. "
        "%lu frames arrived, %lu had nothing but clip or NaN in the middle, %lu "
        "were malformed. Pointing the camera at something inside %.2f m is the "
        "usual fix.",
        static_cast<unsigned long>(seen_), static_cast<unsigned long>(empty_),
        static_cast<unsigned long>(malformed_), max_range_m_);
      return;
    }

    // The median *of the per-frame medians* — one number per frame first, so a
    // single bad frame is one vote rather than a few hundred thousand samples.
    // `spread_time` is the IQR of those, which is the network's scale breathing
    // over the clip; `spread_space` is the typical within-frame IQR.
    const double median = pimesh_core::percentile(medians_, 0.5);
    const double lo = pimesh_core::percentile(medians_, 0.25);
    const double hi = pimesh_core::percentile(medians_, 0.75);

    RCLCPP_INFO(
      get_logger(),
      "scale_probe result frames=%lu seen=%lu empty=%lu malformed=%lu "
      "median_m=%.4f q1_m=%.4f q3_m=%.4f spread_time_m=%.4f spread_space_m=%.4f "
      "clipped_frac=%.4f clipped_max=%.4f",
      static_cast<unsigned long>(measured_), static_cast<unsigned long>(seen_),
      static_cast<unsigned long>(empty_), static_cast<unsigned long>(malformed_),
      median, lo, hi, hi - lo,
      pimesh_core::percentile(spreads_, 0.5),
      pimesh_core::percentile(clipped_, 0.5),
      clipped_.empty() ? 0.0 : *std::max_element(clipped_.begin(), clipped_.end()));
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point started_;

  double patch_fraction_ {0.25};
  double max_range_m_ {6.0};
  double duration_s_ {30.0};
  std::size_t warmup_frames_ {5};
  bool printed_ {false};

  std::uint64_t seen_ {0};
  std::uint64_t measured_ {0};
  std::uint64_t empty_ {0};
  std::uint64_t malformed_ {0};
  std::vector<double> medians_;
  std::vector<double> spreads_;
  std::vector<double> clipped_;
};

}  // namespace pimesh_depth

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_depth::ScaleProbe)
