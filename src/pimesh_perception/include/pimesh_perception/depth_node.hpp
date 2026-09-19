#ifndef PIMESH_PERCEPTION__DEPTH_NODE_HPP_
#define PIMESH_PERCEPTION__DEPTH_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "opencv2/core.hpp"
#include "pimesh_perception/depth_engine.hpp"
#include "pimesh_perception/mailbox.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "pimesh_msgs/msg/pipeline_stats.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace pimesh_perception
{

/// Monocular depth on the GPU: one decoded frame in, one metric depth map out.
///
/// **This is the pipeline's clock.** A frame costs ~55 ms here against a ~17 ms
/// frame interval — measured 17.42 Hz out of 59 Hz on bags/desk1, 2026-09-15 — so
/// this node keeps roughly one frame in three and drops the rest —
/// through the same one-slot mailbox every expensive stage here uses, because a
/// queue between a 59 Hz producer and a 15 Hz consumer is not a buffer, it is
/// latency with a nice name. Everything downstream of this node inherits its
/// rate, which is why `docs/info/pipeline.md` tells the fusion stage not to
/// assume 30 Hz input.
///
/// **It subscribes with a `ConstSharedPtr` and that is measured, not taste.**
/// `/image_raw` now has two consumers — `keypoint_node` and this one. rclcpp
/// serves *ownership-taking* subscriptions by moving the buffer into the last one
/// and **copying it for every other**; subscriptions taking a shared const
/// pointer go through a different path that hands one buffer to all of them.
/// With two `unique_ptr` consumers, 0 of 574 frames arrived at the published
/// address; with two `ConstSharedPtr` consumers it was 504/504. A `unique_ptr`
/// here would silently add a 2.7 MB copy per frame and `tools/gates/ipc.sh` would
/// start failing — which is exactly what it is for.
///
/// **Two topics, and the second one exists for synchronisation.** `/depth` is
/// 32FC1 metres. `/depth/rgb` is the *exact* frame that depth was inferred on,
/// republished unchanged. A separate republisher cannot serve that purpose: it
/// and this node drop *different* frames, so their stamp sets rarely intersect
/// and an exact-sync consumer limps at a fraction of either rate. Sourcing the
/// colour from the frame we actually processed makes every `/depth` message
/// pairable by construction.
///
/// **A third topic, `/depth/image/compressed`, is for a person and not for the
/// pipeline.** 32FC1 metres render as a near-black image in any viewer that does
/// not know what to do with them, and RViz's Image display has no colour map at
/// all — only a per-frame `Normalize Range`, which rescales every frame to its own
/// min and max so the *same distance is a different shade from frame to frame*.
/// So the colouring happens here, with `cv::COLORMAP_INFERNO` over a **fixed**
/// range of `[0, max_range]`: a colour then means a distance, the same one, in
/// every frame. Near is bright and far is black, which is the way round that puts
/// the emphasis on what the camera can actually see — the far clip is this
/// pipeline's "too far away or no idea", and it should be the quietest thing on
/// the screen rather than the loudest.
///
/// Like `keypoint_node`'s annotated preview it is rate-capped and its cost is
/// accounted separately from the per-frame budget, because encoding a JPEG is not
/// work the pipeline needs done.
///
/// **Both carry the input frame's stamp and `camera_optical_frame`.** Derived
/// data keeps the header of what it describes, not the moment the work finished.
/// That stamp is the Pi's kernel capture time (P1) and it is honest, so `/depth`
/// is genuinely dated to when the light arrived. The frame id is the static edge
/// `pimesh_bringup` publishes and unit-tests to be the optical convention — this
/// node does not re-derive that rotation, it names the frame.
class DepthNode : public rclcpp::Node
{
public:
  explicit DepthNode(const rclcpp::NodeOptions & options);
  ~DepthNode() override;

private:
  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void work();
  void process_frame(const sensor_msgs::msg::Image & msg);
  void publish_preview(const sensor_msgs::msg::Image & source);
  void log_stats();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr preview_pub_;
  /// `/pipeline/stats`. **This row is the one that teaches people to read the
  /// panel**: depth drops roughly two frames in three by design, and a column
  /// that conflated that with a transport loss would make the healthy pipeline
  /// and the broken one look identical. See PipelineStats.msg.
  rclcpp::Publisher<pimesh_msgs::msg::PipelineStats>::SharedPtr stats_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  std::unique_ptr<DepthEngine> engine_;
  Mailbox<sensor_msgs::msg::Image::ConstSharedPtr> mailbox_;
  std::thread worker_;

  std::string optical_frame_ {"camera_optical_frame"};
  float depth_scale_ {10.0F};
  float max_range_ {6.0F};
  bool publish_rgb_ {true};

  /// Reused across frames so the hot path allocates nothing: 3 MB of input
  /// tensor, 1 MB of model output, and the resized metre map. A per-frame
  /// allocation of that size is not fatal, it is just a cost paid 15 times a
  /// second for no reason.
  std::vector<float> input_;
  std::vector<float> output_;
  cv::Mat metres_small_;
  cv::Mat metres_;

  /// The inferno preview's working buffers, reused for the same reason as the
  /// tensors above: 8-bit depth, its colour-mapped twin, and the JPEG bytes.
  cv::Mat preview_8u_;
  cv::Mat preview_colour_;
  std::vector<unsigned char> jpeg_;

  int preview_quality_ {80};
  double preview_period_s_ {0.1};
  bool have_preview_time_ {false};
  rclcpp::Time last_preview_;

  // --- Counters for the stats line ------------------------------------------
  //
  // Written by the worker, read by the timer on an executor thread, hence atomic.
  // A racy count is a harmless wrong number, which is exactly the kind that ends
  // up quoted in a doc.
  std::atomic<std::uint64_t> frames_in_ {0};
  std::atomic<std::uint64_t> frames_out_ {0};
  std::atomic<std::uint64_t> failures_ {0};
  std::atomic<double> infer_sum_ms_ {0.0};
  std::atomic<double> total_sum_ms_ {0.0};
  std::atomic<double> total_max_ms_ {0.0};
  std::atomic<double> preview_sum_ms_ {0.0};
  std::atomic<std::uint64_t> previews_ {0};

  /// Every per-frame cost in the current stats window, so the summary can carry a
  /// **p95 and not only a mean**.
  ///
  /// A mean is the wrong statistic to be alone here. This node's consumers are
  /// paced by it, so what matters downstream is the tail: a stage that averages
  /// 60 ms and stalls for 200 ms four times a minute starves fusion in a way no
  /// mean will ever show, and `tools/gates/depth.sh` asserts its budget against
  /// this rather than against the average. The max is kept beside it because a
  /// p95 over a 5 s window is ~4 samples in and can miss the single worst one.
  ///
  /// A mutex rather than an atomic because it is a vector, and it is cheap for the
  /// same reason the vector stays small: this is pushed at the depth rate, ~15
  /// times a second, and drained by the stats timer. Contention here would require
  /// the timer and the worker to collide within a few microseconds of each other.
  std::mutex cost_mutex_;
  std::vector<double> window_costs_ms_;

  std::uint64_t last_logged_out_ {0};
  std::size_t last_logged_dropped_ {0};
  double last_infer_sum_ms_ {0.0};
  double last_total_sum_ms_ {0.0};
  double last_preview_sum_ms_ {0.0};
  std::uint64_t last_logged_previews_ {0};
  rclcpp::Time last_log_;
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__DEPTH_NODE_HPP_
