// Monocular depth on the GPU, and the pipeline's clock.
//
// Everything downstream of this node runs at its rate, because nothing can
// integrate a depth map that does not exist yet. At ~53 ms a frame that is
// ~19 Hz against a camera delivering 42-60, so **most frames are dropped, and
// that is the design**: the one-deep mailbox keeps the newest and discards the
// rest, because a frame the camera took 400 ms ago has nothing to offer a live
// reconstruction once a newer one exists. `dropped_mailbox` on this stage is
// expected to be large; a growing *backlog* would be the fault, and there
// cannot be one behind a mailbox that is one deep.
//
// Two topics, and the second one is not redundant. `/depth` is the map;
// `/depth/rgb` is **the exact frame it was inferred on**, republished with the
// same stamp. A separate republisher cannot do this job: it and this node drop
// *different* frames, so their stamp sets rarely intersect and an exact-time
// synchroniser downstream limps along at a couple of hertz (the predecessor
// measured that, and no queue depth fixed it). Sourcing the RGB from the frame
// we actually processed makes every pair exact by construction.

#ifndef PIMESH_PERCEPTION__DEPTH_NODE_HPP_
#define PIMESH_PERCEPTION__DEPTH_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pimesh_msgs/msg/pipeline_stats.hpp>

#include "pimesh_perception/depth_model.hpp"
#include "pimesh_perception/mailbox.hpp"

namespace pimesh_perception
{

class DepthNode : public rclcpp::Node
{
public:
  explicit DepthNode(const rclcpp::NodeOptions & options);
  ~DepthNode() override;

private:
  using ImageConstPtr = sensor_msgs::msg::Image::ConstSharedPtr;

  void on_frame(ImageConstPtr msg);
  void work_loop();
  void publish_stats();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<pimesh_msgs::msg::PipelineStats>::SharedPtr stats_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  Mailbox<ImageConstPtr> mailbox_;
  std::thread worker_;
  std::atomic<bool> running_{true};

  // Constructed on the WORKER thread, not here: loading the model and warming
  // a CUDA session takes seconds, and doing that in the constructor would
  // block the component container while every other component waits to load.
  std::unique_ptr<DepthModel> model_;
  DepthModel::Options model_options_;
  std::atomic<bool> ready_{false};
  std::atomic<bool> load_failed_{false};

  cv::Mat depth_;                   // reused across frames; the worker owns it
  double stale_after_s_{5.0};
  std::string optical_frame_{"camera_optical_frame"};

  std::atomic<uint64_t> processed_{0};
  std::atomic<uint64_t> skipped_{0};

  std::mutex window_mutex_;
  std::vector<double> window_ms_;
  std::chrono::steady_clock::time_point window_start_;
  std::chrono::steady_clock::time_point last_frame_;
  bool ever_received_{false};
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__DEPTH_NODE_HPP_
