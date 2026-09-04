// The container's only network subscriber, and its only JPEG decode.
//
// Everything about this node's shape follows from two constraints in CLAUDE.md:
//
//   * **One reader on the Wi-Fi link.** Five RELIABLE subscribers each pull
//     their own unicast copy from the Pi and collapse the link — the
//     predecessor measured ~2 frames/s per reader against 14.7 Hz for one. So
//     exactly one node subscribes `/image_raw/compressed`, and everything else
//     consumes its output.
//   * **No work in a subscription callback beyond a bounded copy.** The
//     callback moves a shared_ptr into a one-deep mailbox and returns; a worker
//     thread does the ~4 ms decode. An executor thread that blocks on imdecode
//     is an executor thread not delivering the next frame.
//
// The output is published as a `unique_ptr`, which is what makes the rest of
// the container zero-copy: with intra-process comms on, rclcpp hands the same
// buffer to every subscriber in this process instead of serialising it. Publish
// a stack copy or a shared_ptr instead and it silently falls back to copying —
// which is why `just gate-ipc` compares the actual addresses rather than
// trusting the launch flag.

#ifndef PIMESH_PERCEPTION__DECODE_NODE_HPP_
#define PIMESH_PERCEPTION__DECODE_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pimesh_msgs/msg/pipeline_stats.hpp>

#include "pimesh_perception/mailbox.hpp"

namespace pimesh_perception
{

class DecodeNode : public rclcpp::Node
{
public:
  explicit DecodeNode(const rclcpp::NodeOptions & options);
  ~DecodeNode() override;

private:
  using CompressedConstPtr = sensor_msgs::msg::CompressedImage::ConstSharedPtr;

  void on_frame(CompressedConstPtr msg);
  void decode_loop();
  void publish_stats();

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<pimesh_msgs::msg::PipelineStats>::SharedPtr stats_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  Mailbox<CompressedConstPtr> mailbox_;
  std::thread worker_;
  std::atomic<bool> running_{true};

  cv::Mat bgr_;                     // reused across frames; the worker owns it
  double stale_after_s_{2.0};
  int64_t log_addresses_{0};        // frames left to log; 0 means never
  std::atomic<uint64_t> decoded_{0};
  std::atomic<uint64_t> failed_{0};

  // Written by the worker, read by the stats timer on the executor's thread.
  std::mutex window_mutex_;
  std::vector<double> window_ms_;
  std::chrono::steady_clock::time_point window_start_;
  std::chrono::steady_clock::time_point last_frame_;
  bool ever_received_{false};
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__DECODE_NODE_HPP_
