#ifndef PIMESH_PERCEPTION__DECODE_NODE_HPP_
#define PIMESH_PERCEPTION__DECODE_NODE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "opencv2/core.hpp"
#include "pimesh_perception/mailbox.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace pimesh_perception
{

/// The container's only network subscriber: JPEG in, `bgr8` out, once.
///
/// Every expensive stage on the dev box wants the same pixels, and the
/// predecessor let each of them subscribe to the Pi's stream directly. Five
/// RELIABLE readers over Wi-Fi each pull their own unicast copy, and the link
/// collapsed into a retransmit storm — ~2 frames/s per reader against 14.7 Hz
/// for a single one. So this node exists to be the *one* reader, and to be the
/// one decode: `cv::imdecode` costs milliseconds, and doing it per consumer
/// would be paying for the Wi-Fi mistake a second time inside the process.
///
/// What it publishes is a `std::unique_ptr<Image>` moved into `publish()`. Inside
/// a container with `use_intra_process_comms`, a subscriber whose callback also
/// takes a `unique_ptr` is then handed that very buffer — 2.7 MB transferred as
/// eight bytes. Both halves are required and neither announces itself: a
/// `const &` callback downstream works perfectly and quietly copies, which is
/// why `tools/gates/ipc.sh` compares the addresses with the mechanism switched
/// on and off rather than trusting either end's good intentions.
///
/// The decode runs on its own thread behind a one-slot mailbox. A 4 ms
/// `imdecode` in the subscription callback would hold an executor thread for 4 ms
/// out of every 17, which works until the next stage is added and is the habit
/// this repo refuses to start.
class DecodeNode : public rclcpp::Node
{
public:
  explicit DecodeNode(const rclcpp::NodeOptions & options);
  ~DecodeNode() override;

private:
  void on_frame(std::unique_ptr<sensor_msgs::msg::CompressedImage> msg);
  void work();
  void decode_one(std::unique_ptr<sensor_msgs::msg::CompressedImage> msg);
  void log_stats();

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  /// A `unique_ptr` slot, which is right here for a reason that does not apply to
  /// the stage after this one: this subscription is on the *inter-process* topic
  /// from the Pi, where the middleware constructs a fresh message for us in any
  /// case, so owning it costs nothing and lets the decoder work in place.
  Mailbox<std::unique_ptr<sensor_msgs::msg::CompressedImage>> mailbox_;
  std::thread worker_;

  bool log_payloads_ {false};

  /// Reused across frames, so the decoder allocates its 2.7 MB once rather than
  /// 59 times a second. The message's buffer cannot serve this purpose: it is a
  /// fresh allocation per publish by construction.
  cv::Mat bgr_;

  // --- Counters, for the stats line -----------------------------------------
  //
  // Touched by the worker thread and read by the timer on an executor thread,
  // hence atomic. A racy frame count would be a harmless wrong number, which is
  // exactly the kind of wrong number that gets quoted in a doc.
  std::atomic<std::uint64_t> frames_in_ {0};
  std::atomic<std::uint64_t> frames_out_ {0};
  std::atomic<std::uint64_t> failures_ {0};
  std::atomic<std::uint64_t> gaps_ {0};
  std::atomic<double> cost_sum_ms_ {0.0};
  std::atomic<double> cost_max_ms_ {0.0};
  double gap_threshold_ms_ {50.0};

  std::uint64_t last_logged_in_ {0};
  std::uint64_t last_logged_out_ {0};
  std::size_t last_logged_dropped_ {0};
  rclcpp::Time last_log_;
  rclcpp::Time last_arrival_;
  bool have_arrival_ {false};
  std::uint64_t seq_ {0};
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__DECODE_NODE_HPP_
