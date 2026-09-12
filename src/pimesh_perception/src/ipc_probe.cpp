// The instrument tools/gates/ipc.sh measures with, as a component rather than a
// program — which is the only shape that can answer the question.
//
// The claim under test is that a 2.7 MB frame crosses from decode_node to its
// consumers as a pointer. That is a statement about what happens *inside one
// process*, so the measuring subscriber has to be loaded into that process; a
// standalone `ros2 run` probe would be measuring the serialised path by
// construction and would report a mismatch that means nothing. Hence no
// `*_main.cpp` beside this file: a thin main here would be a binary whose only
// use is to produce a misleading result.
//
// It prints one line per frame with the address of the buffer it was handed. The
// gate compares those against the addresses decode_node logged, with
// intra-process comms on and then off, because address equality on its own is
// not evidence — two allocations in one process can coincide, and one did, at
// 1/22, in the hello-world run on 2026-09-08.

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "pimesh_perception/image_buffer.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace pimesh_perception
{

class IpcProbe : public rclcpp::Node
{
public:
  explicit IpcProbe(const rclcpp::NodeOptions & options)
  : Node("ipc_probe", options)
  {
    rcl_interfaces::msg::ParameterDescriptor topic_desc;
    topic_desc.description = "Decoded image topic to watch.";
    const std::string topic =
      declare_parameter("input_topic", std::string("/image_raw"), topic_desc);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.reliable();

    // **A shared const pointer, and the gate's whole result hangs on this choice.**
    //
    // The intuitive signature is `std::unique_ptr`, on the reasoning that rclcpp can
    // only hand a buffer over to a callback willing to own it. That is true of a
    // topic with one consumer and false as soon as there are two: rclcpp serves
    // ownership-taking subscriptions by moving into the *last* one and copying for
    // each of the others, so with this probe and keypoint_node both asking to own
    // the frame, one of them received a 2.7 MB copy. Measured 2026-09-12: **0 of
    // 574** frames matched the published address that way, against every frame when
    // both take a shared const pointer — which rclcpp serves by handing the same
    // buffer to all of them (`add_shared_msg_to_buffers`).
    //
    // So this probe deliberately measures the configuration the pipeline actually
    // uses. A probe that took ownership would report a failure caused by its own
    // presence, which is the worst kind of instrument.
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      topic, qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {this->on_frame(std::move(msg));});

    RCLCPP_INFO(get_logger(), "probing %s for pointer handover", topic.c_str());
  }

private:
  void on_frame(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    ++count_;

    // The address of the message object, which is what the publisher logged
    // before moving it. Also the address of the pixel buffer: a serialised copy
    // would give a different one for both, and printing both makes it impossible
    // to read a coincidence on one as a handover.
    //
    // Touching a pixel is deliberate. A probe that only reads the pointer would
    // pass over a message whose `data` had been moved out from under it, and
    // "the pointer arrived" is a weaker claim than "the pixels arrived in it".
    const cv::Mat frame = mat_over(*msg);
    const int centre = frame.empty() ? -1 :
      static_cast<int>(frame.at<cv::Vec3b>(frame.rows / 2, frame.cols / 2)[1]);

    RCLCPP_INFO(
      get_logger(), "probe n=%lu payload=%p pixels=%p %ux%u centre_g=%d",
      static_cast<unsigned long>(count_), static_cast<const void *>(msg.get()),
      static_cast<const void *>(msg->data.data()), msg->width, msg->height, centre);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::uint64_t count_ {0};
};

}  // namespace pimesh_perception

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_perception::IpcProbe)
