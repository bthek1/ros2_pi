#include "pimesh_hello/echo_node.hpp"

#include <memory>
#include <string>
#include <utility>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace pimesh_hello
{

EchoNode::EchoNode(const rclcpp::NodeOptions & options)
: Node("echo_node", options),
  last_receipt_(0, 0, RCL_ROS_TIME)
{
  rcl_interfaces::msg::ParameterDescriptor topic_desc;
  topic_desc.description = "Topic to subscribe to.";
  const std::string topic =
    declare_parameter("input_topic", std::string("/hello_node/hello"), topic_desc);

  subscription_ = create_subscription<std_msgs::msg::String>(
    topic, 10,
    [this](std::unique_ptr<std_msgs::msg::String> msg) {this->on_message(std::move(msg));});

  RCLCPP_INFO(get_logger(), "subscribed to %s", topic.c_str());
}

void EchoNode::on_message(std::unique_ptr<std_msgs::msg::String> msg)
{
  // Receipt time, from this node's own clock.
  //
  // Not header.stamp — std_msgs/String has no header, which is a convenient
  // moment to internalise why this repo distrusts stamps anyway: usb_cam 0.8.1
  // puts absolute stamps a random sub-second in the past, redrawn every launch,
  // and a freshness gate built on them dropped 100% of frames. Deltas between
  // stamps are trustworthy because they are kernel capture intervals; the
  // absolute value is not. Measuring arrival on the receiver's clock sidesteps
  // the question entirely, and is what every gate in this workspace does.
  const rclcpp::Time now = this->now();
  const double dt_ms = (count_ == 0) ? 0.0 : (now - last_receipt_).seconds() * 1e3;
  last_receipt_ = now;
  ++count_;

  // The address is the evidence for P2. Logged as the raw pointer the callback
  // was handed: under intra-process this is the very object HelloNode built.
  RCLCPP_INFO(
    get_logger(), "echo n=%zu dt=%.1fms payload=%p \"%s\"",
    count_, dt_ms, static_cast<const void *>(msg.get()), msg->data.c_str());
}

}  // namespace pimesh_hello

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_hello::EchoNode)
