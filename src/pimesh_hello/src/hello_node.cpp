// The talker: a wall timer, a String, and two parameters that are declared
// properly. It is deliberately the least interesting node in the workspace —
// everything that matters here is the shape, not the payload.

#include "pimesh_hello/hello_node.hpp"

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace pimesh_hello
{

HelloNode::HelloNode(const rclcpp::NodeOptions & options)
: Node("hello_node", options)
{
  // Declared with a descriptor, not bare. The description is what
  // `ros2 param describe` shows a person who did not write this node, and the
  // range is enforced by rclcpp itself — at declaration, against the value from
  // the command line or the YAML, and again on every later set. That is the
  // difference between a documented parameter and a hopeful one.
  rcl_interfaces::msg::ParameterDescriptor rate_desc;
  rate_desc.description = "Publish rate in hertz.";
  rcl_interfaces::msg::FloatingPointRange rate_range;
  rate_range.from_value = 0.1;
  rate_range.to_value = 100.0;
  rate_desc.floating_point_range.push_back(rate_range);
  const double rate_hz = declare_parameter("rate_hz", 1.0, rate_desc);

  rcl_interfaces::msg::ParameterDescriptor text_desc;
  text_desc.description = "The string to publish, once per tick.";
  text_ = declare_parameter("text", std::string("hello world"), text_desc);

  // `~/hello` — the private namespace, so the topic is /hello_node/hello and
  // follows the node if it is ever renamed or pushed into a namespace. A
  // hard-coded "/hello" would collide the moment two of these ran at once.
  //
  // Default QoS: RELIABLE, KEEP_LAST(10). That is fine *because the message is
  // twenty bytes*. The repo-wide KEEP_LAST(1) rule is about megabyte-class
  // images, where a depth-10 queue is not a buffer but a third of a second of
  // stale video — see docs/info/architecture.md.
  publisher_ = create_publisher<std_msgs::msg::String>("~/hello", 10);

  // duration<double> keeps the division in floating point; the cast to
  // nanoseconds at the end is the only rounding that happens.
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_hz));
  timer_ = create_wall_timer(period, [this]() {this->tick();});

  RCLCPP_INFO(
    get_logger(), "publishing \"%s\" on %s at %.2f Hz",
    text_.c_str(), publisher_->get_topic_name(), rate_hz);
}

void HelloNode::tick()
{
  // A unique_ptr, moved into publish(). This exact pairing — a unique_ptr in,
  // a unique_ptr-taking subscription callback out — is what makes the
  // intra-process path eligible to hand the pointer over instead of serialising
  // the message. P2 proves it; publishing this way from the start means P2 adds
  // a subscriber and nothing else.
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = text_;

  // Read the address *before* the move — afterwards this unique_ptr is null.
  // This is the publisher half of P2's evidence.
  ++seq_;
  RCLCPP_INFO(
    get_logger(), "pub seq=%zu payload=%p",
    seq_, static_cast<const void *>(msg.get()));

  publisher_->publish(std::move(msg));
}

}  // namespace pimesh_hello

// Registers the class in the ament index under its fully-qualified name, so a
// container can find and construct it at runtime knowing only that string. The
// standalone binary does not use this; the container uses nothing else.
RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_hello::HelloNode)
