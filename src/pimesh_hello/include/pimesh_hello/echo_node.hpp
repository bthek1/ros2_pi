#ifndef PIMESH_HELLO__ECHO_NODE_HPP_
#define PIMESH_HELLO__ECHO_NODE_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace pimesh_hello
{

/// Receives what HelloNode publishes, and says where the message was.
///
/// The subscription callback takes a `std::unique_ptr`, which is not a style
/// choice: that signature is half of what makes the intra-process path eligible
/// to move the pointer instead of serialising the message. The other half is
/// the publisher moving a `unique_ptr` in. A `const &` callback here would work
/// perfectly and quietly copy.
class EchoNode : public rclcpp::Node
{
public:
  explicit EchoNode(const rclcpp::NodeOptions & options);

private:
  void on_message(std::unique_ptr<std_msgs::msg::String> msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Time last_receipt_;
  std::size_t count_ {0};
};

}  // namespace pimesh_hello

#endif  // PIMESH_HELLO__ECHO_NODE_HPP_
