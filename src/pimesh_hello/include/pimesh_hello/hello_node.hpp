#ifndef PIMESH_HELLO__HELLO_NODE_HPP_
#define PIMESH_HELLO__HELLO_NODE_HPP_

#include <cstddef>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace pimesh_hello
{

/// Publishes a fixed string on `~/hello` at a configurable rate.
///
/// The class lives in a header, rather than entirely inside its .cpp, because
/// two things construct it: the component container (through the ament index,
/// by name) and the standalone main (through this declaration, by type). Both
/// get the same object — which is the whole point of the exercise.
class HelloNode : public rclcpp::Node
{
public:
  /// The signature rclcpp_components requires. A component with any other
  /// constructor runs standalone and fails to load in a container.
  explicit HelloNode(const rclcpp::NodeOptions & options);

private:
  void tick();

  std::string text_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::size_t seq_ {0};
};

}  // namespace pimesh_hello

#endif  // PIMESH_HELLO__HELLO_NODE_HPP_
