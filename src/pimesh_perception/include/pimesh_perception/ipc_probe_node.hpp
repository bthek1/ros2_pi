// A diagnostic component that exists to answer one question: is the container
// really passing buffers, or is it quietly serialising them?
//
// `use_intra_process_comms=True` is a request, not a guarantee. rclcpp falls
// back to a copy without complaint if the publisher hands it a stack value or a
// shared_ptr, if the QoS is transient_local, or if the two nodes turn out to be
// in different processes. All three failures look identical from the outside:
// the pipeline works, and the memory bandwidth quietly triples.
//
// So this node logs the address of the buffer it received. `just gate-ipc`
// compares it with the address decode_node logged when it published the same
// frame. Equal addresses cannot be produced by a serialised path — the copy
// would live somewhere else — which makes it evidence rather than a green tick.
//
// It is off by default and switched on by the gate (`probe:=true`). It stays
// in the tree rather than being deleted after P2 because every later phase adds
// a component to this container and can break the guarantee.

#ifndef PIMESH_PERCEPTION__IPC_PROBE_NODE_HPP_
#define PIMESH_PERCEPTION__IPC_PROBE_NODE_HPP_

#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace pimesh_perception
{

class IpcProbeNode : public rclcpp::Node
{
public:
  explicit IpcProbeNode(const rclcpp::NodeOptions & options);

private:
  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  int64_t remaining_{0};
  uint64_t received_{0};
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__IPC_PROBE_NODE_HPP_
