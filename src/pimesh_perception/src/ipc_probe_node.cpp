#include "pimesh_perception/ipc_probe_node.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace pimesh_perception
{

IpcProbeNode::IpcProbeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ipc_probe_node", options)
{
  rcl_interfaces::msg::ParameterDescriptor frames_desc;
  frames_desc.description =
    "how many frames to log an address for before going quiet. The gate needs "
    "a handful; logging every frame at 30 Hz would bury the launch output.";
  frames_desc.read_only = true;
  remaining_ = declare_parameter<int>("frames", 10, frames_desc);

  // ConstSharedPtr, not a copy. With intra-process comms on and only shared
  // subscribers, rclcpp wraps decode_node's unique_ptr in a shared_ptr and
  // hands the SAME allocation to each of them — no serialisation, no memcpy.
  // A by-value `sensor_msgs::msg::Image` callback would force a copy here and
  // make the address comparison fail for a real reason.
  //
  // QoS must match the publisher's or the two never connect, and a QoS
  // mismatch on an intra-process pair is silent: the topic simply has no
  // subscribers, which the gate would report as a broken container.
  sub_ = create_subscription<sensor_msgs::msg::Image>(
    "rgb/image", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    std::bind(&IpcProbeNode::on_image, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(), "probing %s for %ld frames (intra-process %s)",
    sub_->get_topic_name(), remaining_,
    options.use_intra_process_comms() ? "ON" : "OFF");
}

void IpcProbeNode::on_image(sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  ++received_;
  if (remaining_ <= 0) {
    return;
  }
  --remaining_;

  const int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  RCLCPP_INFO(
    get_logger(), "ipc received stamp=%ld buffer=%p %ux%u %s",
    stamp_ns, static_cast<const void *>(msg->data.data()),
    msg->width, msg->height, msg->encoding.c_str());

  if (remaining_ == 0) {
    RCLCPP_INFO(get_logger(), "ipc probe quiet now; %lu frames seen",
      static_cast<unsigned long>(received_));
  }
}

}  // namespace pimesh_perception

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_perception::IpcProbeNode)
