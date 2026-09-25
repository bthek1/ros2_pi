#ifndef PIMESH_DATASET__NODES__DATASET_NODE_HPP_
#define PIMESH_DATASET__NODES__DATASET_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "pimesh_dataset/dataset_reader.hpp"
#include "pimesh_msgs/msg/pipeline_stats.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace pimesh_dataset
{

/// `camera_node` with a public dataset where the sensor is.
///
/// **It stands in for the camera, exactly.** Same two topics, same QoS, same
/// frame, same `/pipeline/stats` row under the stage name `capture` — so nothing
/// downstream of `decode_node` can tell which one is publishing, which is the
/// only arrangement under which a trajectory measured against TUM's ground truth
/// says anything about the pipeline that runs on the Pi's camera.
///
/// It exists because this project had never measured its pose against anything
/// outside itself. `gates/odom.sh` asserts a reprojection residual and a
/// paired-surface gap, and P7 is the demonstration of how far that gets you: the
/// rotation had been composed inverted for six days and every internal number
/// describing it was right.
///
/// **Three things it deliberately does not do.**
///
/// It does not loop. A replay that restarts sends every `header.stamp`
/// backwards, `odometry_node` stamps TF with the frame's own stamp as it must,
/// and `tf2::BufferCore` then refuses everything for the rest of the run — the
/// failure `tools/view/replay.sh` is shaped around. There is no parameter for it
/// because there is no correct value.
///
/// It does not re-encode. TUM's frames are lossless PNG and they go on the wire
/// as PNG; `cv::imdecode` in `decode_node` reads either codec without being
/// told. Transcoding to JPEG to "match the camera" would put this node's
/// quality setting inside the number the gate reports, which is a variable
/// nobody asked for in a measurement whose whole point is the trajectory.
///
/// It does not invent intrinsics. They come from a standard `camera_info` YAML
/// through the same loader `camera_node` uses, and that loader is handed the
/// size of the **first decoded frame** — so serving the C922's 1280x720
/// calibration over Freiburg's 640x480 frames is a refusal to start rather than
/// an ATE that is really a measurement of our calibration against somebody
/// else's room.
class DatasetNode : public rclcpp::Node
{
public:
  explicit DatasetNode(const rclcpp::NodeOptions & options);
  ~DatasetNode() override;

private:
  void publish_loop();
  void publish_stats();
  sensor_msgs::msg::CameraInfo build_camera_info(std::uint32_t width, std::uint32_t height);

  std::vector<DatasetFrame> frames_;
  std::string frame_id_;
  double rate_scale_ {1.0};
  sensor_msgs::msg::CameraInfo camera_info_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<pimesh_msgs::msg::PipelineStats>::SharedPtr stats_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::Time last_stats_;
  std::uint64_t last_stats_frames_ {0};

  std::thread worker_;
  std::atomic<bool> running_ {true};
  /// Read by the stats timer on the executor thread and written by the publish
  /// thread — the same race `camera_node` made these atomic for in P10.
  std::atomic<std::uint64_t> published_ {0};
  std::atomic<std::uint64_t> late_ {0};
  std::atomic<bool> finished_ {false};
};

}  // namespace pimesh_dataset

#endif  // PIMESH_DATASET__NODES__DATASET_NODE_HPP_
