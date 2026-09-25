// Replay a public RGB-D sequence into the pipeline as if it were the camera.
//
// The interesting decisions in this file are all refusals, and they are in the
// constructor: it either comes up serving the dataset's own frames at the
// dataset's own stamps with the dataset's own intrinsics, or it does not come up.

#include "pimesh_dataset/nodes/dataset_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "opencv2/imgcodecs.hpp"
#include "pimesh_camera/calibration.hpp"
#include "pimesh_camera/camera_info.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace pimesh_dataset
{
namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & text)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = text;
  return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describe_range_double(
  const std::string & text, double low, double high)
{
  auto descriptor = describe(text);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = low;
  range.to_value = high;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

/// The image file's bytes, as they are on disk.
std::vector<unsigned char> read_file(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  return std::vector<unsigned char>(
    std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

/// `"rgb/1305031452.791720.png"` -> `"png"`, which is what a CompressedImage's
/// `format` field carries and what a viewer's transport hint reads.
std::string extension_of(const std::string & path)
{
  const std::size_t dot = path.rfind('.');
  if (dot == std::string::npos) {return "jpeg";}
  std::string ext = path.substr(dot + 1);
  std::transform(
    ext.begin(), ext.end(), ext.begin(),
    [](unsigned char c) {return static_cast<char>(::tolower(c));});
  // image_transport spells it `jpeg`, and `jpg` on the wire is a format string
  // some readers do not match. The codec is the same one either way.
  if (ext == "jpg") {return "jpeg";}
  return ext;
}

}  // namespace

DatasetNode::DatasetNode(const rclcpp::NodeOptions & options)
: Node("dataset_node", options)
{
  // No default, on purpose. Every other path-shaped parameter in this workspace
  // has one because there is a right answer (the calibration, the model); there
  // is no right dataset, and a node that came up replaying whatever happened to
  // be in a hard-coded directory is a second publisher on the topic the Pi uses.
  const auto dataset_dir = declare_parameter(
    "dataset_dir", std::string(),
    describe(
      "Directory of an unpacked TUM-format sequence — the one holding rgb.txt, "
      "rgb/ and groundtruth.txt. Empty is a refusal to start, not a default. "
      "bash tools/fetch-dataset.sh --print-path prints it."));
  const auto index_name = declare_parameter(
    "index_name", std::string("rgb.txt"),
    describe("The frame index inside that directory: '<timestamp> <path>' per line."));

  frame_id_ = declare_parameter(
    "frame_id", std::string("camera_optical_frame"),
    describe(
      "TF frame the image plane is in. The optical convention, always — the same "
      "string camera_node stamps with, because nothing downstream may be able to "
      "tell which of the two is publishing."));

  rate_scale_ = declare_parameter(
    "rate_scale", 1.0,
    describe_range_double(
      "Multiplier on the dataset's own frame rate. 1.0 replays it in real time, "
      "which is the only setting a measured rate means anything at. Below 1.0 "
      "gives the pipeline more time per frame and is a diagnostic, not a run.",
      0.05, 10.0));

  const auto max_frames = static_cast<std::size_t>(
    declare_parameter(
      "max_frames", 0,
      describe("Stop after this many frames. 0 replays the whole sequence.")));

  const double stats_period_s = declare_parameter(
    "stats_period_s", 0.1,
    describe_range_double(
      "How often to publish /pipeline/stats under the stage name 'capture'. "
      "camera_node's cadence, because this node is standing in for it.",
      0.02, 60.0));

  if (dataset_dir.empty()) {
    throw std::runtime_error(
      "dataset_node needs dataset_dir set to an unpacked TUM sequence. There is "
      "no default: a replay source that starts on its own would be a second "
      "publisher on /image_raw/compressed. Get one with "
      "`bash tools/fetch-dataset.sh` and pass "
      "`source:=dataset dataset_dir:=$(bash tools/fetch-dataset.sh --print-path)`.");
  }

  const FrameList list = read_tum_index(dataset_dir, index_name);
  if (!list.ok) {
    throw std::runtime_error("dataset_node cannot read the sequence: " + list.why);
  }
  frames_ = list.frames;
  if (max_frames > 0 && frames_.size() > max_frames) {frames_.resize(max_frames);}

  // **Decode the first frame here, and only to learn its size.** The size is
  // what makes the calibration check an assertion rather than a comment: the
  // loader refuses a YAML whose image_width/image_height disagree with what it
  // is handed, so publishing the C922's 1280x720 intrinsics over 640x480 frames
  // stops this node from starting. The alternative — trusting a parameter to say
  // how big the frames are — is two numbers that agree only because somebody
  // typed both.
  const cv::Mat first = cv::imread(frames_.front().path, cv::IMREAD_COLOR);
  if (first.empty()) {
    throw std::runtime_error(
      "dataset_node could not decode the first frame, " + frames_.front().path +
      " — the index lists it and OpenCV cannot read it");
  }
  camera_info_ = build_camera_info(
    static_cast<std::uint32_t>(first.cols), static_cast<std::uint32_t>(first.rows));

  // camera_node's QoS, both topics, for the reason in its own comment: RELIABLE
  // because BEST_EFFORT delivers zero large frames, KEEP_LAST(1) because the
  // freshest frame is the only one anybody wants — and here also because a
  // consumer that fell behind must drop frames rather than replay a stale room
  // at the wrong stamp.
  rclcpp::QoS image_qos(rclcpp::KeepLast(1));
  image_qos.reliable();
  rclcpp::QoS info_qos(rclcpp::KeepLast(1));
  info_qos.reliable().transient_local();

  const auto image_topic = declare_parameter(
    "image_topic", std::string("/image_raw/compressed"),
    describe("Where the frames go. camera_node's topic, because this replaces it."));
  const auto info_topic = declare_parameter(
    "info_topic", std::string("/camera_info"),
    describe("Where the dataset's intrinsics go. camera_node's topic, likewise."));

  image_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(image_topic, image_qos);
  info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, info_qos);
  stats_pub_ = create_publisher<pimesh_msgs::msg::PipelineStats>("/pipeline/stats", 10);

  const double span_s =
    static_cast<double>(frames_.back().stamp_ns - frames_.front().stamp_ns) * 1e-9;
  RCLCPP_INFO(
    get_logger(),
    "dataset %s: %zu frames, %.1fs, %.2f Hz, %ux%u, replaying at %.2fx into %s",
    dataset_dir.c_str(), frames_.size(), span_s,
    (span_s > 0.0) ? static_cast<double>(frames_.size() - 1) / span_s : 0.0,
    camera_info_.width, camera_info_.height, rate_scale_, image_pub_->get_topic_name());
  RCLCPP_INFO(
    get_logger(), "dataset intrinsics fx=%.4f fy=%.4f cx=%.4f cy=%.4f in frame '%s'",
    camera_info_.k[0], camera_info_.k[4], camera_info_.k[2], camera_info_.k[5],
    frame_id_.c_str());

  last_stats_ = now();
  stats_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(stats_period_s)),
    [this] {this->publish_stats();});

  // A thread, not a timer, and for camera_node's reason one step removed: the
  // loop sleeps to the next frame's own deadline, and a sleep on an executor
  // thread holds the callback group for as long as it lasts.
  worker_ = std::thread([this] {this->publish_loop();});
}

DatasetNode::~DatasetNode()
{
  running_ = false;
  if (worker_.joinable()) {worker_.join();}
}

sensor_msgs::msg::CameraInfo DatasetNode::build_camera_info(
  std::uint32_t width, std::uint32_t height)
{
  const auto url = declare_parameter(
    "camera_info_url", std::string(),
    describe(
      "Standard camera_info YAML for this sequence, as package://, file:// or a "
      "path. Empty is a refusal: a dataset's intrinsics are a fact about "
      "somebody else's camera and there is nothing sensible to fall back to."));

  if (url.empty()) {
    throw std::runtime_error(
      "dataset_node needs camera_info_url. camera_node falls back to nominal "
      "intrinsics and warns, which is right for a camera whose calibration has "
      "not been run yet; here there is no such thing as a nominal value — the "
      "numbers belong to the rig that recorded the sequence. Serving anything "
      "else makes every unprojection wrong by a constant factor and the "
      "resulting trajectory a measurement of the mismatch.");
  }

  // **The same loader camera_node uses, handed the frame size measured off the
  // frames.** That is what turns "do not serve the wrong intrinsics" from a note
  // into a refusal: `load_calibration` rejects a YAML whose image_width or
  // image_height disagrees with what it is told the stream is.
  const pimesh_camera::Calibration cal = pimesh_camera::load_calibration(url, width, height);
  if (!cal.ok) {
    throw std::runtime_error(
      "dataset_node cannot use camera_info_url '" + url + "' (" + cal.path + "): " + cal.why);
  }

  RCLCPP_INFO(
    get_logger(), "intrinsics '%s' from %s", cal.camera_name.c_str(), cal.path.c_str());
  return pimesh_camera::make_camera_info(width, height, cal.k, cal.d);
}

void DatasetNode::publish_loop()
{
  // Absolute deadlines from one origin, never `sleep_for(interval)` in a loop:
  // the second accumulates every scheduling delay, so a 20 s clip finishes late
  // by however much the machine was busy and the *rate* the pipeline saw is not
  // the rate the dataset recorded.
  const auto origin = std::chrono::steady_clock::now();
  const std::int64_t first_ns = frames_.front().stamp_ns;

  for (std::size_t i = 0; i < frames_.size() && running_ && rclcpp::ok(); ++i) {
    const DatasetFrame & frame = frames_[i];
    const double offset_s =
      static_cast<double>(frame.stamp_ns - first_ns) * 1e-9 / rate_scale_;
    const auto due = origin + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(offset_s));

    const auto before = std::chrono::steady_clock::now();
    if (before < due) {
      std::this_thread::sleep_for(due - before);
    } else if (i > 0 && (before - due) > std::chrono::milliseconds(20)) {
      // Counted rather than logged per frame. A replay that cannot keep up is
      // publishing a 30 Hz sequence at some other rate, and the trajectory is
      // then measured against a clip the pipeline never saw at its own speed.
      ++late_;
    }

    auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
    // **The dataset's own stamp**, not `now()`. It is what the ground-truth file
    // is indexed by, so a trajectory written out of this pipeline associates
    // against that file directly — no offset to fit, nothing to get wrong. It is
    // also 2011, which is fine: nothing in this pipeline compares a header stamp
    // to a wall clock (see the constraint in CLAUDE.md about stamp-age gates),
    // and every delta between two of them is a real capture interval.
    msg->header.stamp.sec = static_cast<std::int32_t>(frame.stamp_ns / 1000000000LL);
    msg->header.stamp.nanosec = static_cast<std::uint32_t>(frame.stamp_ns % 1000000000LL);
    msg->header.frame_id = frame_id_;
    msg->format = extension_of(frame.path);
    msg->data = read_file(frame.path);
    if (msg->data.empty()) {
      RCLCPP_WARN(get_logger(), "empty read on %s — skipped", frame.path.c_str());
      continue;
    }

    // The intrinsics go out beside every frame, on the same stamp, like
    // camera_node's do. Transient-local means a late subscriber gets one anyway;
    // republishing costs a few hundred bytes in-process.
    camera_info_.header = msg->header;
    info_pub_->publish(camera_info_);
    image_pub_->publish(std::move(msg));
    ++published_;
  }

  finished_ = true;
  if (running_) {
    // Said once, clearly, because the difference between "the clip ended" and
    // "the source died" is the difference between reading a result and
    // debugging one — and a gate that kept measuring past the end would be
    // averaging the idle tail into its numbers.
    RCLCPP_INFO(
      get_logger(), "dataset finished: %lu frames published, %lu behind schedule",
      static_cast<unsigned long>(published_.load()), static_cast<unsigned long>(late_.load()));
  }
}

void DatasetNode::publish_stats()
{
  const rclcpp::Time stamp = now();
  const double span_s = (stamp - last_stats_).seconds();
  if (span_s <= 0.0) {return;}

  const std::uint64_t total = published_.load();
  const std::uint64_t delta = total - last_stats_frames_;
  last_stats_frames_ = total;
  last_stats_ = stamp;

  auto msg = std::make_unique<pimesh_msgs::msg::PipelineStats>();
  msg->header.stamp = stamp;
  // **`capture`, not `dataset`.** The dashboard keys its headline figure on this
  // string, and a stage name nobody publishes shows as a permanent em dash —
  // identical to how it draws a stage that has not reported yet. This node is
  // the capture stage for the length of a replay; saying otherwise would make
  // the page wrong about something rather than silent about it.
  msg->stage = "capture";
  msg->rate_hz = static_cast<float>(static_cast<double>(delta) / span_s);
  // Not measured, and not invented. There is no capture latency here: the frames
  // were captured in Freiburg in 2011. A zero would read as "instant", which is
  // the `cost_mean=0.00` failure; camera_node refuses to invent this field for
  // the same reason and leaves it at its default.
  msg->frames_in = total;
  msg->frames_out = total;
  msg->detail = finished_.load() ? "dataset replay finished" : "dataset replay";
  if (late_.load() > 0) {
    msg->detail += ", " + std::to_string(late_.load()) + " frames behind schedule";
  }
  stats_pub_->publish(std::move(msg));
}

}  // namespace pimesh_dataset

RCLCPP_COMPONENTS_REGISTER_NODE(pimesh_dataset::DatasetNode)
