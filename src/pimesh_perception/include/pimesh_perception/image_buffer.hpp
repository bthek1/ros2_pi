#ifndef PIMESH_PERCEPTION__IMAGE_BUFFER_HPP_
#define PIMESH_PERCEPTION__IMAGE_BUFFER_HPP_

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

#include "opencv2/core.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace pimesh_perception
{

/// The `bgr8` encoding string, spelled once.
///
/// It is a string on the wire and a promise about memory layout, and the two are
/// only related by everybody agreeing. `rgb8` and `bgr8` differ by nothing a
/// subscriber can detect except this field, so a node that writes the wrong one
/// publishes frames with the red and blue channels swapped and no error
/// anywhere: the image merely looks like a badly white-balanced room, the ORB
/// detector works fine on it (it is grayscale by then), and the first place it
/// shows is a blue-tinted mesh several milestones later.
inline const char * bgr8_encoding() {return "bgr8";}

/// A cv::Mat *over* a message's pixels — no copy, no ownership.
///
/// This is the whole point of carrying the frame by pointer. The message holds
/// `height * step` bytes of BGR; a cv::Mat is a header plus a data pointer, so
/// the conversion from one to the other is arithmetic rather than a 2.7 MB
/// memcpy. The returned Mat is valid exactly as long as the message is, which is
/// why the argument is a reference and not a value: a Mat over a temporary's
/// buffer is a dangling pointer that reads correctly for a while.
///
/// Returns an empty Mat on anything it cannot vouch for rather than guessing.
/// A wrong `step` does not crash — it shears the image by one column per row,
/// which looks like a camera fault.
inline cv::Mat mat_over(const sensor_msgs::msg::Image & msg)
{
  if (msg.encoding != bgr8_encoding()) {return cv::Mat();}
  if (msg.width == 0 || msg.height == 0) {return cv::Mat();}

  const std::size_t step = (msg.step != 0) ? msg.step : static_cast<std::size_t>(msg.width) * 3;
  if (step < static_cast<std::size_t>(msg.width) * 3) {return cv::Mat();}
  if (msg.data.size() < step * msg.height) {return cv::Mat();}

  return cv::Mat(
    static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
    const_cast<std::uint8_t *>(msg.data.data()), step);
}

/// The `32FC1` encoding string, spelled once, for the same reason `bgr8` is.
inline const char * depth32f_encoding() {return "32FC1";}

/// `mat_over` for a 32FC1 depth map — a header over the message's own floats.
///
/// **It lives here rather than beside its one caller**, which is the habit this
/// project arrived at the hard way: `percentile` existed in four anonymous
/// namespaces and FNV-1a in two, none of them reachable by a test, and one of
/// them had been wrong since milestone A. This was a seventh copy of the same
/// shape — the mat-over-a-message arithmetic — sitting inside fusion_node.cpp
/// where nothing could call it.
///
/// Empty on anything it cannot vouch for, which is a refusal rather than a guess.
/// A wrong step on a float image does not look like the shear it produces on a
/// colour one; it looks like a room made of diagonal streaks, which is easy to
/// blame on the depth model.
///
/// **It is stricter than `mat_over` about `step == 0`**, and deliberately so
/// rather than by oversight: `mat_over` reads a zero as "not set, derive it",
/// because bgr8 arrives from publishers this workspace does not own. The only
/// 32FC1 publisher here is `depth_node`, which always sets it, so a zero on this
/// topic is a malformed message and not an omission. The asymmetry is written
/// down because the next person to see the two side by side will otherwise
/// "fix" one of them.
inline cv::Mat depth_mat_over(const sensor_msgs::msg::Image & msg)
{
  if (msg.encoding != depth32f_encoding()) {return cv::Mat();}
  if (msg.width == 0 || msg.height == 0) {return cv::Mat();}
  const std::size_t row_bytes = static_cast<std::size_t>(msg.width) * sizeof(float);
  if (msg.step < row_bytes) {return cv::Mat();}
  if (msg.data.size() < static_cast<std::size_t>(msg.step) * msg.height) {return cv::Mat();}
  return cv::Mat(
    static_cast<int>(msg.height), static_cast<int>(msg.width), CV_32FC1,
    const_cast<std::uint8_t *>(msg.data.data()), msg.step);
}

/// Size a message to hold `mat` and copy the pixels in.
///
/// The copy is real and is the price of publishing a `unique_ptr`: each message
/// is a fresh allocation, so there is nowhere for the decoder to have written
/// the pixels except its own buffer and then here. It is one pass over 2.7 MB —
/// measured in gates/ipc.sh as part of the decode cost — and it buys the thing
/// that matters, which is that this copy happens *once* in the process and every
/// consumer downstream gets the pointer.
inline void fill_bgr8(sensor_msgs::msg::Image & msg, const cv::Mat & mat)
{
  msg.encoding = bgr8_encoding();
  msg.height = static_cast<std::uint32_t>(mat.rows);
  msg.width = static_cast<std::uint32_t>(mat.cols);
  msg.is_bigendian = 0;
  msg.step = static_cast<std::uint32_t>(mat.cols) * 3;
  msg.data.resize(static_cast<std::size_t>(msg.step) * msg.height);

  // Row by row, because a cv::Mat is not necessarily contiguous — a Mat that is
  // a view into a larger one has a stride of the parent's width. memcpy over
  // `total() * elemSize()` is the version of this function that works on every
  // frame until the day somebody passes a cropped one.
  for (int row = 0; row < mat.rows; ++row) {
    std::memcpy(msg.data.data() + static_cast<std::size_t>(row) * msg.step,
      mat.ptr(row), static_cast<std::size_t>(msg.step));
  }
}

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__IMAGE_BUFFER_HPP_
