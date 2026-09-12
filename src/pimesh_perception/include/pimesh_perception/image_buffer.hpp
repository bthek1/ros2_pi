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
