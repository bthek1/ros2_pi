// JPEG → BGR8, as a free function with no ROS in it.
//
// It is separated from decode_node for the same reason `to_system_clock_ns` was
// pulled out of `wait_frame`: the node needs a camera and a container, the
// arithmetic does not. A synthetic JPEG made with cv::imencode exercises this
// on any machine — see test/test_jpeg.cpp.

#ifndef PIMESH_PERCEPTION__JPEG_HPP_
#define PIMESH_PERCEPTION__JPEG_HPP_

#include <cstdint>
#include <vector>

#include <opencv2/core.hpp>

namespace pimesh_perception
{

/// Decode compressed image bytes into a BGR8 `cv::Mat`.
///
/// Returns false on anything OpenCV cannot decode — a truncated frame, a
/// corrupt one, an empty buffer. **A bad frame is a transport fault, not a
/// reason to die**: the node counts it in `dropped_transport` and waits for
/// the next one. Over Wi-Fi, a frame that fails to reassemble is a normal
/// event, and a decode stage that threw on one would take the pipeline down
/// every few minutes.
///
/// `bgr` is reused across calls when its size and type already match, so the
/// steady state allocates nothing.
bool decode_bgr8(const std::vector<uint8_t> & compressed, cv::Mat & bgr);

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__JPEG_HPP_
