#ifndef PIMESH_CAMERA__CAMERA_INFO_HPP_
#define PIMESH_CAMERA__CAMERA_INFO_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "sensor_msgs/msg/camera_info.hpp"

namespace pimesh_camera
{

/// Build a `CameraInfo` for a single, unrectified pinhole camera.
///
/// Free function rather than a node method so that it can be tested without a
/// device: constructing `CameraNode` opens `/dev/video0`, which does not exist
/// on the dev box, and a matrix layout is exactly the sort of thing that is
/// silently wrong for months. R and P are the part worth guarding — a consumer
/// that reads P and finds zeros gets a projection that maps the whole image to
/// the origin, and nothing anywhere fails.
///
/// \param k  row-major 3x3 intrinsics; shorter input leaves the rest zero
/// \param d  plumb_bob coefficients k1 k2 p1 p2 k3
inline sensor_msgs::msg::CameraInfo make_camera_info(
  std::uint32_t width, std::uint32_t height,
  const std::vector<double> & k, const std::vector<double> & d)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = width;
  info.height = height;
  info.distortion_model = "plumb_bob";
  info.d = d;

  for (std::size_t i = 0; i < std::min<std::size_t>(info.k.size(), k.size()); ++i) {
    info.k[i] = k[i];
  }

  // R is identity: there is one camera and it is not rectified against another.
  info.r[0] = info.r[4] = info.r[8] = 1.0;

  // P is K with a zero fourth column, laid out 3x4 row-major:
  //
  //     [ fx   0  cx  Tx ]      p[0] p[1] p[2]  p[3]
  //     [  0  fy  cy  Ty ]  ->  p[4] p[5] p[6]  p[7]
  //     [  0   0   1   0 ]      p[8] p[9] p[10] p[11]
  //
  // Tx and Ty are zero for a monocular camera; they are the stereo baseline
  // term. K is 3x3 row-major, so fx=k[0], cx=k[2], fy=k[4], cy=k[5].
  info.p[0] = info.k[0];
  info.p[2] = info.k[2];
  info.p[5] = info.k[4];
  info.p[6] = info.k[5];
  info.p[10] = 1.0;

  return info;
}

}  // namespace pimesh_camera

#endif  // PIMESH_CAMERA__CAMERA_INFO_HPP_
