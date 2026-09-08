#include "pimesh_perception/depth_convert.hpp"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace pimesh_perception
{

const float kImagenetMean[3] = {0.485f, 0.456f, 0.406f};
const float kImagenetStd[3] = {0.229f, 0.224f, 0.225f};

void preprocess_frame(const cv::Mat & bgr, int side, std::vector<float> & out)
{
  if (side <= 0 || side % 14 != 0) {
    throw std::invalid_argument(
      "input side must be a positive multiple of 14 — the ViT works on 14x14 patches");
  }
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    throw std::invalid_argument("preprocess_frame expects a non-empty 8-bit BGR image");
  }

  // INTER_AREA for the downscale: it averages the pixels it discards instead
  // of point-sampling them, which is what stops a 1280x720 frame aliasing into
  // 518x518 and handing the transformer texture that was never in the room.
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(side, side), 0, 0, cv::INTER_AREA);
  cv::Mat rgb;
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
  cv::Mat unit;
  rgb.convertTo(unit, CV_32FC3, 1.0 / 255.0);

  const size_t plane = static_cast<size_t>(side) * static_cast<size_t>(side);
  out.assign(3 * plane, 0.0f);

  // HWC to CHW, normalising on the way. The model wants three contiguous
  // planes and OpenCV holds interleaved pixels, so the transpose is
  // unavoidable; doing it in the same pass as the normalisation at least
  // touches the data once instead of twice.
  for (int y = 0; y < side; ++y) {
    const auto * row = unit.ptr<cv::Vec3f>(y);
    for (int x = 0; x < side; ++x) {
      const size_t i = static_cast<size_t>(y) * static_cast<size_t>(side) +
        static_cast<size_t>(x);
      for (int c = 0; c < 3; ++c) {
        out[c * plane + i] = (row[x][c] - kImagenetMean[c]) / kImagenetStd[c];
      }
    }
  }
}

void relative_to_metres(
  const cv::Mat & relative, double depth_scale, double max_depth_m, cv::Mat & metres)
{
  if (depth_scale <= 0.0 || max_depth_m <= 0.0) {
    throw std::invalid_argument("depth_scale and max_depth_m must be positive");
  }

  // z = depth_scale / relative, bounded at max_depth_m. Flooring the
  // denominator at depth_scale/max_depth_m makes the bound hold by
  // construction: the largest value the division can produce IS max_depth_m,
  // so there is no clipping pass afterwards and no 1/x on a near-zero value.
  const double floor_value = depth_scale / max_depth_m;
  cv::Mat bounded;
  cv::max(relative, floor_value, bounded);
  cv::divide(depth_scale, bounded, metres, 1.0, CV_32FC1);
}

}  // namespace pimesh_perception
