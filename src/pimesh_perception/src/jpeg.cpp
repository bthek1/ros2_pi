#include "pimesh_perception/jpeg.hpp"

#include <opencv2/imgcodecs.hpp>

namespace pimesh_perception
{

bool decode_bgr8(const std::vector<uint8_t> & compressed, cv::Mat & bgr)
{
  if (compressed.empty()) {
    return false;
  }

  // A cv::Mat header over the caller's bytes: imdecode reads through it and
  // copies nothing on the way in. const_cast is safe here because imdecode
  // only reads the source, and cv::Mat has no const-data constructor.
  const cv::Mat wrapped(
    1, static_cast<int>(compressed.size()), CV_8UC1,
    const_cast<uint8_t *>(compressed.data()));

  // IMREAD_COLOR forces 3-channel BGR8 regardless of what the file claims, so
  // a greyscale or CMYK JPEG cannot silently change the published encoding out
  // from under a downstream stage that trusts `encoding == "bgr8"`.
  //
  // The three-argument form decodes INTO `bgr`, reusing its allocation when the
  // dimensions already match. OpenCV reallocates when they do not, so a camera
  // that changed resolution mid-stream is handled — it just costs one alloc.
  //
  // **Read the RETURN value, not `bgr`.** On failure imdecode leaves the
  // destination exactly as it found it — still holding the PREVIOUS frame —
  // so `!bgr.empty()` is true for a corrupt buffer and the node would go on to
  // republish a stale image with a fresh timestamp. Caught by
  // test_jpeg.cpp's ReportsFailureEvenWhenTheDestinationHoldsAnOlderFrame,
  // which is the whole reason this is a tested free function.
  const cv::Mat decoded = cv::imdecode(wrapped, cv::IMREAD_COLOR, &bgr);
  return !decoded.empty();
}

}  // namespace pimesh_perception
