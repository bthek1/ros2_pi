// The two pure steps either side of the model: what goes in, and what the
// numbers coming out actually mean.
//
// These were extracted out of `DepthModel` so they could be tested at all.
// Inside that class they sit either side of an `Ort::Session`, which needs a
// 99 MB model file and a GPU to exist; as free functions over `cv::Mat` they
// can be handed a value whose answer is known. That is the trade the house
// rule asks for — if logic is hard to test, that is a fact about the code.
//
// Deliberately in `perception_core` and NOT in `depth_core`: nothing here
// includes an ONNX Runtime header, so these tests build and run on a machine
// that has never installed it.

#ifndef PIMESH_PERCEPTION__DEPTH_CONVERT_HPP_
#define PIMESH_PERCEPTION__DEPTH_CONVERT_HPP_

#include <vector>

#include <opencv2/core.hpp>

namespace pimesh_perception
{

/// ImageNet's normalisation, which Depth Anything V2 inherits from DINOv2.
/// Getting these wrong does not crash — it makes the depth quietly worse,
/// which is the hardest kind of bug to see.
extern const float kImagenetMean[3];
extern const float kImagenetStd[3];

/// One BGR frame to the model's input tensor: resized to `side` square,
/// converted to RGB, scaled to [0,1], ImageNet-normalised, and laid out CHW.
///
/// Three conversions in one pass, and three separate silent-wrong-answer bugs
/// if any is skipped: BGR instead of RGB swaps the red and blue statistics,
/// HWC instead of CHW feeds the transformer interleaved noise, and skipping
/// the normalisation shifts every activation. None of them throws.
///
/// `out` is resized to 3*side*side.
void preprocess_frame(const cv::Mat & bgr, int side, std::vector<float> & out);

/// The model's relative INVERSE depth to metres, bounded at `max_depth_m`.
///
/// Depth Anything emits a number that is larger for nearer things, on a scale
/// it invents per frame: distance is `depth_scale / relative`. **`depth_scale`
/// is arbitrary until somebody holds a tape measure against a wall.**
///
/// The bound is applied to the INVERSE depth, by flooring the denominator,
/// rather than by clipping the result afterwards. Dividing first would run 1/x
/// on values near zero — the ones the model means as "background, no idea" —
/// and produce infinities to clean up after. Flooring bounds the output by
/// construction and never divides by anything small.
void relative_to_metres(
  const cv::Mat & relative, double depth_scale, double max_depth_m, cv::Mat & metres);

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__DEPTH_CONVERT_HPP_
