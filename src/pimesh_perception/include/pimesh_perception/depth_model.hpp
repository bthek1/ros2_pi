#ifndef PIMESH_PERCEPTION__DEPTH_MODEL_HPP_
#define PIMESH_PERCEPTION__DEPTH_MODEL_HPP_

#include <cmath>
#include <cstddef>

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

namespace pimesh_perception
{

/// The arithmetic on either side of Depth Anything V2, with no ONNX Runtime in
/// sight.
///
/// **Separated for two reasons and the second is the structural one.** The first
/// is that every line here is wrong-in-silence material: a channel order swapped,
/// a normalisation applied in the wrong units, or a reciprocal taken before the
/// clamp all produce a depth map that renders as a plausible-looking room and is
/// numerically nonsense. Nothing downstream can tell. The second is that this
/// header has no GPU dependency at all, so it compiles and its tests run on
/// **both** machines — where the inference engine behind it exists only where
/// ONNX Runtime does. A suite that only runs on the dev box is one that stops
/// being run.

/// 518 = 37 x 14. The model is a ViT-S/14, so the input must be a multiple of the
/// patch size, and 518 is the resolution it was trained at. Feeding it something
/// else works — the exported graph takes dynamic spatial dims — and degrades
/// quietly, which is the worst kind of working.
constexpr int kModelSize = 518;

/// ImageNet statistics, in RGB order. Not tunables: they are baked into what the
/// encoder learned, and the ONNX export does **not** include them. Its input is
/// named `pixel_values`, the HuggingFace convention for "already normalised", so
/// feeding raw 0..1 pixels is accepted without complaint and produces output that
/// still looks like a depth map — darker where the room is darker.
constexpr float kImagenetMean[3] = {0.485F, 0.456F, 0.406F};
constexpr float kImagenetStd[3] = {0.229F, 0.224F, 0.225F};

/// Number of floats `preprocess` writes.
constexpr std::size_t kInputElements =
  3U * static_cast<std::size_t>(kModelSize) * static_cast<std::size_t>(kModelSize);

/// Turn one `bgr8` frame into the model's NCHW float input.
///
/// `dst` must have room for `kInputElements` floats. Four things happen here and
/// every one of them has an opposite that also "works":
///
///  - **Stretch, do not letterbox.** The frame is 16:9 and the model wants a
///    square, so something has to give. Stretching is what the predecessor did,
///    and matching it is what makes any comparison between the two mean
///    something; letterboxing would leave grey bars that the model cheerfully
///    assigns a distance to.
///  - **INTER_AREA on the way down.** Bilinear downsampling from 1280x720 to
///    518x518 point-samples roughly every other pixel and aliases fine texture
///    into noise that the encoder then has an opinion about.
///  - **BGR to RGB.** The one that no error message will ever mention.
///  - **Planar, not interleaved.** NCHW means all of R, then all of G, then all
///    of B. Writing interleaved pixels into an NCHW tensor hands the model a
///    third of each channel in the wrong plane, and the output is still a smooth,
///    confident-looking surface.
///
/// The per-channel normalisation is folded into one multiply-add. `(v/255 - m)/s`
/// is algebraically `v * (1/(255 s)) - m/s`, which is the same arithmetic with a
/// division per *channel* instead of per *pixel* — 805k divisions saved per frame,
/// on a budget where preprocessing has to disappear next to a 51 ms inference.
inline void preprocess(const cv::Mat & bgr, float * dst)
{
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(kModelSize, kModelSize), 0, 0, cv::INTER_AREA);

  float gain[3];
  float bias[3];
  for (int c = 0; c < 3; ++c) {
    gain[c] = 1.0F / (255.0F * kImagenetStd[c]);
    bias[c] = -kImagenetMean[c] / kImagenetStd[c];
  }

  const std::size_t plane = static_cast<std::size_t>(kModelSize) * kModelSize;
  for (int y = 0; y < kModelSize; ++y) {
    const auto * row = resized.ptr<cv::Vec3b>(y);
    const std::size_t base = static_cast<std::size_t>(y) * kModelSize;
    for (int x = 0; x < kModelSize; ++x) {
      const cv::Vec3b & px = row[x];
      // px is B,G,R; channel c of the tensor is R,G,B — hence the 2 - c.
      for (int c = 0; c < 3; ++c) {
        dst[c * plane + base + x] = static_cast<float>(px[2 - c]) * gain[c] + bias[c];
      }
    }
  }
}

/// Model output to metres, clamping **before** the reciprocal.
///
/// Depth Anything emits *relative inverse* depth: larger means nearer, and the
/// units are whatever the training loss left behind. So distance is
/// `scale / relative`, where `scale` is a constant that monocular depth cannot
/// supply — see the node on why that is honest rather than a placeholder. This
/// function's whole job is the two failure modes of that division.
///
/// **The clamp is in inverse space and that is not a stylistic choice.** The
/// model's output tends to zero on anything it reads as "background, no idea",
/// and `1/x` there is not a large number, it is `inf`. Clamping the *result*
/// afterwards is one branch too late: `std::min(inf, 6.0f)` is indeed 6.0, but
/// any arithmetic that touched the infinity first is already NaN, and a NaN
/// integrated into a TSDF poisons voxels that were fine. Clamping the divisor at
/// `scale / max_range` means the division can never produce anything above
/// `max_range` in the first place.
///
/// Non-finite input — which the model can emit for a wholly saturated frame — is
/// mapped to `max_range` rather than propagated, on the same principle: this
/// pipeline's convention is that far-away and don't-know are the same answer, and
/// neither of them is NaN.
inline void to_metres(
  const cv::Mat & relative, float scale, float max_range, cv::Mat & metres)
{
  metres.create(relative.rows, relative.cols, CV_32FC1);
  const float floor_inverse = scale / max_range;

  for (int y = 0; y < relative.rows; ++y) {
    const float * in = relative.ptr<float>(y);
    float * out = metres.ptr<float>(y);
    for (int x = 0; x < relative.cols; ++x) {
      const float r = in[x];
      // The `!(r > floor)` spelling rather than `r <= floor` is deliberate: it
      // catches NaN too, because every comparison against NaN is false.
      out[x] = (!std::isfinite(r) || !(r > floor_inverse)) ? max_range : scale / r;
    }
  }
}

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__DEPTH_MODEL_HPP_
