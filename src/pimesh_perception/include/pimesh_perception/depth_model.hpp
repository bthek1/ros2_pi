// The depth model, wrapped so it can be reasoned about without a node.
//
// Everything ONNX Runtime touches lives behind this class: session creation and
// the provider decision, the preprocessing a ViT expects, and the conversion
// from what the model actually emits to metres. The node above it moves
// messages and owns a thread; it does not know what ImageNet normalisation is.
//
// **The model does not measure distance.** Depth Anything V2 emits *relative
// inverse* depth: a number that is larger for nearer things, on a scale it
// invents per frame. Metres come out only after dividing a scale factor by it,
// and that scale factor is arbitrary until somebody holds a tape measure
// against a wall — which is P5's job, not this class's. Treat every metre
// figure this produces as provisional until then, and do not let a downstream
// stage quietly assume otherwise.

#ifndef PIMESH_PERCEPTION__DEPTH_MODEL_HPP_
#define PIMESH_PERCEPTION__DEPTH_MODEL_HPP_

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <onnxruntime_cxx_api.h>

namespace pimesh_perception
{

/// Which execution provider the session actually got. Reported rather than
/// assumed: ONNX Runtime falls back to CPU *silently* when the CUDA runtime is
/// missing or the wrong major version, and on this GPU that is the difference
/// between ~53 ms and ~290 ms a frame — visible as "the mesh got slow" and as
/// nothing else.
enum class Provider
{
  kCuda,
  kCpu,
};

const char * to_string(Provider provider);

class DepthModel
{
public:
  struct Options
  {
    std::string model_path;
    /// The transformer works on 14x14 patches, so the input side must be a
    /// multiple of 14. 518 = 37 x 14 is what the model was trained at.
    int input_side{518};
    /// Try CUDA first. When false the session is CPU-only — for measuring the
    /// fallback deliberately rather than discovering it.
    bool use_cuda{true};
    /// Metres. Turns the model's relative inverse depth into distance:
    /// `z = depth_scale / relative`. **Arbitrary until pinned by tape measure
    /// at P5** — the predecessor's room came out at 2.69.
    double depth_scale{10.0};
    /// Everything further than this reads as exactly this. Applied BEFORE the
    /// reciprocal, by flooring the inverse depth, so 1/x never explodes on the
    /// values the model means as "background, no idea".
    double max_depth_m{6.0};
  };

  explicit DepthModel(const Options & options);
  ~DepthModel();

  /// One BGR frame in, one 32FC1 depth map in metres out, at the frame's own
  /// resolution. `depth` is resized and reused across calls.
  void infer(const cv::Mat & bgr, cv::Mat & depth);

  Provider provider() const {return provider_;}
  const Options & options() const {return options_;}
  /// What the model calls its input and output. Queried, not hardcoded: those
  /// names are a property of whoever exported the ONNX file.
  const std::string & input_name() const {return input_name_;}
  const std::string & output_name() const {return output_name_;}

private:
  void preprocess(const cv::Mat & bgr);

  Options options_;
  Provider provider_{Provider::kCpu};

  // Ort::Env must outlive every session made from it, so it is a member and
  // not a local in the constructor.
  Ort::Env env_;
  std::unique_ptr<Ort::Session> session_;
  Ort::MemoryInfo memory_;

  std::string input_name_;
  std::string output_name_;

  // Reused across frames. A ViT forward pass is ~53 ms, so allocating 3 MB of
  // input tensor per frame would be noise against it — but reusing the buffer
  // also means the tensor handed to ONNX Runtime always has the same address
  // and lifetime, which is one fewer thing to reason about.
  std::vector<float> input_;
};

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__DEPTH_MODEL_HPP_
