#include "pimesh_perception/depth_model.hpp"

#include <algorithm>
#include <array>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

#include "pimesh_perception/depth_convert.hpp"

namespace pimesh_perception
{

const char * to_string(Provider provider)
{
  switch (provider) {
    case Provider::kCuda: return "CUDAExecutionProvider";
    case Provider::kCpu: return "CPUExecutionProvider";
  }
  return "unknown";
}

DepthModel::DepthModel(const Options & options)
: options_(options),
  env_(ORT_LOGGING_LEVEL_WARNING, "pimesh_depth"),
  memory_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
  if (options_.input_side % 14 != 0) {
    throw std::invalid_argument(
      "input_side must be a multiple of 14 — the ViT works on 14x14 patches");
  }

  Ort::SessionOptions session_options;
  session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  if (options_.use_cuda) {
    // AppendExecutionProvider_CUDA THROWS when the CUDA runtime is absent or
    // the wrong major version. Catching it here is what turns a silent CPU
    // session into a reported one — the caller logs which provider it got, and
    // `just gate-depth` fails on anything but CUDA.
    try {
      OrtCUDAProviderOptions cuda_options{};
      cuda_options.device_id = 0;
      session_options.AppendExecutionProvider_CUDA(cuda_options);
      provider_ = Provider::kCuda;
    } catch (const Ort::Exception &) {
      provider_ = Provider::kCpu;
    }
  }

  session_ = std::make_unique<Ort::Session>(
    env_, options_.model_path.c_str(), session_options);

  Ort::AllocatorWithDefaultOptions allocator;
  input_name_ = session_->GetInputNameAllocated(0, allocator).get();
  output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

  const auto side = static_cast<size_t>(options_.input_side);
  input_.assign(3 * side * side, 0.0f);

  // Warm the session: the first Run pays for CUDA context creation, kernel
  // autotuning and workspace allocation — hundreds of milliseconds that say
  // nothing about steady-state cost, and that would otherwise be paid on the
  // first real frame while the rest of the pipeline waits.
  cv::Mat warm(options_.input_side, options_.input_side, CV_8UC3, cv::Scalar(128, 128, 128));
  cv::Mat scratch;
  infer(warm, scratch);
}

DepthModel::~DepthModel() = default;

void DepthModel::preprocess(const cv::Mat & bgr)
{
  // The work itself lives in perception_core, with no ONNX Runtime header
  // anywhere near it, so `test_depth_convert` can drive it on a machine that
  // has never installed the runtime.
  preprocess_frame(bgr, options_.input_side, input_);
}

void DepthModel::infer(const cv::Mat & bgr, cv::Mat & depth)
{
  preprocess(bgr);

  const int side = options_.input_side;
  const std::array<int64_t, 4> shape{1, 3, side, side};
  auto tensor = Ort::Value::CreateTensor<float>(
    memory_, input_.data(), input_.size(), shape.data(), shape.size());

  const char * in_names[] = {input_name_.c_str()};
  const char * out_names[] = {output_name_.c_str()};
  auto outputs = session_->Run(
    Ort::RunOptions{nullptr}, in_names, &tensor, 1, out_names, 1);

  // The model emits [1, side, side] — no channel axis. A cv::Mat header over
  // ONNX Runtime's buffer, so nothing is copied to read it.
  const float * data = outputs.front().GetTensorData<float>();
  const cv::Mat relative(side, side, CV_32FC1, const_cast<float *>(data));

  // Relative INVERSE depth: larger means nearer, on a scale the model invents
  // per frame. The conversion, and the reason the bound is applied before the
  // division rather than after it, are in depth_convert.
  cv::Mat metres;
  relative_to_metres(relative, options_.depth_scale, options_.max_depth_m, metres);

  // Back to the frame's own resolution. Linear rather than area: this is an
  // UPscale, where area degenerates to nearest-neighbour and leaves blocky
  // 2.5x2.5 px steps that a TSDF would happily integrate as real geometry.
  cv::resize(metres, depth, bgr.size(), 0, 0, cv::INTER_LINEAR);
}

}  // namespace pimesh_perception
