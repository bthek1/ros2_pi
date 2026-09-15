// The ONNX Runtime implementation of DepthEngine.
//
// Compiled **only** where CMake found ONNX Runtime — see depth_engine.hpp for why
// the node above it builds everywhere and only this file is conditional, and
// CMakeLists.txt for the linker flag this whole arrangement depends on.
//
// **That linker flag is the single most important thing about this file, and it
// is not in this file.** `libonnxruntime_providers_cuda.so` is dlopened by
// `libonnxruntime.so` and carries no RPATH or RUNPATH of its own, and DT_RUNPATH
// — which is what CMake and every modern linker emit by default — is not
// inherited down a dlopen chain, while the older DT_RPATH is. Measured
// 2026-09-15, same source and same libraries, one flag apart: RUNPATH gives
// CPUExecutionProvider at 236.62 ms and RPATH gives CUDAExecutionProvider at
// 51.20 ms, with no error message in either case. `tools/gates/gpu-stack.sh`
// keeps a control run that asserts the default-flags build does *not* reach CUDA,
// so the flag cannot quietly stop being load-bearing.

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "pimesh_perception/depth_engine.hpp"
#include "pimesh_perception/depth_model.hpp"

namespace pimesh_perception
{

namespace
{

class OrtDepthEngine : public DepthEngine
{
public:
  OrtDepthEngine(const std::string & model_path, bool want_cuda)
  : env_(ORT_LOGGING_LEVEL_ERROR, "pimesh_depth")
  {
    Ort::SessionOptions options;
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    if (want_cuda) {
      try {
        OrtCUDAProviderOptionsV2 * cuda = nullptr;
        Ort::ThrowOnError(Ort::GetApi().CreateCUDAProviderOptions(&cuda));
        // Release through a guard: AppendExecutionProvider_CUDA_V2 can throw, and
        // leaking the options object on that path would be a leak on exactly the
        // route we expect to be taken when something is wrong.
        std::unique_ptr<OrtCUDAProviderOptionsV2, void (*)(OrtCUDAProviderOptionsV2 *)>
        guard(cuda, [](OrtCUDAProviderOptionsV2 * p) {Ort::GetApi().ReleaseCUDAProviderOptions(p);});
        options.AppendExecutionProvider_CUDA_V2(*cuda);
        provider_ = "CUDAExecutionProvider";
      } catch (const Ort::Exception & e) {
        diagnostic_ = std::string("CUDA provider refused: ") + e.what();
      }
    } else {
      diagnostic_ = "CUDA not requested (use_cuda:=false) — this is the gate's control run";
    }

    try {
      session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), options);
    } catch (const Ort::Exception & e) {
      if (provider_ != "CUDAExecutionProvider") {throw;}
      // The CUDA provider was accepted and then failed to initialise: a broken
      // cuDNN, a card with no free memory, a driver mismatch. Fall back rather
      // than dying, and *say so* — a node that comes up on the CPU having
      // announced it is a node somebody can diagnose. A node that exits leaves
      // the pipeline with no depth at all, which is worse for a stage everything
      // downstream is waiting on.
      diagnostic_ = std::string("CUDA session failed, fell back to CPU: ") + e.what();
      provider_ = "CPUExecutionProvider";
      Ort::SessionOptions cpu_only;
      cpu_only.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
      session_ = std::make_unique<Ort::Session>(env_, model_path.c_str(), cpu_only);
    }

    Ort::AllocatorWithDefaultOptions allocator;
    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
    input_names_[0] = input_name_.c_str();
    output_names_[0] = output_name_.c_str();

    memory_ = std::make_unique<Ort::MemoryInfo>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
  }

  bool infer(const float * input, float * output) override
  {
    try {
      // const_cast because CreateTensor wants a mutable pointer for a buffer it
      // only reads. The alternative is copying 3 MB per frame to satisfy a
      // signature.
      auto tensor = Ort::Value::CreateTensor<float>(
        *memory_, const_cast<float *>(input), kInputElements,
        input_shape_.data(), input_shape_.size());

      auto results = session_->Run(
        Ort::RunOptions{nullptr}, input_names_.data(), &tensor, 1,
        output_names_.data(), 1);

      // Check the shape rather than trusting it. A model swapped for one with a
      // different output size would otherwise be a silent buffer overrun here,
      // and the file is fetched from the network by a script.
      const auto shape = results[0].GetTensorTypeAndShapeInfo().GetShape();
      int64_t elements = 1;
      for (const auto d : shape) {elements *= d;}
      const int64_t want = static_cast<int64_t>(kModelSize) * kModelSize;
      if (elements != want) {
        diagnostic_ = "model returned " + std::to_string(elements) +
          " values, expected " + std::to_string(want);
        return false;
      }

      std::copy_n(results[0].GetTensorData<float>(), want, output);
      return true;
    } catch (const Ort::Exception & e) {
      diagnostic_ = std::string("inference failed: ") + e.what();
      return false;
    }
  }

  const std::string & provider() const override {return provider_;}
  const std::string & diagnostic() const override {return diagnostic_;}

private:
  Ort::Env env_;
  std::unique_ptr<Ort::Session> session_;
  std::unique_ptr<Ort::MemoryInfo> memory_;

  std::string provider_ {"CPUExecutionProvider"};
  std::string diagnostic_;

  std::string input_name_;
  std::string output_name_;
  std::array<const char *, 1> input_names_ {};
  std::array<const char *, 1> output_names_ {};

  // NCHW, batch 1. The exported graph takes dynamic spatial dims, so nothing
  // stops a wrong size here from running — see kModelSize in depth_model.hpp.
  std::array<int64_t, 4> input_shape_ {1, 3, kModelSize, kModelSize};
};

}  // namespace

std::unique_ptr<DepthEngine> make_depth_engine(
  const std::string & model_path, bool want_cuda, std::string & error)
{
  try {
    return std::make_unique<OrtDepthEngine>(model_path, want_cuda);
  } catch (const Ort::Exception & e) {
    error = std::string("could not load ") + model_path + ": " + e.what();
  } catch (const std::exception & e) {
    error = std::string("could not load ") + model_path + ": " + e.what();
  }
  return nullptr;
}

}  // namespace pimesh_perception
