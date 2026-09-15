// The toolchain proof for P4, standalone and deliberately so.
//
// P4's plan says: do the toolchain work first, as a small C++ program that loads
// the model, runs a frame and prints the provider and the time; no ROS code until
// it prints CUDAExecutionProvider. This is that program, kept rather than thrown
// away, because the claim it makes is one the project has to be able to re-check
// after a driver update, an ONNX Runtime bump or a new machine — and because it
// is the only instrument here with no rclcpp, no container and no camera between
// it and the GPU.
//
// Compiled by tools/gates/gpu-stack.sh with g++ directly, not by colcon. That is
// on purpose: the thing being tested is whether a plain C++ link against the
// ONNX Runtime tarball reaches the GPU, and putting ament, an overlay and a
// component container in front of that would be testing four things and
// reporting one.
//
// **What "provider" means here, precisely.** ONNX Runtime's C++ API has no
// per-session "which execution provider did you actually use" query, so this
// reports the EP the session was successfully *built* with: the CUDA EP is
// appended, and if either appending it or creating the session throws, the
// session is rebuilt CPU-only and that is what gets reported. That is a real
// signal — the CUDA provider failing to load throws at exactly those two points
// (measured 2026-09-15) — but it is not proof that arithmetic happened on the
// GPU. The time is the other half of that, which is why this prints it and why
// gates/gpu-stack.sh asserts on both ends of a 4x separation rather than on the
// word alone.
//
//   gpu_probe <model.onnx> [--cpu] [--runs N]
//
// Output is `gpu key=value` lines, the same shape every other probe in this repo
// uses, so a gate reads it with awk rather than by parsing prose.

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <numeric>
#include <string>
#include <vector>

namespace
{

// 518 = 37 * 14: Depth Anything V2 Small is a ViT-S/14 and the input must be a
// multiple of the patch size. The pixels themselves are irrelevant to a timing
// measurement — a transformer's cost does not depend on what it is looking at —
// so this feeds a constant rather than pulling in OpenCV to decode a real frame.
constexpr int64_t kSize = 518;

}  // namespace

int main(int argc, char ** argv)
{
  const char * model = nullptr;
  bool want_cuda = true;
  int runs = 25;

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--cpu") == 0) {
      want_cuda = false;
    } else if (std::strcmp(argv[i], "--runs") == 0 && i + 1 < argc) {
      runs = std::atoi(argv[++i]);
    } else {
      model = argv[i];
    }
  }
  if (model == nullptr) {
    std::fprintf(stderr, "usage: gpu_probe <model.onnx> [--cpu] [--runs N]\n");
    return 2;
  }

  std::string available;
  for (const auto & p : Ort::GetAvailableProviders()) {
    available += (available.empty() ? "" : ",") + p;
  }

  Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "gpu_probe");

  // Build the session, with an explicit fallback that *logs which one it got*.
  // Silent fallback is the whole failure mode this phase is guarding against:
  // ONNX Runtime is perfectly happy to run this model on the CPU at 200 ms and
  // say nothing about it.
  std::string provider = "CPUExecutionProvider";
  std::string refusal;
  Ort::SessionOptions options;
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  if (want_cuda) {
    try {
      OrtCUDAProviderOptionsV2 * cuda = nullptr;
      Ort::ThrowOnError(Ort::GetApi().CreateCUDAProviderOptions(&cuda));
      options.AppendExecutionProvider_CUDA_V2(*cuda);
      Ort::GetApi().ReleaseCUDAProviderOptions(cuda);
      provider = "CUDAExecutionProvider";
    } catch (const Ort::Exception & e) {
      refusal = e.what();
    }
  }

  std::unique_ptr<Ort::Session> session;
  try {
    session = std::make_unique<Ort::Session>(env, model, options);
  } catch (const Ort::Exception & e) {
    if (provider != "CUDAExecutionProvider") {
      std::fprintf(stderr, "gpu_probe: session creation failed: %s\n", e.what());
      return 1;
    }
    // The CUDA provider was accepted and then could not initialise — a broken
    // cuDNN, a busy GPU, a driver mismatch. Fall back rather than dying, so the
    // gate gets a number and a provider name to fail on instead of a stack
    // trace it has to interpret.
    refusal = e.what();
    provider = "CPUExecutionProvider";
    Ort::SessionOptions cpu_only;
    cpu_only.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session = std::make_unique<Ort::Session>(env, model, cpu_only);
  }

  Ort::AllocatorWithDefaultOptions alloc;
  auto in_name = session->GetInputNameAllocated(0, alloc);
  auto out_name = session->GetOutputNameAllocated(0, alloc);
  const char * inputs[] = {in_name.get()};
  const char * outputs[] = {out_name.get()};

  std::vector<int64_t> shape{1, 3, kSize, kSize};
  std::vector<float> pixels(3 * kSize * kSize, 0.5F);
  auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  // Warm-up is not politeness. The first run pays for CUDA context creation,
  // cuBLAS handle setup and kernel autotuning — 300 ms against a 53 ms steady
  // state, measured — and averaging that in would make the number depend
  // entirely on how many runs were requested.
  const int warmup = std::min(5, std::max(1, runs / 5));
  std::vector<double> ms;
  ms.reserve(static_cast<std::size_t>(runs));
  double first_ms = 0.0;
  std::vector<int64_t> out_shape;

  for (int i = 0; i < runs + warmup; ++i) {
    auto tensor = Ort::Value::CreateTensor<float>(
      memory, pixels.data(), pixels.size(), shape.data(), shape.size());
    const auto t0 = std::chrono::steady_clock::now();
    auto result = session->Run(Ort::RunOptions{nullptr}, inputs, &tensor, 1, outputs, 1);
    const auto t1 = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (i == 0) {
      first_ms = elapsed;
      out_shape = result[0].GetTensorTypeAndShapeInfo().GetShape();
    }
    if (i >= warmup) {ms.push_back(elapsed);}
  }

  std::sort(ms.begin(), ms.end());
  const double mean = std::accumulate(ms.begin(), ms.end(), 0.0) / static_cast<double>(ms.size());
  const double p95 = ms[static_cast<std::size_t>(0.95 * static_cast<double>(ms.size() - 1))];

  std::string shape_text;
  for (auto d : out_shape) {shape_text += (shape_text.empty() ? "" : "x") + std::to_string(d);}

  std::printf("gpu provider=%s\n", provider.c_str());
  std::printf("gpu available=%s\n", available.c_str());
  std::printf("gpu input=%s\n", in_name.get());
  std::printf("gpu output=%s\n", out_name.get());
  std::printf("gpu output_shape=%s\n", shape_text.c_str());
  std::printf("gpu runs=%zu\n", ms.size());
  std::printf("gpu first_ms=%.2f\n", first_ms);
  std::printf("gpu mean_ms=%.2f\n", mean);
  std::printf("gpu p95_ms=%.2f\n", p95);
  std::printf("gpu min_ms=%.2f\n", ms.front());
  if (!refusal.empty()) {
    // One line, no newlines in it, so the gate can print it verbatim.
    std::replace(refusal.begin(), refusal.end(), '\n', ' ');
    std::printf("gpu refusal=%s\n", refusal.c_str());
  }
  return 0;
}
