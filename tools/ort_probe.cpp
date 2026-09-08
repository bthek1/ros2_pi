// Does ONNX Runtime actually run this model on the GPU on this machine?
//
// P4's plan says: do the toolchain work first and STANDALONE, and write no ROS
// code until this prints `CUDAExecutionProvider`. That order is not fussiness.
// The failure this guards against is silent: ONNX Runtime will happily create a
// session with no CUDA provider at all, run everything on the CPU, and report
// it as a warning nobody reads. On this GPU that is the difference between
// ~75 ms and ~290 ms a frame — a pipeline that works and one that does not —
// and it looks identical from the outside.
//
// So this program takes the CUDA provider's presence as a HARD requirement and
// exits non-zero without it, rather than falling back and mentioning it.
//
// Built by `just gpu-probe`, with g++ directly. No colcon, no ROS, no CMake:
// the point is to remove everything that could be blamed instead.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace
{

// Depth Anything V2 Small is a ViT-S/14: the transformer works on 14x14
// patches, so the input side must be a multiple of 14. 518 = 37 x 14 is the
// size it was trained at, and the one the predecessor measured 72-79 ms on.
constexpr int64_t kSide = 518;

double percentile(std::vector<double> v, double f)
{
  if (v.empty()) {
    return 0.0;
  }
  const auto i = std::min(v.size() - 1, static_cast<size_t>(f * v.size()));
  std::nth_element(v.begin(), v.begin() + i, v.end());
  return v[i];
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: ort_probe <model.onnx> [runs]\n");
    return 2;
  }
  const std::string model_path = argv[1];
  const int runs = argc > 2 ? std::atoi(argv[2]) : 20;

  std::printf("onnxruntime %s\n", Ort::GetVersionString().c_str());
  std::printf("providers compiled in:");
  for (const auto & p : Ort::GetAvailableProviders()) {
    std::printf(" %s", p.c_str());
  }
  std::printf("\n");

  Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "pimesh_probe");
  Ort::SessionOptions options;
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

  // The whole point of the probe. If the CUDA runtime is missing or the wrong
  // major version, this THROWS — it does not quietly return a CPU session.
  bool cuda = false;
  try {
    OrtCUDAProviderOptions cuda_options{};
    cuda_options.device_id = 0;
    options.AppendExecutionProvider_CUDA(cuda_options);
    cuda = true;
    std::printf("appended CUDAExecutionProvider (device 0)\n");
  } catch (const Ort::Exception & e) {
    std::printf("FAILED to append CUDAExecutionProvider: %s\n", e.what());
  }

  Ort::Session session(env, model_path.c_str(), options);
  Ort::AllocatorWithDefaultOptions allocator;

  // Ask the model what it wants rather than hardcoding names that are a
  // property of whoever exported it.
  const auto in_name = session.GetInputNameAllocated(0, allocator);
  const auto out_name = session.GetOutputNameAllocated(0, allocator);
  const auto in_shape =
    session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  std::printf("input  '%s' [", in_name.get());
  for (auto d : in_shape) {
    std::printf(" %ld", static_cast<long>(d));
  }
  std::printf(" ]  (-1 is a dynamic axis)\noutput '%s'\n", out_name.get());

  // A mid-grey frame. The content does not matter — the cost of a ViT forward
  // pass is fixed by its shape, not by what is in the pixels.
  std::vector<float> input(1 * 3 * kSide * kSide, 0.5f);
  const std::array<int64_t, 4> shape{1, 3, kSide, kSide};
  auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  auto tensor = Ort::Value::CreateTensor<float>(
    memory, input.data(), input.size(), shape.data(), shape.size());

  const char * in_names[] = {in_name.get()};
  const char * out_names[] = {out_name.get()};

  // Warm up, and do not count it. The first Run pays for CUDA context
  // creation, kernel autotuning and workspace allocation — hundreds of
  // milliseconds that say nothing about steady-state cost. `depth_node` warms
  // its session at startup for the same reason.
  auto warm = session.Run(
    Ort::RunOptions{nullptr}, in_names, &tensor, 1, out_names, 1);
  const auto out_shape =
    warm.front().GetTensorTypeAndShapeInfo().GetShape();
  std::printf("first run produced [");
  for (auto d : out_shape) {
    std::printf(" %ld", static_cast<long>(d));
  }
  std::printf(" ]\n");

  std::vector<double> ms;
  ms.reserve(static_cast<size_t>(runs));
  for (int i = 0; i < runs; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    auto result = session.Run(
      Ort::RunOptions{nullptr}, in_names, &tensor, 1, out_names, 1);
    const auto t1 = std::chrono::steady_clock::now();
    ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
  }

  const double mean = std::accumulate(ms.begin(), ms.end(), 0.0) / ms.size();
  std::printf(
    "\n%d runs at %ldx%ld: mean %.1f ms, p95 %.1f ms, min %.1f ms\n",
    runs, static_cast<long>(kSide), static_cast<long>(kSide),
    mean, percentile(ms, 0.95), *std::min_element(ms.begin(), ms.end()));

  if (!cuda) {
    std::printf("\nPROBE FAIL — no CUDA provider; this is the CPU path\n");
    return 1;
  }
  // The provider being *appended* is necessary but not sufficient: ONNX
  // Runtime falls back to CPU per-node for anything CUDA cannot run, and a
  // session that fell back wholesale still reports the provider as present.
  // The timing is what actually distinguishes them — the predecessor measured
  // 72-79 ms on the GPU against 280-305 ms on the CPU for this exact model, so
  // anything near the upper figure is a fallback wearing a green tick.
  if (mean > 150.0) {
    std::printf(
      "\nPROBE FAIL — %.1f ms is CPU-shaped; the GPU measured 72-79 ms here\n",
      mean);
    return 1;
  }
  std::printf("\nPROBE PASS — CUDAExecutionProvider, %.1f ms/frame\n", mean);
  return 0;
}
