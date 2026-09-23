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

#include <dlfcn.h>
#include <glob.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "pimesh_depth/depth_engine.hpp"
#include "pimesh_depth/depth_model.hpp"

namespace pimesh_depth
{

namespace
{

/// Load the CUDA runtime libraries by absolute path, then the CUDA provider,
/// before ONNX Runtime tries to.
///
/// **This is the second half of the RPATH story, and the gate that proved the
/// first half could not see it.** `tools/gates/gpu-stack.sh` establishes that
/// `-Wl,--disable-new-dtags` is what lets the CUDA provider find `libcublasLt` and
/// friends — and it establishes that for an *executable*, because `tools/gpu_probe`
/// is one. A component is not. Measured 2026-09-15, same libraries, same flags,
/// the two cases one container apart: `gpu_probe` reached `CUDAExecutionProvider`
/// at 51 ms/frame, and `depth_node` inside `component_container_isolated` reported
/// "Failed to load library …libonnxruntime_providers_cuda.so with error:
/// libcublasLt.so.13: cannot open shared object file", then ran on the CPU at
/// 517 ms/frame with the pipeline around it working perfectly.
///
/// **The mechanism, which is not the usual RUNPATH story.** Resolving an object's
/// `DT_NEEDED` entries, glibc searches: the object's own `DT_RPATH`, then the
/// `DT_RPATH` of the objects in its **loader chain**, then `LD_LIBRARY_PATH`, then
/// its own `DT_RUNPATH`, then the cache. Checked here with `readelf -d`:
/// `libonnxruntime_providers_cuda.so` has **neither** RPATH nor RUNPATH;
/// `libonnxruntime.so`, which dlopens it, has `DT_RUNPATH=$ORIGIN`, and a RUNPATH
/// never applies to anything but its own object's dependencies. And a dlopened
/// object has **no loader chain at all** — glibc sets `l_loader` for `DT_NEEDED`
/// dependencies, not for a dlopen — so the only RPATH left that could apply is the
/// **main executable's**. `gpu_probe` is an executable we link, so it has ours;
/// `component_container_isolated` was built by somebody else and has none. The
/// linker flag had not stopped mattering, it had stopped *reaching*.
///
/// That also rules out the obvious fix. Dlopening the provider ourselves does not
/// help — measured, and it was the first thing tried: this library's own
/// `DT_RPATH` is searched to find the *provider*, which was never the part that
/// failed, and not to resolve the provider's dependencies. `LD_LIBRARY_PATH` is
/// read once at process start, and this node has to work in a container launched
/// by `ros2 component load` as well as by our own launch file.
///
/// So: load the dependencies **themselves**, by absolute path, first. Each is then
/// in the process by soname, and the provider's `DT_NEEDED` entries are satisfied
/// from what is already loaded without any search happening at all. Each of these
/// libraries does carry `DT_RUNPATH=$ORIGIN`, which is enough for its *own*
/// dependencies once it has been found, so naming the directory once here is the
/// whole of it.
///
/// A glob rather than a list of sonames, because a list would be a second place
/// that knows which CUDA version `tools/fetch-gpu-stack.sh` pinned, and the two
/// would drift the first time it moved. The prefix holds only the redistributable
/// runtime libraries — `fetch-gpu-stack.sh` keeps the link-time stubs out of it on
/// purpose, and asserts a size floor on `libcublas.so.13` because a 22 kB stub
/// that resolves every symbol and segfaults on first use once got in this way.
///
/// Returns an empty string on success, or the `dlerror()` text. **The caller only
/// reports that text if appending the CUDA provider then fails** — dlopening the
/// provider by itself can fail harmlessly, because it imports symbols ONNX Runtime
/// supplies to it after loading it, and a run that reaches CUDA must not print an
/// error saying it did not. When the append really does fail, this text is the
/// only thing that names the missing library and its path; ONNX Runtime's own
/// message for the same failure ("Failed to load shared library") names neither.
std::string preload_cuda_provider()
{
#ifdef PIMESH_GPU_PREFIX
  const std::string lib_dir = std::string(PIMESH_GPU_PREFIX) + "/lib";

  // RTLD_LAZY here and RTLD_NOW below: these are pulled in for their *presence*,
  // and cuDNN's dispatch libraries legitimately carry symbols that resolve only
  // once an engine library is loaded beside them. Failures are ignored on purpose
  // — the list is a glob, so it is allowed to name something this ONNX Runtime
  // build does not need — and the one dlopen whose failure is reported is the
  // provider's, which is the only one that decides anything.
  glob_t found {};
  if (glob((lib_dir + "/libcu*.so.*").c_str(), 0, nullptr, &found) == 0) {
    for (std::size_t i = 0; i < found.gl_pathc; ++i) {
      dlopen(found.gl_pathv[i], RTLD_LAZY | RTLD_GLOBAL);
    }
  }
  globfree(&found);

  // The provider itself, last, so that a failure here is about the provider and
  // not about something it needs. RTLD_GLOBAL so its symbols are visible to
  // anything ONNX Runtime loads after it, and the handle is deliberately never
  // closed: ONNX Runtime holds its own reference for the life of the session, and
  // unloading a CUDA library underneath a live context is not a recoverable state.
  // glibc keys loaded objects by device and inode, so when ONNX Runtime dlopens
  // this same absolute path a moment later it receives the handle already open
  // rather than loading a second copy.
  //
  // **RTLD_LAZY, and RTLD_NOW is measured wrong here.** The provider is half of a
  // bridge: it imports `Provider_GetHost` and other symbols that ONNX Runtime
  // supplies to it *after* loading it, so resolving everything up front fails on a
  // provider that is completely fine. Measured 2026-09-15 — with RTLD_NOW this
  // returned "undefined symbol: Provider_GetHost" while the session went on to
  // reach CUDAExecutionProvider at 54.8 ms/frame, which is a diagnostic line
  // saying the opposite of what happened. Lazy binding still fails loudly for the
  // two things this check is actually for: the file being absent, and a dependency
  // of it being unfindable.
  const std::string provider = lib_dir + "/libonnxruntime_providers_cuda.so";
  if (dlopen(provider.c_str(), RTLD_LAZY | RTLD_GLOBAL) != nullptr) {return {};}
  const char * error = dlerror();
  return error != nullptr ? std::string(error) : ("dlopen failed: " + provider);
#else
  return "built without PIMESH_GPU_PREFIX";
#endif
}

class OrtDepthEngine : public DepthEngine
{
public:
  OrtDepthEngine(const std::string & model_path, bool want_cuda)
  : env_(ORT_LOGGING_LEVEL_ERROR, "pimesh_depth")
  {
    Ort::SessionOptions options;
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    if (want_cuda) {
      // Before anything else: see preload_cuda_provider() above. Without this the
      // append below throws inside a component container and succeeds in a
      // standalone executable, which is the most confusing pair of outcomes this
      // stage can produce.
      //
      // **Its failure is not reported unless the append below also fails**, and
      // that is the correction to a log line this node printed for half a day.
      // The step that actually fixes the container case is loading the CUDA
      // *libraries*; dlopening the provider on top of that is a cheap early
      // check, and it can fail for a reason that means nothing — the provider is
      // half of a bridge, importing `Provider_GetHost` and friends that ONNX
      // Runtime supplies to it after loading it, so resolving it on its own
      // reports `undefined symbol: Provider_GetHost` on a provider that is
      // completely fine. RTLD_LAZY does not help: the symbol is reached through a
      // data relocation, which is bound eagerly whatever the mode. So a run that
      // went on to reach CUDA at 55 ms/frame was logging "could not load the CUDA
      // provider" beside it — a diagnostic saying the opposite of what happened,
      // which is worse than no diagnostic at all.
      //
      // It is kept, not discarded, because when the append *does* fail this text
      // is the only thing that names the missing library and its path; ONNX
      // Runtime's own message for the same failure names neither.
      const std::string preload_error = preload_cuda_provider();
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
        // Whatever the preload said, the provider is in the session. Nothing to
        // report.
        diagnostic_.clear();
      } catch (const Ort::Exception & e) {
        // Now the preload error earns its place: ONNX Runtime's message for this
        // failure names no library and no path, and the dlerror text names both.
        diagnostic_ = std::string("CUDA provider refused: ") + e.what() +
          (preload_error.empty() ? "" : " (" + preload_error + ")");
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

}  // namespace pimesh_depth
