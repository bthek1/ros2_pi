#ifndef PIMESH_PERCEPTION__DEPTH_ENGINE_HPP_
#define PIMESH_PERCEPTION__DEPTH_ENGINE_HPP_

#include <memory>
#include <string>

namespace pimesh_perception
{

/// One frame of inference, behind an interface, so that `depth_node` compiles on
/// a machine with no ONNX Runtime.
///
/// **This abstraction is here for a specific and checkable reason.** The Pi
/// builds this whole workspace — `tools/gates/build.sh` builds it at both ends,
/// because the one thing a cross-distro C++ project cannot afford is a package
/// that compiles on one machine only. But the GPU stack is dev-box-only: it is
/// x86-64, it is CUDA, and the Pi is a sensor head that will never run inference.
///
/// The obvious answer — build `depth_node` only where ONNX Runtime is found —
/// breaks something specific. `pimesh_bringup`'s `test_transforms.py` asserts
/// that **every** plugin string in the launch file's `COMPONENTS` is registered
/// in the ament index, which is what catches a typo in a name that is otherwise
/// only resolved at runtime. A component missing on the Pi fails that test there,
/// and `tools/gates/test.sh` asserts the same suites pass at both ends with zero
/// skips. Weakening the assertion to accommodate the Pi would blunt the check on
/// the machine where it actually matters.
///
/// So the *node* builds everywhere — same class, same registration, same
/// parameters, same topics — and only the engine behind it is conditional.
/// Exactly one implementation of `make_depth_engine` is compiled:
/// `depth_engine_ort.cpp` where ONNX Runtime was found, `depth_engine_null.cpp`
/// where it was not. The Pi therefore builds a `depth_node` that refuses to start,
/// with a message saying why, which is the correct behaviour for a node that
/// machine must never run.
class DepthEngine
{
public:
  virtual ~DepthEngine() = default;

  /// Run one frame. `input` is `kInputElements` floats NCHW; `output` receives
  /// `kModelSize * kModelSize` floats of relative inverse depth.
  ///
  /// Returns false on a failed run rather than throwing, because the caller is a
  /// worker thread whose job is to drop a bad frame and stay alive.
  virtual bool infer(const float * input, float * output) = 0;

  /// The execution provider the session was **built** with.
  ///
  /// ONNX Runtime's C++ API has no per-session "which provider actually ran this"
  /// query, so this reports what was successfully appended. That is a real signal
  /// — a CUDA provider that cannot load its libraries throws at exactly that
  /// point, measured 2026-09-15 — but it is not proof that arithmetic happened on
  /// the GPU. The other half of that proof is the time, which is why the node
  /// logs both and why `tools/gates/depth.sh` asserts on both.
  virtual const std::string & provider() const = 0;

  /// Non-empty when the engine fell back or refused, describing why. Logged at
  /// startup so a silent CPU fallback is one visible line rather than "the mesh
  /// got slow".
  virtual const std::string & diagnostic() const = 0;
};

/// Build an engine, or return null with `error` set.
///
/// `want_cuda` false forces the CPU provider — which is not a debugging
/// convenience but the control run `tools/gates/depth.sh` needs: an 80 ms budget
/// that the CPU path has never been shown to fail is a threshold nobody has
/// watched exclude anything.
std::unique_ptr<DepthEngine> make_depth_engine(
  const std::string & model_path, bool want_cuda, std::string & error);

}  // namespace pimesh_perception

#endif  // PIMESH_PERCEPTION__DEPTH_ENGINE_HPP_
