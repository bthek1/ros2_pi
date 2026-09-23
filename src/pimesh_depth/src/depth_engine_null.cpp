// The DepthEngine that exists so the Pi can build depth_node.
//
// Compiled **only** where CMake did not find ONNX Runtime. It is the reason the
// component registers, the launch file's plugin string resolves, and
// `pimesh_bringup`'s test_transforms passes identically on both machines — see
// depth_engine.hpp for the long version.
//
// It refuses rather than pretending. There is no "return zeros and carry on"
// mode, because a depth map full of plausible numbers is exactly the failure this
// project keeps finding in other guises: the pipeline would run, the mesh would
// build, and the first honest signal would be a room-shaped nothing several
// stages downstream.

#include <memory>
#include <string>

#include "pimesh_depth/depth_engine.hpp"

namespace pimesh_depth
{

std::unique_ptr<DepthEngine> make_depth_engine(
  const std::string & model_path, bool want_cuda, std::string & error)
{
  (void)model_path;
  (void)want_cuda;
  error =
    "this workspace was built without ONNX Runtime, so depth_node cannot infer. "
    "On the dev box: bash tools/fetch-gpu-stack.sh, then rebuild. "
    "On the Pi this is expected and correct — the Pi is a sensor head and runs "
    "pimesh_camera only; depth_node builds there so that one launch file and one "
    "set of unit tests are valid on both machines, not so that it can run.";
  return nullptr;
}

}  // namespace pimesh_depth
