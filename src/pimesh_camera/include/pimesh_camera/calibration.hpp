#ifndef PIMESH_CAMERA__CALIBRATION_HPP_
#define PIMESH_CAMERA__CALIBRATION_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace pimesh_camera
{

/// A calibration read off disk, or the reason it was not.
///
/// `ok` is the only thing a caller may branch on. The rest is meaningful when it
/// is true and is left at its default when it is false, so a caller that ignores
/// `ok` gets zeros rather than stale numbers — an fx of 0 fails visibly where a
/// silently retained nominal one does not.
struct Calibration
{
  bool ok {false};
  /// Why not, in a form fit to put in a log line. Empty when `ok`.
  std::string why;
  /// The path the URL resolved to. Set even on failure — "no such file" is
  /// useless without it, and this is the field that catches a wrong
  /// `package://`.
  std::string path;
  std::string camera_name;
  std::uint32_t width {0};
  std::uint32_t height {0};
  std::string distortion_model;
  /// Row-major 3x3.
  std::vector<double> k;
  /// plumb_bob k1 k2 p1 p2 k3.
  std::vector<double> d;
};

/// Split a `package://<pkg>/<relative path>` URL.
///
/// Pure, so that the parsing is testable without the ament index: resolving the
/// package to a directory needs a package that is actually installed, and a unit
/// test that depends on its own workspace being installed is a test that fails
/// for reasons unrelated to what it checks.
///
/// \return true if `url` had the `package://` scheme, with `package` and
///         `relative` filled in. False leaves both untouched.
bool split_package_url(const std::string & url, std::string & package, std::string & relative);

/// Turn a camera-info URL into a filesystem path.
///
/// Accepts `package://<pkg>/<path>`, `file://<path>` and a bare path. An
/// unresolvable package yields an empty string; every other form is returned
/// whether or not anything is there, because "does this file exist" is
/// `load_calibration`'s question and it gives a better error than this can.
std::string resolve_camera_info_url(const std::string & url);

/// Read a standard `camera_info` YAML — the format `cameracalibrator` writes.
///
/// Validated rather than trusted, because every field here is one that is wrong
/// silently. The refusals, in the order they are checked:
///
///  - the file is missing or unreadable
///  - it is not YAML, or not a mapping
///  - `camera_matrix` is not 9 numbers, or `distortion_coefficients` not 5
///  - `distortion_model` is not `plumb_bob`
///  - **`image_width`/`image_height` disagree with the stream being captured**
///
/// The last is the dangerous one and the reason this function takes the expected
/// size at all. A 1280x720 calibration applied to a 1920x1080 stream is wrong by
/// a constant factor in fx, fy, cx and cy at once; nothing about the resulting
/// `CameraInfo` looks malformed, every consumer accepts it, and the error
/// surfaces as a reconstruction that is uniformly the wrong scale. Passing 0 for
/// either expected dimension skips that one check, which is what a tool that
/// only wants to read the file back does.
Calibration load_calibration(
  const std::string & url, std::uint32_t expect_width, std::uint32_t expect_height);

/// Does this calibration carry lens distortion, or is it a placeholder?
///
/// A real checkerboard run on a consumer webcam always produces non-zero
/// coefficients — a C922 has visible barrel distortion at the frame edges. A
/// loaded file whose D is all zeros is therefore either hand-written or a copy
/// of the nominal values, and must not be allowed to switch off the warning that
/// says so. This is what keeps `calibrated` from being a claim anybody can make
/// by editing a file.
bool has_distortion(const Calibration & calibration);

}  // namespace pimesh_camera

#endif  // PIMESH_CAMERA__CALIBRATION_HPP_
