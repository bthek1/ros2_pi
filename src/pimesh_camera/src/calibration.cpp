#include "pimesh_camera/calibration.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "yaml-cpp/yaml.h"

namespace pimesh_camera
{
namespace
{

constexpr char kPackageScheme[] = "package://";
constexpr char kFileScheme[] = "file://";

/// Read a `{rows, cols, data}` block and check that it says what it contains.
///
/// The rows*cols == data.size() check is not pedantry. This is the shape every
/// matrix in a camera_info YAML has, the count is written down twice, and a
/// hand-edited file where somebody deleted a number from `data` and left `cols`
/// at 3 is a file that parses.
bool read_matrix(
  const YAML::Node & node, const char * name, std::size_t expected,
  std::vector<double> & out, std::string & why)
{
  const YAML::Node block = node[name];
  if (!block || !block.IsMap()) {
    why = std::string("no '") + name + "' mapping";
    return false;
  }
  const YAML::Node data = block["data"];
  if (!data || !data.IsSequence()) {
    why = std::string("'") + name + "' has no 'data' sequence";
    return false;
  }

  out.clear();
  for (const auto & value : data) {
    out.push_back(value.as<double>());
  }

  if (out.size() != expected) {
    std::ostringstream os;
    os << "'" << name << "' has " << out.size() << " values, expected " << expected;
    why = os.str();
    return false;
  }

  // Declared shape against actual contents, when the file bothers to declare it.
  if (block["rows"] && block["cols"]) {
    const auto rows = block["rows"].as<std::size_t>();
    const auto cols = block["cols"].as<std::size_t>();
    if (rows * cols != out.size()) {
      std::ostringstream os;
      os << "'" << name << "' says " << rows << "x" << cols << " but carries " << out.size()
         << " values";
      why = os.str();
      return false;
    }
  }
  return true;
}

}  // namespace

bool split_package_url(const std::string & url, std::string & package, std::string & relative)
{
  const std::string scheme(kPackageScheme);
  if (url.compare(0, scheme.size(), scheme) != 0) {return false;}

  const std::string rest = url.substr(scheme.size());
  const std::size_t slash = rest.find('/');
  // A bare `package://pimesh_bringup` names no file, and a leading slash names
  // no package. Both are the caller's typo, and neither may be guessed at.
  if (slash == std::string::npos || slash == 0 || slash + 1 >= rest.size()) {return false;}

  package = rest.substr(0, slash);
  relative = rest.substr(slash + 1);
  return true;
}

std::string resolve_camera_info_url(const std::string & url)
{
  std::string package, relative;
  if (split_package_url(url, package, relative)) {
    try {
      // get_package_share_path, not get_package_share_directory: the latter is
      // deprecated in Lyrical (a -Wdeprecated-declarations warning here) and not
      // in Jazzy, so it is the spelling that is only half-right. This one exists
      // and is current on both (checked 2026-09-12).
      return (ament_index_cpp::get_package_share_path(package) / relative).string();
    } catch (const std::exception &) {
      // The package is not installed. Empty rather than a guessed path, so the
      // caller's message can say "no such package" instead of inventing a
      // plausible-looking filename that was never going to be there.
      return {};
    }
  }

  const std::string file(kFileScheme);
  if (url.compare(0, file.size(), file) == 0) {return url.substr(file.size());}

  return url;
}

Calibration load_calibration(
  const std::string & url, std::uint32_t expect_width, std::uint32_t expect_height)
{
  Calibration cal;

  if (url.empty()) {
    cal.why = "no camera_info_url set";
    return cal;
  }

  cal.path = resolve_camera_info_url(url);
  if (cal.path.empty()) {
    cal.why = "cannot resolve '" + url + "' — is that package installed?";
    return cal;
  }

  std::error_code ec;
  if (!std::filesystem::is_regular_file(cal.path, ec)) {
    cal.why = "no such file";
    return cal;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(cal.path);
  } catch (const std::exception & e) {
    cal.why = std::string("not parseable as YAML: ") + e.what();
    return cal;
  }
  if (!root.IsMap()) {
    cal.why = "not a YAML mapping";
    return cal;
  }

  try {
    if (!read_matrix(root, "camera_matrix", 9, cal.k, cal.why)) {return cal;}
    if (!read_matrix(root, "distortion_coefficients", 5, cal.d, cal.why)) {return cal;}

    cal.distortion_model =
      root["distortion_model"] ? root["distortion_model"].as<std::string>() : std::string();
    cal.camera_name =
      root["camera_name"] ? root["camera_name"].as<std::string>() : std::string();
    cal.width = root["image_width"] ? root["image_width"].as<std::uint32_t>() : 0;
    cal.height = root["image_height"] ? root["image_height"].as<std::uint32_t>() : 0;
  } catch (const std::exception & e) {
    // A non-numeric entry in `data`, or a string where a width should be. YAML
    // is permissive about this and `as<double>()` is not.
    cal.why = std::string("a field is the wrong type: ") + e.what();
    cal.k.clear();
    cal.d.clear();
    return cal;
  }

  // plumb_bob is what make_camera_info writes into the message and what five
  // coefficients mean. A `rational_polynomial` file carries eight, would have
  // been caught above, and a `fisheye` one carries four of a different model —
  // accepting either while publishing "plumb_bob" would be a lie in the message
  // rather than an error in the file.
  if (cal.distortion_model != "plumb_bob") {
    cal.why = "distortion_model is '" + cal.distortion_model + "', expected 'plumb_bob'";
    cal.k.clear();
    cal.d.clear();
    return cal;
  }

  if (expect_width != 0 && expect_height != 0 &&
    (cal.width != expect_width || cal.height != expect_height))
  {
    std::ostringstream os;
    os << "calibrated at " << cal.width << "x" << cal.height << " but capturing at "
       << expect_width << "x" << expect_height
       << " — intrinsics do not carry across resolutions";
    cal.why = os.str();
    cal.k.clear();
    cal.d.clear();
    return cal;
  }

  cal.ok = true;
  cal.why.clear();
  return cal;
}

bool has_distortion(const Calibration & calibration)
{
  if (!calibration.ok) {return false;}
  for (const double coefficient : calibration.d) {
    if (std::abs(coefficient) > 1e-9) {return true;}
  }
  return false;
}

}  // namespace pimesh_camera
