// The calibration loader's refusals.
//
// Every case here is a file that parses as YAML and is wrong anyway, because
// that is the only interesting category. A syntax error announces itself at the
// first read; a 1280x720 calibration published over a 1920x1080 stream does not
// announce itself at all — it scales fx, fy, cx and cy by one constant, produces
// a `CameraInfo` no consumer objects to, and surfaces several milestones later
// as a reconstruction that is uniformly the wrong size.
//
// No camera and no ROS graph: the loader is a free function over a path for
// exactly this reason.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "gtest/gtest.h"
#include "pimesh_camera/calibration.hpp"

using pimesh_camera::Calibration;
using pimesh_camera::has_distortion;
using pimesh_camera::load_calibration;
using pimesh_camera::resolve_camera_info_url;
using pimesh_camera::split_package_url;

namespace
{

/// A well-formed 1280x720 calibration, as `cameracalibrator` writes it. The
/// numbers are plausible for a C922 but are not this project's calibration —
/// that one is measured, lives in pimesh_bringup, and is not what this tests.
const char kGoodYaml[] = R"(image_width: 1280
image_height: 720
camera_name: c922_720p
camera_matrix:
  rows: 3
  cols: 3
  data: [905.1, 0.0, 641.2, 0.0, 903.7, 359.4, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0812, -0.1623, 0.0009, -0.0004, 0.0451]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [905.1, 0.0, 641.2, 0.0, 0.0, 903.7, 359.4, 0.0, 0.0, 0.0, 1.0, 0.0]
)";

/// A temp file that removes itself, so a failing assertion cannot leave a stray
/// YAML behind for the next test to find.
class TempYaml
{
public:
  explicit TempYaml(const std::string & contents)
  : path_((std::filesystem::temp_directory_path() /
      ("pimesh_calib_" + std::to_string(::rand()) + ".yaml")).string())
  {
    std::ofstream out(path_);
    out << contents;
  }
  ~TempYaml() {std::filesystem::remove(path_);}
  const std::string & path() const {return path_;}

private:
  std::string path_;
};

}  // namespace

// --- The happy path, so that every refusal below is a refusal of something ---

TEST(Calibration, LoadsAWellFormedFile)
{
  TempYaml file(kGoodYaml);
  const Calibration cal = load_calibration(file.path(), 1280, 720);

  ASSERT_TRUE(cal.ok) << cal.why;
  EXPECT_EQ(cal.camera_name, "c922_720p");
  EXPECT_EQ(cal.width, 1280u);
  EXPECT_EQ(cal.height, 720u);
  EXPECT_EQ(cal.distortion_model, "plumb_bob");
  ASSERT_EQ(cal.k.size(), 9u);
  ASSERT_EQ(cal.d.size(), 5u);
  EXPECT_DOUBLE_EQ(cal.k[0], 905.1);
  EXPECT_DOUBLE_EQ(cal.k[2], 641.2);
  EXPECT_DOUBLE_EQ(cal.k[4], 903.7);
  EXPECT_DOUBLE_EQ(cal.k[5], 359.4);
  EXPECT_DOUBLE_EQ(cal.d[0], 0.0812);
  EXPECT_TRUE(has_distortion(cal));
}

// Passing 0 for either expected dimension skips the size check, which is what a
// tool reading the file back for its own purposes wants.
TEST(Calibration, ZeroExpectedSizeSkipsTheSizeCheck)
{
  TempYaml file(kGoodYaml);
  EXPECT_TRUE(load_calibration(file.path(), 0, 0).ok);
}

// --- The refusals ------------------------------------------------------------

// **The one this function exists for.** A right-looking calibration at the wrong
// resolution.
TEST(Calibration, RefusesACalibrationForADifferentResolution)
{
  TempYaml file(kGoodYaml);
  const Calibration cal = load_calibration(file.path(), 1920, 1080);

  EXPECT_FALSE(cal.ok);
  // The message has to carry both sizes, or it sends somebody looking at the
  // wrong file.
  EXPECT_NE(cal.why.find("1280x720"), std::string::npos) << cal.why;
  EXPECT_NE(cal.why.find("1920x1080"), std::string::npos) << cal.why;
  // Nothing usable is handed back on a refusal: a caller that ignores `ok` gets
  // zeros, which fail visibly, rather than intrinsics for the wrong stream.
  EXPECT_TRUE(cal.k.empty());
  EXPECT_TRUE(cal.d.empty());
}

TEST(Calibration, RefusesAMissingFile)
{
  const Calibration cal = load_calibration("/nonexistent/pimesh/nope.yaml", 1280, 720);

  EXPECT_FALSE(cal.ok);
  // "no such file" is the *one* failure camera_node treats as non-fatal, because
  // it is the normal state before anybody has run the board. The prefix is load
  // bearing, so it is pinned here.
  EXPECT_EQ(cal.why.rfind("no such file", 0), 0u) << cal.why;
  EXPECT_EQ(cal.path, "/nonexistent/pimesh/nope.yaml");
}

TEST(Calibration, RefusesAnEmptyUrl)
{
  const Calibration cal = load_calibration("", 1280, 720);
  EXPECT_FALSE(cal.ok);
  // Not "no such file": there is no file to be missing, and camera_node must not
  // confuse "I was asked for nothing" with "what I was asked for is gone".
  EXPECT_NE(cal.why.rfind("no such file", 0), 0u) << cal.why;
}

TEST(Calibration, RefusesSomethingThatIsNotYaml)
{
  TempYaml file("\tthis: [is not, valid\n  yaml at: all\n");
  const Calibration cal = load_calibration(file.path(), 1280, 720);
  EXPECT_FALSE(cal.ok);
}

TEST(Calibration, RefusesAScalarInsteadOfAMapping)
{
  TempYaml file("just-a-string\n");
  const Calibration cal = load_calibration(file.path(), 1280, 720);
  EXPECT_FALSE(cal.ok);
  EXPECT_NE(cal.why.find("mapping"), std::string::npos) << cal.why;
}

// Eight numbers where nine belong. The file still parses, and every consumer of
// the resulting K would read a shifted matrix.
TEST(Calibration, RefusesAShortCameraMatrix)
{
  std::string yaml(kGoodYaml);
  yaml.replace(
    yaml.find("data: [905.1"), std::string("data: [905.1, 0.0, 641.2, 0.0, 903.7, 359.4, 0.0, 0.0, 1.0]").size(),
    "data: [905.1, 0.0, 641.2, 0.0, 903.7, 359.4, 0.0, 1.0]");
  TempYaml file(yaml);

  const Calibration cal = load_calibration(file.path(), 1280, 720);
  EXPECT_FALSE(cal.ok);
  EXPECT_NE(cal.why.find("camera_matrix"), std::string::npos) << cal.why;
  EXPECT_NE(cal.why.find("8"), std::string::npos) << cal.why;
}

// `rows`/`cols` disagreeing with what `data` holds. This is what a hand-edit
// looks like: a coefficient deleted and the shape left behind.
TEST(Calibration, RefusesADeclaredShapeThatDisagreesWithTheData)
{
  std::string yaml(kGoodYaml);
  yaml.replace(yaml.find("cols: 5"), std::string("cols: 5").size(), "cols: 4");
  TempYaml file(yaml);

  const Calibration cal = load_calibration(file.path(), 1280, 720);
  EXPECT_FALSE(cal.ok);
  EXPECT_NE(cal.why.find("distortion_coefficients"), std::string::npos) << cal.why;
}

// A different distortion model with the right *number* of coefficients would be
// published as plumb_bob and silently misinterpreted, so the name is checked as
// well as the count.
TEST(Calibration, RefusesANonPlumbBobModel)
{
  std::string yaml(kGoodYaml);
  yaml.replace(yaml.find("plumb_bob"), std::string("plumb_bob").size(), "equidistant");
  TempYaml file(yaml);

  const Calibration cal = load_calibration(file.path(), 1280, 720);
  EXPECT_FALSE(cal.ok);
  EXPECT_NE(cal.why.find("equidistant"), std::string::npos) << cal.why;
}

TEST(Calibration, RefusesANonNumericCoefficient)
{
  std::string yaml(kGoodYaml);
  yaml.replace(yaml.find("0.0812"), std::string("0.0812").size(), "nan-ish-text");
  TempYaml file(yaml);

  const Calibration cal = load_calibration(file.path(), 1280, 720);
  EXPECT_FALSE(cal.ok);
}

// --- The placeholder detector ------------------------------------------------
//
// This is what keeps `calibrated` honest. A file of the nominal numbers loads
// perfectly and must still not be allowed to claim the camera has been
// calibrated, because a C922 has visible barrel distortion and a run that
// measured none did not measure.
TEST(Calibration, AllZeroDistortionIsNotACalibration)
{
  std::string yaml(kGoodYaml);
  yaml.replace(
    yaml.find("data: [0.0812, -0.1623, 0.0009, -0.0004, 0.0451]"),
    std::string("data: [0.0812, -0.1623, 0.0009, -0.0004, 0.0451]").size(),
    "data: [0.0, 0.0, 0.0, 0.0, 0.0]");
  TempYaml file(yaml);

  const Calibration cal = load_calibration(file.path(), 1280, 720);
  ASSERT_TRUE(cal.ok) << cal.why;      // it is a valid file...
  EXPECT_FALSE(has_distortion(cal));   // ...and not a calibration
}

TEST(Calibration, AFailedLoadIsNeverCalibrated)
{
  EXPECT_FALSE(has_distortion(load_calibration("/nonexistent/nope.yaml", 1280, 720)));
}

// --- URL forms ---------------------------------------------------------------

TEST(Calibration, SplitsAPackageUrl)
{
  std::string package, relative;
  ASSERT_TRUE(split_package_url("package://pimesh_bringup/config/camera_info/c922_720p.yaml",
      package, relative));
  EXPECT_EQ(package, "pimesh_bringup");
  EXPECT_EQ(relative, "config/camera_info/c922_720p.yaml");
}

TEST(Calibration, RejectsMalformedPackageUrls)
{
  std::string package, relative;
  // No path after the package name, a leading slash where the name belongs, and
  // a trailing slash naming no file. Each would otherwise be guessed at.
  EXPECT_FALSE(split_package_url("package://pimesh_bringup", package, relative));
  EXPECT_FALSE(split_package_url("package:///config/x.yaml", package, relative));
  EXPECT_FALSE(split_package_url("package://pimesh_bringup/", package, relative));
  // Not a package URL at all.
  EXPECT_FALSE(split_package_url("file:///tmp/x.yaml", package, relative));
  EXPECT_FALSE(split_package_url("/tmp/x.yaml", package, relative));
}

TEST(Calibration, ResolvesFileUrlsAndBarePaths)
{
  EXPECT_EQ(resolve_camera_info_url("file:///tmp/x.yaml"), "/tmp/x.yaml");
  EXPECT_EQ(resolve_camera_info_url("/tmp/x.yaml"), "/tmp/x.yaml");
  EXPECT_EQ(resolve_camera_info_url("relative/x.yaml"), "relative/x.yaml");
}

// An uninstalled package resolves to nothing rather than to a plausible-looking
// path that was never going to exist — the difference between "no such package"
// and "no such file", which send you to different places.
TEST(Calibration, AnUnknownPackageResolvesToNothing)
{
  EXPECT_TRUE(resolve_camera_info_url("package://no_such_pimesh_package/x.yaml").empty());
}
