// CameraInfo's matrix layout.
//
// Worth a test precisely because getting it wrong changes nothing visible. A P
// matrix left at zeros still publishes, still shows up in `ros2 topic echo`,
// and still looks like a CameraInfo; it only fails much later, as a depth
// unprojection that maps the room to a point. Nine indices and twelve indices
// with two different meanings is the kind of thing to pin down once.

#include <vector>

#include "gtest/gtest.h"
#include "pimesh_camera/camera_info.hpp"

using pimesh_camera::make_camera_info;

namespace
{
// The nominal C922 720p intrinsics the node ships with, row-major.
const std::vector<double> kK{907.0, 0.0, 640.0, 0.0, 907.0, 360.0, 0.0, 0.0, 1.0};
const std::vector<double> kD{0.1, -0.2, 0.001, 0.002, 0.05};
}  // namespace

TEST(CameraInfo, CarriesTheIntrinsicsAndSizeThrough)
{
  const auto info = make_camera_info(1280, 720, kK, kD);

  EXPECT_EQ(info.width, 1280u);
  EXPECT_EQ(info.height, 720u);
  EXPECT_EQ(info.distortion_model, "plumb_bob");
  EXPECT_EQ(info.d, kD);
  for (std::size_t i = 0; i < kK.size(); ++i) {
    EXPECT_DOUBLE_EQ(info.k[i], kK[i]) << "K[" << i << "]";
  }
}

// R is identity for a single unrectified camera. Not zeros: a consumer that
// rotates by R would send every ray to the origin.
TEST(CameraInfo, RectificationMatrixIsIdentity)
{
  const auto info = make_camera_info(1280, 720, kK, kD);

  const double expected[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(info.r[i], expected[i]) << "R[" << i << "]";
  }
}

// P is K with a zero fourth column, 3x4 row-major:
//
//     [ fx   0  cx  0 ]
//     [  0  fy  cy  0 ]
//     [  0   0   1  0 ]
//
// The indices that matter are 0, 2, 5, 6 and 10, and every other entry must be
// zero — including p[3] and p[7], the stereo baseline terms, which are zero
// because this is one camera.
TEST(CameraInfo, ProjectionMatrixIsKWithAZeroFourthColumn)
{
  const auto info = make_camera_info(1280, 720, kK, kD);

  EXPECT_DOUBLE_EQ(info.p[0], 907.0) << "fx";
  EXPECT_DOUBLE_EQ(info.p[2], 640.0) << "cx";
  EXPECT_DOUBLE_EQ(info.p[5], 907.0) << "fy";
  EXPECT_DOUBLE_EQ(info.p[6], 360.0) << "cy";
  EXPECT_DOUBLE_EQ(info.p[10], 1.0);

  for (std::size_t i : {1u, 3u, 4u, 7u, 8u, 9u, 11u}) {
    EXPECT_DOUBLE_EQ(info.p[i], 0.0) << "P[" << i << "] should be zero";
  }
}

// Projecting a point through P must land where projecting it through K does,
// which is the property the layout above exists to have. A transposed P would
// pass every index check and fail this one.
TEST(CameraInfo, ProjectingThroughPAgreesWithK)
{
  const auto info = make_camera_info(1280, 720, kK, kD);

  // A point 2 m ahead and 0.5 m to the right, in the optical frame.
  const double x = 0.5, y = -0.25, z = 2.0;

  const double u_k = (info.k[0] * x + info.k[2] * z) / z;
  const double v_k = (info.k[4] * y + info.k[5] * z) / z;
  const double u_p = (info.p[0] * x + info.p[1] * y + info.p[2] * z + info.p[3]) / z;
  const double v_p = (info.p[4] * x + info.p[5] * y + info.p[6] * z + info.p[7]) / z;

  EXPECT_DOUBLE_EQ(u_p, u_k);
  EXPECT_DOUBLE_EQ(v_p, v_k);
  // ...and the answer is where it should be on a 1280x720 image.
  EXPECT_NEAR(u_p, 640.0 + 907.0 * 0.25, 1e-9);
  EXPECT_NEAR(v_p, 360.0 - 907.0 * 0.125, 1e-9);
}

// A short or empty K must not read past the end of the input. The node's
// parameter is a double array a user can set to anything.
TEST(CameraInfo, ToleratesAShortIntrinsicMatrix)
{
  const auto info = make_camera_info(640, 480, {}, {});

  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_DOUBLE_EQ(info.k[i], 0.0) << "K[" << i << "]";
  }
  // R is still identity — it does not come from the parameter.
  EXPECT_DOUBLE_EQ(info.r[0], 1.0);
  EXPECT_EQ(info.width, 640u);
}
