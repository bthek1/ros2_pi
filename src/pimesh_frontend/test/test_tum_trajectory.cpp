// The TUM trajectory writer — the file `evo` reads to produce P11's number.
//
// **This is an instrument, and a gate's instrument needs its own tests.** The
// same argument `test_orb_reference` makes for the ORB reference script: the
// thing deciding a gate's verdict cannot be the one part of it nobody checks.
// What makes it sharp here is that the reader is somebody else's parser, and
// `evo` accepts nearly anything this could emit — so every way of getting it
// wrong ends in a number rather than an error.
//
// Three of those ways, and each has a test below:
//
//   - **`%g` on the timestamp.** `1.30503e+09` parses perfectly and collapses a
//     20 s clip onto one instant, which `evo` associates against a single
//     ground-truth pose and reports a small, confident ATE over.
//   - **the quaternion in `w x y z` order.** An ATE over the translation part
//     never looks at the rotation, so this is invisible in exactly the figure
//     P11 is about and wrong in everything after it.
//   - **a file with one pose in it.** Umeyama alignment puts a single point
//     exactly on its reference, so the ATE is 0.000000 — and zero reads as
//     perfect. This is `gates/keypoints.sh`'s `cost_mean=0.00` in a new costume.
//
// The expectations are **literal bytes**, not a round trip through a parser
// written beside the writer — `test_mesh_io`'s lesson, where a writer and a
// reader that are wrong together round-trip perfectly.

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "pimesh_frontend/tum_trajectory.hpp"

using pimesh_frontend::TumPose;
using pimesh_frontend::format_tum_line;
using pimesh_frontend::write_tum_trajectory;

namespace
{

/// A pose with every component distinct, so a swapped field cannot pass.
TumPose sample()
{
  TumPose p;
  p.stamp_ns = 1305031452791720000LL;
  p.x = 1.5;
  p.y = -2.25;
  p.z = 3.125;
  p.qx = 0.1;
  p.qy = 0.2;
  p.qz = 0.3;
  p.qw = 0.4;
  return p;
}

std::string temp_path()
{
  return (std::filesystem::temp_directory_path() /
         ("pimesh_traj_" + std::to_string(::rand()) + ".tum")).string();
}

std::vector<std::string> lines_of(const std::string & path)
{
  std::vector<std::string> out;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {out.push_back(line);}
  return out;
}

}  // namespace

TEST(TumTrajectory, IsTheEightFieldsInTumsOrder)
{
  // `timestamp tx ty tz qx qy qz qw`. Every number is distinct and none is a
  // round figure, so a transposed pair or a dropped field changes the string.
  EXPECT_EQ(
    format_tum_line(sample()),
    "1305031452.791720000 1.5 -2.25 3.125 0.1 0.2 0.3 0.4");
}

TEST(TumTrajectory, PutsTheQuaternionInXyzwAndNotWxyz)
{
  // Stated separately from the test above because it is the failure that an ATE
  // cannot see: `evo_ape` with the default translation-part relation reads
  // columns 2-4 and never looks at 5-8. A writer that led with `w` would pass
  // every number P11 prints and be wrong for RPE and for everything after it.
  TumPose p = sample();
  p.qx = 0.0;
  p.qy = 0.0;
  p.qz = 0.0;
  p.qw = 1.0;
  const std::string line = format_tum_line(p);
  const std::string tail = line.substr(line.size() - 7);
  EXPECT_EQ(tail, "0 0 0 1") << "identity must be '0 0 0 1', not '1 0 0 0': " << line;
}

TEST(TumTrajectory, NeverWritesATimestampInScientificNotation)
{
  // The one field where a float format is a correctness bug rather than a
  // precision one — it is the association key.
  const std::string line = format_tum_line(sample());
  const std::string stamp = line.substr(0, line.find(' '));
  EXPECT_EQ(stamp.find('e'), std::string::npos) << stamp;
  EXPECT_EQ(stamp.find('E'), std::string::npos) << stamp;
  EXPECT_EQ(stamp, "1305031452.791720000");
}

TEST(TumTrajectory, KeepsTwoFramesAtThirtyHertzDistinct)
{
  // 33 ms apart is the smallest gap a 30 Hz sequence produces, and the timestamp
  // is what `evo` associates on: two lines with one stamp are one pose as far as
  // the ATE is concerned, and the run silently shrinks.
  TumPose a = sample();
  TumPose b = sample();
  b.stamp_ns += 33333333LL;
  EXPECT_NE(
    format_tum_line(a).substr(0, format_tum_line(a).find(' ')),
    format_tum_line(b).substr(0, format_tum_line(b).find(' ')));
}

TEST(TumTrajectory, PadsSubSecondStampsRatherThanTruncatingThem)
{
  // 1.05 s must be `1.050000000`, not `1.5` — a fraction printed without its
  // leading zeros is a timestamp that is monotonically wrong and still ordered,
  // which is the shape of error nobody spots in a column of numbers.
  TumPose p;
  p.stamp_ns = 1050000000LL;
  p.qw = 1.0;
  EXPECT_EQ(format_tum_line(p).substr(0, 11), "1.050000000");
}

TEST(TumTrajectory, WritesAHeaderAndOneLinePerPose)
{
  const std::string path = temp_path();
  const std::vector<TumPose> poses = {sample(), sample(), sample()};
  std::vector<TumPose> rising = poses;
  for (std::size_t i = 0; i < rising.size(); ++i) {
    rising[i].stamp_ns += static_cast<std::int64_t>(i) * 33333333LL;
  }

  ASSERT_EQ(write_tum_trajectory(path, rising), "");
  const auto lines = lines_of(path);
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_EQ(lines[0], "# timestamp tx ty tz qx qy qz qw");
  EXPECT_EQ(lines[1], format_tum_line(rising[0]));
  EXPECT_EQ(lines[3], format_tum_line(rising[2]));
  std::filesystem::remove(path);
}

TEST(TumTrajectory, RefusesATrajectoryOfFewerThanTwoPoses)
{
  // The false green this refusal exists for: Umeyama puts one point exactly on
  // its reference, so a one-pose file gives an ATE of 0.000000 and no warning.
  const std::string path = temp_path();
  EXPECT_NE(write_tum_trajectory(path, {}), "");
  EXPECT_NE(write_tum_trajectory(path, {sample()}), "");
  // And it writes nothing — a stale file from a previous run being left in place
  // would be measured as though it were this run's.
  EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(TumTrajectory, RefusesNonFiniteAndZeroNormPoses)
{
  const std::string path = temp_path();
  std::vector<TumPose> poses = {sample(), sample()};
  poses[1].stamp_ns += 1000;

  poses[1].z = std::numeric_limits<double>::quiet_NaN();
  // numpy reads `nan` without complaint and every statistic downstream becomes
  // nan — which prints, and prints in the same table as a real result.
  EXPECT_NE(write_tum_trajectory(path, poses), "");

  poses[1].z = 0.0;
  poses[1].qx = poses[1].qy = poses[1].qz = poses[1].qw = 0.0;
  const std::string why = write_tum_trajectory(path, poses);
  EXPECT_NE(why, "");
  EXPECT_NE(why.find("zero-norm"), std::string::npos) << why;

  EXPECT_FALSE(std::filesystem::exists(path));
}

TEST(TumTrajectory, SaysWhyWhenItCannotWrite)
{
  std::vector<TumPose> poses = {sample(), sample()};
  poses[1].stamp_ns += 1000;
  const std::string why = write_tum_trajectory("/nonexistent/dir/traj.tum", poses);
  EXPECT_NE(why, "");
  EXPECT_NE(why.find("/nonexistent/dir/traj.tum"), std::string::npos) << why;
}
