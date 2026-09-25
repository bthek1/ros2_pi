#ifndef PIMESH_FRONTEND__TUM_TRAJECTORY_HPP_
#define PIMESH_FRONTEND__TUM_TRAJECTORY_HPP_

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace pimesh_frontend
{

/// One pose, in the form the TUM RGB-D benchmark's file format wants.
struct TumPose
{
  /// Nanoseconds since the Unix epoch — the message's own stamp, not a double.
  std::int64_t stamp_ns {0};
  double x {0.0};
  double y {0.0};
  double z {0.0};
  double qx {0.0};
  double qy {0.0};
  double qz {0.0};
  double qw {1.0};
};

/// Format one pose as a TUM trajectory line: `timestamp tx ty tz qx qy qz qw`.
///
/// **This exists as a function so that a test can call it**, which is the rule
/// four helpers in this workspace have been moved for. It is also the piece of
/// P11 with the most ways to be wrong while looking right, because its reader is
/// `evo` — somebody else's parser, which will accept almost anything this could
/// emit:
///
///  - **the quaternion order is `qx qy qz qw`.** `geometry_msgs/Quaternion`'s
///    fields are in that order too, so the obvious loop is correct; a writer that
///    led with `w` produces a file `evo` reads without complaint and an ATE that
///    is *unchanged*, because an ATE over the translation part never looks at the
///    rotation. It would then be wrong for RPE and for anything later, silently.
///  - **the timestamp is fixed point, always.** `%g` on a 2011 Unix timestamp
///    gives `1.30503e+09`, which parses fine and makes every frame in a 20 s clip
///    share one time — `evo` associates them all against a single ground-truth
///    pose and reports a small, confident ATE over nothing.
///  - **it is formatted from the integer nanoseconds**, digit for digit, rather
///    than from a double of seconds. The round trip through
///    `parse_tum_timestamp` is then exact, which is what lets the trajectory this
///    writes be associated against the dataset index it came from rather than
///    merely near it.
std::string format_tum_line(const TumPose & pose);

/// Write a whole trajectory, or say why not.
///
/// \return an empty string on success, otherwise the reason — the same shape as
///         `Calibration::why`, because the caller is a probe whose whole job is
///         to report rather than to cope.
///
/// The refusals are the ones that would otherwise produce a file `evo` reads and
/// reports a plausible number over:
///
///  - **fewer than two poses.** An ATE over one pose is zero after alignment,
///    and zero reads as perfect. This is the `cost_mean=0.00` lesson in a new
///    file format.
///  - **a non-finite coordinate.** numpy loads `nan` happily and every statistic
///    downstream becomes `nan` — which prints, and which nobody reads as an
///    error the first time.
///  - **a zero-norm quaternion**, which is not a rotation; `evo` normalises on
///    load and divides by zero.
std::string write_tum_trajectory(const std::string & path, const std::vector<TumPose> & poses);

// --- Definitions -------------------------------------------------------------

inline std::string format_tum_line(const TumPose & pose)
{
  // Seconds and a nine-digit fraction, split as integers. Negative stamps cannot
  // occur (a ROS stamp is unsigned) and are not represented: the caller builds
  // these from a header.
  const std::int64_t seconds = pose.stamp_ns / 1000000000LL;
  const std::int64_t nanos = pose.stamp_ns % 1000000000LL;

  char buffer[256];
  // %.9g on the coordinates: enough digits that a metre-scale position keeps
  // sub-micrometre resolution, and short enough that a file of 400 poses stays
  // readable. Unlike the timestamp, scientific notation here is harmless — a
  // coordinate is a value, not a key.
  const int written = std::snprintf(
    buffer, sizeof(buffer), "%lld.%09lld %.9g %.9g %.9g %.9g %.9g %.9g %.9g",
    static_cast<long long>(seconds), static_cast<long long>(nanos),
    pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw);
  if (written <= 0) {return std::string();}
  return std::string(buffer, static_cast<std::size_t>(written));
}

inline std::string write_tum_trajectory(
  const std::string & path, const std::vector<TumPose> & poses)
{
  if (poses.size() < 2) {
    return "a trajectory of " + std::to_string(poses.size()) +
           " pose(s) is not a trajectory — evo would align it to an ATE of zero, "
           "and zero reads as perfect";
  }
  for (std::size_t i = 0; i < poses.size(); ++i) {
    const TumPose & p = poses[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(p.qx) || !std::isfinite(p.qy) || !std::isfinite(p.qz) ||
      !std::isfinite(p.qw))
    {
      return "pose " + std::to_string(i) + " has a non-finite component";
    }
    const double norm =
      std::sqrt(p.qx * p.qx + p.qy * p.qy + p.qz * p.qz + p.qw * p.qw);
    if (!(norm > 1e-6)) {
      return "pose " + std::to_string(i) + " has a zero-norm quaternion, which is "
             "not a rotation";
    }
  }

  std::ofstream out(path, std::ios::trunc);
  if (!out) {return "cannot open " + path + " for writing";}
  // A `#` header, which every TUM file carries and every reader of the format
  // skips. It costs one line and it is what tells somebody opening the file what
  // the eight columns are.
  out << "# timestamp tx ty tz qx qy qz qw\n";
  for (const TumPose & pose : poses) {out << format_tum_line(pose) << '\n';}
  out.flush();
  if (!out) {return "write to " + path + " failed";}
  return std::string();
}

}  // namespace pimesh_frontend

#endif  // PIMESH_FRONTEND__TUM_TRAJECTORY_HPP_
