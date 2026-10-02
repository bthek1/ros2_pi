#include "pimesh_mapping/ground_truth.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include "pimesh_core/stats.hpp"
#include "pimesh_mapping/scale_aligner.hpp"

namespace pimesh_mapping
{

Sim3 umeyama(const std::vector<cv::Vec3d> & from, const std::vector<cv::Vec3d> & to)
{
  Sim3 out;
  const std::size_t n = std::min(from.size(), to.size());
  if (n < 3) {return out;}
  cv::Vec3d mf(0, 0, 0), mt(0, 0, 0);
  for (std::size_t i = 0; i < n; ++i) {mf += from[i]; mt += to[i];}
  mf *= 1.0 / static_cast<double>(n);
  mt *= 1.0 / static_cast<double>(n);
  cv::Matx33d cov = cv::Matx33d::zeros();
  double var_from = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const cv::Vec3d a = from[i] - mf;
    const cv::Vec3d b = to[i] - mt;
    cov += cv::Matx33d(b[0] * a[0], b[0] * a[1], b[0] * a[2],
                       b[1] * a[0], b[1] * a[1], b[1] * a[2],
                       b[2] * a[0], b[2] * a[1], b[2] * a[2]);
    var_from += a.dot(a);
  }
  cov *= 1.0 / static_cast<double>(n);
  var_from /= static_cast<double>(n);
  if (var_from < 1e-12) {return out;}

  cv::Mat w, u, vt;
  cv::SVD::compute(cv::Mat(cov), w, u, vt);
  // A collinear `from` leaves the second singular value at zero: a free rotation.
  if (w.at<double>(1) < 1e-9 * std::max(1.0, w.at<double>(0))) {return out;}
  cv::Matx33d s = cv::Matx33d::eye();
  // The reflection guard: the SVD's best orthogonal matrix can be a reflection, which
  // fits as well and is not a rotation.
  if (cv::determinant(cv::Mat(u * vt)) < 0.0) {s(2, 2) = -1.0;}
  const cv::Matx33d um(reinterpret_cast<const double *>(u.data));
  const cv::Matx33d vtm(reinterpret_cast<const double *>(vt.data));
  out.r = um * s * vtm;
  const double trace_ds = w.at<double>(0) * s(0, 0) + w.at<double>(1) * s(1, 1) +
    w.at<double>(2) * s(2, 2);
  out.s = trace_ds / var_from;
  out.t = mt - out.s * (out.r * mf);
  out.ok = std::isfinite(out.s) && out.s > 0.0;
  return out;
}

cv::Affine3d apply(const Sim3 & sim, const cv::Affine3d & pose)
{
  return cv::Affine3d(sim.r * pose.rotation(), sim.s * (sim.r * pose.translation()) + sim.t);
}

std::vector<std::pair<std::int64_t, cv::Affine3d>> load_tum_groundtruth(const std::string & path)
{
  std::vector<std::pair<std::int64_t, cv::Affine3d>> out;
  std::ifstream in(path);
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream fields(line);
    double stamp, tx, ty, tz, qx, qy, qz, qw;
    if (!(fields >> stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {continue;}
    const double n = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (!(n > 1e-9)) {continue;}
    const double x = qx / n, y = qy / n, z = qz / n, w = qw / n;
    const cv::Matx33d r(
      1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
      2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
      2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y));
    // Nearest-sample matching at a 20 ms tolerance does not need the exact integer
    // that dataset_reader takes such care over: a few hundred ns of rounding here
    // cannot move which sample is nearest.
    out.emplace_back(static_cast<std::int64_t>(std::llround(stamp * 1e9)),
      cv::Affine3d(r, cv::Vec3d(tx, ty, tz)));
  }
  std::sort(out.begin(), out.end(), [](const auto & a, const auto & b) {return a.first < b.first;});
  return out;
}

bool groundtruth_at(
  const std::vector<std::pair<std::int64_t, cv::Affine3d>> & gt, std::int64_t stamp_ns,
  cv::Affine3d & pose, std::int64_t tolerance_ns)
{
  const auto it = std::lower_bound(gt.begin(), gt.end(), stamp_ns,
      [](const auto & e, std::int64_t s) {return e.first < s;});
  const std::pair<std::int64_t, cv::Affine3d> * best = nullptr;
  if (it != gt.end()) {best = &*it;}
  if (it != gt.begin()) {
    const auto & prev = *std::prev(it);
    if (!best || std::llabs(prev.first - stamp_ns) < std::llabs(best->first - stamp_ns)) {
      best = &prev;
    }
  }
  if (!best || std::llabs(best->first - stamp_ns) > tolerance_ns) {return false;}
  pose = best->second;
  return true;
}

SurfaceComparison compare_surfaces(
  const TsdfVolume & volume, const TsdfVolume & reference, const std::vector<cv::Affine3d> & poses,
  const cv::Matx33d & k, cv::Size size, double min_overlap, double tolerance)
{
  SurfaceComparison out;
  std::vector<double> gaps, agrees;
  cv::Mat seen, truth;
  for (const cv::Affine3d & pose : poses) {
    volume.raycast(k, pose, size, seen);
    reference.raycast(k, pose, size, truth);
    double gap = 0.0, overlap = 0.0, agree = 0.0;
    // Reference as the "expected" side, the arm as the "incoming": agreement is then
    // a fraction of the arm's surface the reference confirms.
    if (surface_gap(truth, seen, min_overlap, gap, overlap)) {gaps.push_back(gap);}
    if (surface_agreement(truth, seen, tolerance, agree)) {agrees.push_back(agree);}
    ++out.views;
  }
  if (!gaps.empty()) {out.gap_m = pimesh_core::percentile(gaps, 0.5);}
  if (!agrees.empty()) {out.agree = pimesh_core::percentile(agrees, 0.5);}
  return out;
}

ArmScore score_arm(
  const std::vector<RememberedFrame> & frames, const std::vector<cv::Affine3d> & poses,
  const std::vector<std::pair<std::int64_t, cv::Affine3d>> & groundtruth,
  const TsdfVolume::Options & options, int downsample, double min_overlap, double tolerance)
{
  ArmScore score;
  score.frames = frames.size();
  std::vector<RememberedFrame> used;
  std::vector<cv::Affine3d> arm, truth;
  std::vector<cv::Vec3d> truth_positions, arm_positions;
  for (std::size_t i = 0; i < frames.size() && i < poses.size(); ++i) {
    cv::Affine3d gt;
    if (!groundtruth_at(groundtruth, frames[i].stamp_ns, gt)) {continue;}
    used.push_back(frames[i]);
    arm.push_back(poses[i]);
    truth.push_back(gt);
    truth_positions.push_back(gt.translation());
    arm_positions.push_back(poses[i].translation());
  }
  score.judged = used.size();
  score.alignment = umeyama(truth_positions, arm_positions);
  if (!score.alignment.ok) {return score;}

  std::vector<cv::Affine3d> reference_poses;
  double sq = 0.0;
  for (std::size_t i = 0; i < truth.size(); ++i) {
    reference_poses.push_back(apply(score.alignment, truth[i]));
    const cv::Vec3d d = reference_poses.back().translation() - arm[i].translation();
    sq += d.dot(d);
  }
  score.ate_m = std::sqrt(sq / static_cast<double>(truth.size()));

  const RebuildResult arm_volume = rebuild_volume_at(used, arm, options, downsample);
  const RebuildResult reference = rebuild_volume_at(used, reference_poses, options, downsample);

  // Viewed from the reference poses — where the camera really was — at a quarter of
  // the stored resolution, the same reduction the live aligner ray-casts at. Every
  // judged frame is a view, so the figure covers the whole session.
  const RememberedFrame & f0 = used.front();
  const cv::Size size(std::max(1, f0.depth_mm.cols / 4), std::max(1, f0.depth_mm.rows / 4));
  const double sx = static_cast<double>(size.width) / f0.depth_mm.cols;
  const double sy = static_cast<double>(size.height) / f0.depth_mm.rows;
  const cv::Matx33d k(f0.k(0, 0) * sx, 0.0, f0.k(0, 2) * sx, 0.0, f0.k(1, 1) * sy, f0.k(1, 2) * sy,
    0.0, 0.0, 1.0);
  score.surface = compare_surfaces(*arm_volume.volume, *reference.volume, reference_poses, k, size,
    min_overlap, tolerance);
  return score;
}

}  // namespace pimesh_mapping
