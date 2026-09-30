#include "pimesh_mapping/rebuild.hpp"

#include <algorithm>
#include <cmath>

#include "opencv2/imgproc.hpp"

namespace pimesh_mapping
{

FrameMemory::FrameMemory(const Config & config)
: config_(config) {}

bool FrameMemory::offer(
  std::int64_t stamp_ns, const cv::Mat & depth_m, const cv::Mat & bgr, const cv::Matx33d & k,
  const cv::Affine3d & odom_from_camera, const cv::Affine3d & map_from_odom_used)
{
  const std::size_t n = offered_++;
  if (n % stride_ != 0) {return false;}
  if (depth_m.empty() || depth_m.type() != CV_32FC1) {return false;}
  const int ds = std::max(1, config_.downsample);

  RememberedFrame f;
  f.stamp_ns = stamp_ns;
  // Nearest, not area: averaging across an occlusion edge invents a depth between
  // foreground and background that no surface has — a flying pixel the TSDF would
  // integrate as a thin wall in mid-air.
  cv::Mat small;
  cv::resize(depth_m, small, cv::Size(depth_m.cols / ds, depth_m.rows / ds), 0, 0, cv::INTER_NEAREST);
  f.depth_mm.create(small.size(), CV_16UC1);
  for (int r = 0; r < small.rows; ++r) {
    const float * in = small.ptr<float>(r);
    std::uint16_t * out = f.depth_mm.ptr<std::uint16_t>(r);
    for (int c = 0; c < small.cols; ++c) {
      const float mm = in[c] * 1000.0F;
      out[c] = (std::isfinite(mm) && mm > 0.0F && mm < 65535.0F) ?
        static_cast<std::uint16_t>(std::lround(mm)) : 0;
    }
  }
  if (!bgr.empty() && bgr.type() == CV_8UC3 && bgr.size() == depth_m.size()) {
    cv::resize(bgr, f.bgr, small.size(), 0, 0, cv::INTER_AREA);
  }
  const double sx = static_cast<double>(small.cols) / depth_m.cols;
  const double sy = static_cast<double>(small.rows) / depth_m.rows;
  f.k = cv::Matx33d(k(0, 0) * sx, 0.0, k(0, 2) * sx, 0.0, k(1, 1) * sy, k(1, 2) * sy, 0.0, 0.0, 1.0);
  f.odom_from_camera = odom_from_camera;
  f.map_from_odom_used = map_from_odom_used;
  frames_.push_back(std::move(f));

  if (frames_.size() > std::max<std::size_t>(2, config_.max_frames)) {
    // Keep every other frame, first included, and admit half as often from here.
    std::vector<RememberedFrame> kept;
    kept.reserve(frames_.size() / 2 + 1);
    for (std::size_t i = 0; i < frames_.size(); i += 2) {kept.push_back(std::move(frames_[i]));}
    frames_ = std::move(kept);
    stride_ *= 2;
    ++halvings_;
  }
  return true;
}

std::size_t FrameMemory::bytes() const
{
  std::size_t total = 0;
  for (const RememberedFrame & f : frames_) {
    total += f.depth_mm.total() * f.depth_mm.elemSize() + f.bgr.total() * f.bgr.elemSize();
  }
  return total;
}

void FrameMemory::clear()
{
  frames_.clear();
  stride_ = 1;
  offered_ = 0;
  halvings_ = 0;
}

cv::Affine3d correction_at(const Corrections & corrections, std::int64_t stamp_ns)
{
  const auto it = std::upper_bound(
    corrections.begin(), corrections.end(), stamp_ns,
    [](std::int64_t s, const std::pair<std::int64_t, cv::Affine3d> & c) {return s < c.first;});
  if (it == corrections.begin()) {return cv::Affine3d::Identity();}
  return std::prev(it)->second;
}

Shift pose_shift(const std::vector<RememberedFrame> & frames, const Corrections & corrections)
{
  Shift s;
  for (const RememberedFrame & f : frames) {
    const cv::Affine3d live = f.map_from_odom_used * f.odom_from_camera;
    const cv::Affine3d corrected = correction_at(corrections, f.stamp_ns) * f.odom_from_camera;
    s.max_m = std::max(s.max_m, cv::norm(corrected.translation() - live.translation()));
    const cv::Matx33d r = live.rotation().t() * corrected.rotation();
    const double c = std::clamp((cv::trace(r) - 1.0) / 2.0, -1.0, 1.0);
    s.max_deg = std::max(s.max_deg, std::acos(c) * 180.0 / CV_PI);
  }
  return s;
}

RebuildResult rebuild_volume(
  const std::vector<RememberedFrame> & frames, const Corrections & corrections,
  TsdfVolume::Options options, int downsample)
{
  options.allocation_stride = std::max(1, options.allocation_stride / std::max(1, downsample));
  RebuildResult out;
  out.volume = std::make_unique<TsdfVolume>(options);
  cv::Mat depth_m;
  for (const RememberedFrame & f : frames) {
    f.depth_mm.convertTo(depth_m, CV_32FC1, 0.001);
    const cv::Affine3d pose = correction_at(corrections, f.stamp_ns) * f.odom_from_camera;
    const TsdfVolume::IntegrateResult r = out.volume->integrate(depth_m, f.bgr, f.k, pose);
    out.blocks_refused += r.blocks_refused;
    ++out.integrated;
  }
  return out;
}

}  // namespace pimesh_mapping
