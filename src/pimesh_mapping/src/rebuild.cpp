#include "pimesh_mapping/rebuild.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include "opencv2/imgcodecs.hpp"
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
  std::vector<cv::Affine3d> poses;
  poses.reserve(frames.size());
  for (const RememberedFrame & f : frames) {
    poses.push_back(correction_at(corrections, f.stamp_ns) * f.odom_from_camera);
  }
  return rebuild_volume_at(frames, poses, options, downsample);
}

RebuildResult rebuild_volume_at(
  const std::vector<RememberedFrame> & frames, const std::vector<cv::Affine3d> & poses,
  TsdfVolume::Options options, int downsample)
{
  options.allocation_stride = std::max(1, options.allocation_stride / std::max(1, downsample));
  RebuildResult out;
  out.volume = std::make_unique<TsdfVolume>(options);
  cv::Mat depth_m;
  for (std::size_t i = 0; i < frames.size() && i < poses.size(); ++i) {
    const cv::Affine3d & pose = poses[i];
    const cv::Vec3d t = pose.translation();
    if (!std::isfinite(t[0]) || !std::isfinite(t[1]) || !std::isfinite(t[2])) {continue;}
    const RememberedFrame & f = frames[i];
    f.depth_mm.convertTo(depth_m, CV_32FC1, 0.001);
    const TsdfVolume::IntegrateResult r = out.volume->integrate(depth_m, f.bgr, f.k, pose);
    out.blocks_refused += r.blocks_refused;
    ++out.integrated;
  }
  return out;
}

namespace
{

void write_affine(std::ostream & out, const cv::Affine3d & a)
{
  const cv::Matx33d r = a.rotation();
  const cv::Vec3d t = a.translation();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {out << ' ' << r(i, j);}
  }
  out << ' ' << t[0] << ' ' << t[1] << ' ' << t[2];
}

bool read_affine(std::istream & in, cv::Affine3d & a)
{
  cv::Matx33d r;
  cv::Vec3d t;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {if (!(in >> r(i, j))) {return false;}}
  }
  if (!(in >> t[0] >> t[1] >> t[2])) {return false;}
  a = cv::Affine3d(r, t);
  return true;
}

}  // namespace

std::string write_memory(
  const std::string & dir, const std::vector<RememberedFrame> & frames,
  const Corrections & corrections, const TsdfVolume::Options & options, int downsample)
{
  namespace fs = std::filesystem;
  const fs::path target(dir);
  const fs::path partial(dir + ".partial");
  std::error_code ec;
  fs::remove_all(partial, ec);
  if (!fs::create_directories(partial, ec)) {return "cannot create " + partial.string();}

  std::ofstream index(partial / "index.txt");
  // Every double at full precision: a pose round-tripped through six significant
  // figures is a pose moved by up to a millimetre, which is a measurement.
  index << std::setprecision(17);
  index << "# stamp_ns k00 k01 k02 k10 k11 k12 k20 k21 k22 odom(r00..r22 tx ty tz) "
           "used(r00..r22 tx ty tz) depth bgr\n";
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const RememberedFrame & f = frames[i];
    char depth_name[32], bgr_name[32];
    std::snprintf(depth_name, sizeof(depth_name), "d%06zu.png", i);
    std::snprintf(bgr_name, sizeof(bgr_name), "c%06zu.png", i);
    if (!cv::imwrite((partial / depth_name).string(), f.depth_mm)) {
      return std::string("cannot write ") + depth_name;
    }
    const bool has_bgr = !f.bgr.empty();
    if (has_bgr && !cv::imwrite((partial / bgr_name).string(), f.bgr)) {
      return std::string("cannot write ") + bgr_name;
    }
    index << f.stamp_ns;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {index << ' ' << f.k(r, c);}
    }
    write_affine(index, f.odom_from_camera);
    write_affine(index, f.map_from_odom_used);
    index << ' ' << depth_name << ' ' << (has_bgr ? bgr_name : "-") << '\n';
  }
  index.flush();
  if (!index) {return "write to index.txt failed";}

  std::ofstream corr(partial / "corrections.txt");
  corr << std::setprecision(17);
  corr << "# stamp_ns r00..r22 tx ty tz (map <- odom)\n";
  for (const auto & [stamp, c] : corrections) {
    corr << stamp;
    write_affine(corr, c);
    corr << '\n';
  }
  corr.flush();
  if (!corr) {return "write to corrections.txt failed";}

  std::ofstream meta(partial / "meta.txt");
  meta << std::setprecision(17)
       << "downsample " << downsample << '\n'
       << "voxel_size_m " << options.voxel_size_m << '\n'
       << "truncation_voxels " << options.truncation_voxels << '\n'
       << "min_weight " << options.min_weight << '\n'
       << "max_weight " << options.max_weight << '\n'
       << "max_range_m " << options.max_range_m << '\n'
       << "min_range_m " << options.min_range_m << '\n'
       << "max_blocks " << options.max_blocks << '\n'
       << "allocation_stride " << options.allocation_stride << '\n';
  meta.flush();
  if (!meta) {return "write to meta.txt failed";}

  fs::remove_all(target, ec);
  fs::rename(partial, target, ec);
  if (ec) {return "cannot move " + partial.string() + " into place: " + ec.message();}
  return std::string();
}

std::string read_memory(
  const std::string & dir, std::vector<RememberedFrame> & frames, Corrections & corrections,
  TsdfVolume::Options & options, int & downsample)
{
  namespace fs = std::filesystem;
  frames.clear();
  corrections.clear();
  {
    // Every key required: an option silently left at its default is a rebuild with a
    // different volume from the one it is compared with.
    std::ifstream meta(fs::path(dir) / "meta.txt");
    if (!meta) {return "no meta.txt in " + dir;}
    std::string key;
    int seen = 0;
    while (meta >> key) {
      if (key == "downsample") {meta >> downsample;} else if (key == "voxel_size_m") {
        meta >> options.voxel_size_m;
      } else if (key == "truncation_voxels") {meta >> options.truncation_voxels;} else if (
        key == "min_weight") {meta >> options.min_weight;} else if (key == "max_weight") {
        meta >> options.max_weight;
      } else if (key == "max_range_m") {meta >> options.max_range_m;} else if (
        key == "min_range_m") {meta >> options.min_range_m;} else if (key == "max_blocks") {
        meta >> options.max_blocks;
      } else if (key == "allocation_stride") {meta >> options.allocation_stride;} else {
        return "unknown key in meta.txt: " + key;
      }
      if (!meta) {return "unreadable value for " + key + " in meta.txt";}
      ++seen;
    }
    if (seen != 9) {return "meta.txt has " + std::to_string(seen) + " of 9 keys";}
  }
  std::ifstream index(fs::path(dir) / "index.txt");
  if (!index) {return "no index.txt in " + dir;}
  std::string line;
  while (std::getline(index, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream in(line);
    RememberedFrame f;
    if (!(in >> f.stamp_ns)) {return "unreadable stamp: " + line.substr(0, 40);}
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {if (!(in >> f.k(r, c))) {return "short line (K)";}}
    }
    if (!read_affine(in, f.odom_from_camera) || !read_affine(in, f.map_from_odom_used)) {
      return "short line (pose)";
    }
    std::string depth_name, bgr_name;
    if (!(in >> depth_name >> bgr_name)) {return "short line (files)";}
    f.depth_mm = cv::imread((fs::path(dir) / depth_name).string(), cv::IMREAD_UNCHANGED);
    if (f.depth_mm.empty() || f.depth_mm.type() != CV_16UC1) {
      return "missing or not 16-bit: " + depth_name;
    }
    if (bgr_name != "-") {
      f.bgr = cv::imread((fs::path(dir) / bgr_name).string(), cv::IMREAD_COLOR);
      if (f.bgr.empty()) {return "missing colour: " + bgr_name;}
    }
    frames.push_back(std::move(f));
  }
  if (frames.empty()) {return "index.txt names no frames";}
  std::ifstream corr(fs::path(dir) / "corrections.txt");
  if (!corr) {return "no corrections.txt in " + dir;}
  while (std::getline(corr, line)) {
    if (line.empty() || line[0] == '#') {continue;}
    std::istringstream in(line);
    std::int64_t stamp = 0;
    cv::Affine3d c;
    if (!(in >> stamp) || !read_affine(in, c)) {return "unreadable correction line";}
    corrections.emplace_back(stamp, c);
  }
  return std::string();
}

}  // namespace pimesh_mapping
