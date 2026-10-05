#include "pimesh_frontend/map_io.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <vector>

namespace pimesh_frontend
{

namespace
{

constexpr char kMagic[8] = {'P', 'I', 'M', 'E', 'S', 'H', 'K', 'F'};
/// Ceilings on what one keyframe can hold. Not limits anybody should reach — ORB is
/// capped at 500 features — but a corrupt count read as 10^18 must fail as a
/// refusal, not as an allocation.
constexpr std::uint64_t kMaxKeyframes = 1000000;
constexpr std::uint64_t kMaxPerFrame = 100000;

template<typename T>
void put(std::ofstream & out, const T & value)
{
  out.write(reinterpret_cast<const char *>(&value), sizeof(T));
}

template<typename T>
void put_vector(std::ofstream & out, const std::vector<T> & values)
{
  put(out, static_cast<std::uint64_t>(values.size()));
  if (!values.empty()) {
    out.write(reinterpret_cast<const char *>(values.data()),
      static_cast<std::streamsize>(values.size() * sizeof(T)));
  }
}

template<typename T>
bool get(std::ifstream & in, T & value)
{
  in.read(reinterpret_cast<char *>(&value), sizeof(T));
  return static_cast<bool>(in);
}

template<typename T>
bool get_vector(std::ifstream & in, std::vector<T> & values)
{
  std::uint64_t n = 0;
  if (!get(in, n) || n > kMaxPerFrame) {return false;}
  values.resize(static_cast<std::size_t>(n));
  if (n == 0) {return true;}
  in.read(reinterpret_cast<char *>(values.data()), static_cast<std::streamsize>(n * sizeof(T)));
  return static_cast<bool>(in);
}

}  // namespace

std::string save_keyframes(const std::string & path, const std::deque<Keyframe> & frames)
{
  for (const Keyframe & kf : frames) {
    if (!kf.descriptors.empty() && kf.descriptors.type() != CV_8U) {
      return "a keyframe's descriptors are not CV_8U";
    }
  }
  const std::string partial = path + ".partial";
  {
    std::ofstream out(partial, std::ios::binary | std::ios::trunc);
    if (!out) {return "cannot open " + partial + " for writing";}
    out.write(kMagic, sizeof(kMagic));
    put(out, static_cast<std::uint64_t>(frames.size()));
    for (const Keyframe & kf : frames) {
      put(out, kf.stamp_ns);
      const cv::Matx33d r = kf.odom_from_camera.rotation();
      const cv::Vec3d t = kf.odom_from_camera.translation();
      for (int i = 0; i < 9; ++i) {put(out, r.val[i]);}
      for (int i = 0; i < 3; ++i) {put(out, t[i]);}
      const cv::Mat d = kf.descriptors.isContinuous() ? kf.descriptors : kf.descriptors.clone();
      put(out, static_cast<std::int32_t>(d.rows));
      put(out, static_cast<std::int32_t>(d.cols));
      put(out, static_cast<std::int32_t>(d.empty() ? CV_8U : d.type()));
      if (!d.empty()) {
        out.write(reinterpret_cast<const char *>(d.data), static_cast<std::streamsize>(d.total()));
      }
      put_vector(out, kf.bearings);
      put_vector(out, kf.track_ids);
      put_vector(out, kf.landmarks);
      put_vector(out, kf.landmark_row);
    }
    out.flush();
    if (!out) {return "write to " + partial + " failed";}
  }
  if (std::rename(partial.c_str(), path.c_str()) != 0) {
    return "cannot move " + partial + " onto " + path;
  }
  return "";
}

std::string load_keyframes(const std::string & path, std::deque<Keyframe> & out)
{
  std::ifstream in(path, std::ios::binary);
  if (!in) {return "cannot open " + path;}
  char magic[sizeof(kMagic)];
  if (!in.read(magic, sizeof(magic)) || std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
    return path + " is not a keyframe map (wrong magic)";
  }
  std::uint64_t count = 0;
  if (!get(in, count) || count > kMaxKeyframes) {return path + ": implausible keyframe count";}

  std::deque<Keyframe> frames;
  for (std::uint64_t i = 0; i < count; ++i) {
    Keyframe kf;
    cv::Matx33d r;
    cv::Vec3d t;
    bool ok = get(in, kf.stamp_ns);
    for (int j = 0; j < 9 && ok; ++j) {ok = get(in, r.val[j]);}
    for (int j = 0; j < 3 && ok; ++j) {ok = get(in, t[j]);}
    std::int32_t rows = 0, cols = 0, type = 0;
    ok = ok && get(in, rows) && get(in, cols) && get(in, type);
    if (!ok) {return path + ": truncated in keyframe " + std::to_string(i);}
    if (rows < 0 || cols < 0 || static_cast<std::uint64_t>(rows) > kMaxPerFrame || cols > 1024 ||
      type != CV_8U)
    {
      return path + ": implausible descriptor block in keyframe " + std::to_string(i);
    }
    // Shape as written, including 0 x 32: a keyframe with no features still says how
    // wide its descriptors would have been, and a loader that drops that is a round
    // trip that is not one (test_map_io found it).
    if (cols > 0) {kf.descriptors.create(rows, cols, CV_8U);}
    if (rows > 0 && cols > 0) {
      if (!in.read(reinterpret_cast<char *>(kf.descriptors.data),
        static_cast<std::streamsize>(kf.descriptors.total())))
      {
        return path + ": truncated in keyframe " + std::to_string(i);
      }
    }
    if (!get_vector(in, kf.bearings) || !get_vector(in, kf.track_ids) ||
      !get_vector(in, kf.landmarks) || !get_vector(in, kf.landmark_row))
    {
      return path + ": truncated in keyframe " + std::to_string(i);
    }
    kf.odom_from_camera = cv::Affine3d(r, t);
    frames.push_back(std::move(kf));
  }
  // Trailing bytes are a refusal: a file longer than its count says is two writers'
  // worth of map, or a count that was wrong — either way not this map.
  char extra = 0;
  if (in.read(&extra, 1)) {return path + ": bytes after the last keyframe";}
  out = std::move(frames);
  return "";
}

}  // namespace pimesh_frontend
