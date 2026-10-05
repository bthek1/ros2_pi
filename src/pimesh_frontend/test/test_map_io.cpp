// #13's P20: the saved keyframe map.
//
// **Every way of getting this wrong loads a map.** A field written in one order and
// read in another round-trips perfectly if both halves agree, so the layout is also
// checked byte by byte against what map_io.hpp promises — test_mesh_io's lesson. And
// a file cut short is the sharpest case: read leniently it is a smaller map, which
// relocalises less often and is otherwise indistinguishable from a room with fewer
// keyframes in it.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include "pimesh_frontend/map_io.hpp"

using pimesh_frontend::Keyframe;
using pimesh_frontend::load_keyframes;
using pimesh_frontend::save_keyframes;

namespace
{

std::string temp_path(const std::string & name)
{
  return ::testing::TempDir() + "pimesh_map_io_" + name;
}

Keyframe keyframe(std::int64_t stamp, int features, int seed)
{
  Keyframe kf;
  kf.stamp_ns = stamp;
  const double a = 0.1 * seed;
  kf.odom_from_camera = cv::Affine3d(
    cv::Matx33d(std::cos(a), -std::sin(a), 0, std::sin(a), std::cos(a), 0, 0, 0, 1),
    cv::Vec3d(1.0 + seed, -2.0 * seed, 0.5));
  kf.descriptors.create(features, 32, CV_8U);
  for (int r = 0; r < features; ++r) {
    for (int c = 0; c < 32; ++c) {
      kf.descriptors.at<std::uint8_t>(r, c) = static_cast<std::uint8_t>((r * 31 + c * 7 + seed) & 0xff);
    }
    kf.bearings.emplace_back(0.01 * r, -0.02 * r, 1.0);
    kf.track_ids.push_back(seed * 1000 + r);
    if (r % 3 != 0) {
      kf.landmark_row.push_back(r);
      kf.landmarks.emplace_back(0.1 * r, 0.2, 2.0 + 0.01 * r);
    }
  }
  return kf;
}

std::vector<char> bytes_of(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  return std::vector<char>(std::istreambuf_iterator<char>(in), {});
}

void write_bytes(const std::string & path, const std::vector<char> & bytes)
{
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
}

}  // namespace

TEST(MapIo, RoundTripsEveryField)
{
  std::deque<Keyframe> saved{keyframe(100, 40, 1), keyframe(2000000000LL, 7, 2), keyframe(-5, 0, 3)};
  const std::string path = temp_path("round_trip");
  ASSERT_EQ(save_keyframes(path, saved), "");
  std::deque<Keyframe> loaded;
  ASSERT_EQ(load_keyframes(path, loaded), "");
  ASSERT_EQ(loaded.size(), saved.size());
  for (std::size_t i = 0; i < saved.size(); ++i) {
    const Keyframe & a = saved[i];
    const Keyframe & b = loaded[i];
    EXPECT_EQ(a.stamp_ns, b.stamp_ns);
    EXPECT_EQ(cv::norm(a.odom_from_camera.matrix - b.odom_from_camera.matrix), 0.0);
    ASSERT_EQ(a.descriptors.size(), b.descriptors.size());
    if (!a.descriptors.empty()) {EXPECT_EQ(cv::norm(a.descriptors, b.descriptors, cv::NORM_L1), 0.0);}
    EXPECT_EQ(a.bearings, b.bearings);
    EXPECT_EQ(a.track_ids, b.track_ids);
    EXPECT_EQ(a.landmarks, b.landmarks);
    EXPECT_EQ(a.landmark_row, b.landmark_row);
  }
}

TEST(MapIo, TheLayoutIsTheOneTheHeaderPromises)
{
  // A writer and reader that swapped two fields together would pass the round trip.
  // So: the magic, the count, and the first keyframe's leading fields at the offsets
  // map_io.hpp documents.
  std::deque<Keyframe> saved{keyframe(123456789, 4, 1)};
  const std::string path = temp_path("layout");
  ASSERT_EQ(save_keyframes(path, saved), "");
  const std::vector<char> b = bytes_of(path);
  ASSERT_GE(b.size(), 8u + 8u + 8u + 96u + 12u);
  EXPECT_EQ(std::string(b.data(), 8), "PIMESHKF");
  std::uint64_t count = 0;
  std::memcpy(&count, b.data() + 8, 8);
  EXPECT_EQ(count, 1u);
  std::int64_t stamp = 0;
  std::memcpy(&stamp, b.data() + 16, 8);
  EXPECT_EQ(stamp, 123456789);
  // rotation[0..8] row-major, then translation: translation x is double #9 after stamp.
  double r01 = 0.0, tx = 0.0;
  std::memcpy(&r01, b.data() + 24 + 1 * 8, 8);
  std::memcpy(&tx, b.data() + 24 + 9 * 8, 8);
  EXPECT_DOUBLE_EQ(r01, saved[0].odom_from_camera.rotation()(0, 1));
  EXPECT_DOUBLE_EQ(tx, saved[0].odom_from_camera.translation()[0]);
  std::int32_t rows = 0, cols = 0, type = -1;
  std::memcpy(&rows, b.data() + 24 + 96, 4);
  std::memcpy(&cols, b.data() + 24 + 100, 4);
  std::memcpy(&type, b.data() + 24 + 104, 4);
  EXPECT_EQ(rows, 4);
  EXPECT_EQ(cols, 32);
  EXPECT_EQ(type, CV_8U);
}

TEST(MapIo, EveryTruncationIsRefusedAndLeavesTheCallerAlone)
{
  std::deque<Keyframe> saved{keyframe(1, 20, 1), keyframe(2, 20, 2)};
  const std::string path = temp_path("truncated");
  ASSERT_EQ(save_keyframes(path, saved), "");
  const std::vector<char> whole = bytes_of(path);
  // Every cut, not a sample: a reader that tolerated one particular short read would
  // pass a sampled test by luck of where it cut.
  for (std::size_t cut = 0; cut < whole.size(); ++cut) {
    write_bytes(path, std::vector<char>(whole.begin(), whole.begin() + static_cast<long>(cut)));
    std::deque<Keyframe> out{keyframe(99, 3, 9)};
    EXPECT_NE(load_keyframes(path, out), "") << "a file cut at byte " << cut << " loaded";
    ASSERT_EQ(out.size(), 1u) << "a failed load replaced the caller's map";
    EXPECT_EQ(out[0].stamp_ns, 99);
  }
}

TEST(MapIo, TrailingBytesAreRefused)
{
  std::deque<Keyframe> saved{keyframe(1, 5, 1)};
  const std::string path = temp_path("trailing");
  ASSERT_EQ(save_keyframes(path, saved), "");
  std::vector<char> b = bytes_of(path);
  b.push_back(0);
  write_bytes(path, b);
  std::deque<Keyframe> out;
  EXPECT_NE(load_keyframes(path, out), "");
  EXPECT_TRUE(out.empty());
}

TEST(MapIo, AWrongMagicOrAMissingFileIsRefused)
{
  std::deque<Keyframe> out;
  EXPECT_NE(load_keyframes(temp_path("does_not_exist"), out), "");
  const std::string path = temp_path("magic");
  ASSERT_EQ(save_keyframes(path, {keyframe(1, 5, 1)}), "");
  std::vector<char> b = bytes_of(path);
  b[7] = 'X';
  write_bytes(path, b);
  EXPECT_NE(load_keyframes(path, out), "");
}

TEST(MapIo, AnImplausibleCountIsARefusalNotAnAllocation)
{
  const std::string path = temp_path("count");
  ASSERT_EQ(save_keyframes(path, {keyframe(1, 5, 1)}), "");
  std::vector<char> b = bytes_of(path);
  const std::uint64_t huge = 1ULL << 60;
  std::memcpy(b.data() + 8, &huge, 8);
  write_bytes(path, b);
  std::deque<Keyframe> out;
  EXPECT_NE(load_keyframes(path, out), "");
}

TEST(MapIo, AnEmptyMapRoundTripsAsEmpty)
{
  const std::string path = temp_path("empty");
  ASSERT_EQ(save_keyframes(path, {}), "");
  std::deque<Keyframe> out{keyframe(1, 1, 1)};
  ASSERT_EQ(load_keyframes(path, out), "");
  EXPECT_TRUE(out.empty());
}

TEST(MapIo, NoPartialFileIsLeftBehind)
{
  const std::string path = temp_path("partial");
  ASSERT_EQ(save_keyframes(path, {keyframe(1, 5, 1)}), "");
  std::ifstream partial(path + ".partial");
  EXPECT_FALSE(partial.good());
}
