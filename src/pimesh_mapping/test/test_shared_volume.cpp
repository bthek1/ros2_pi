// The object `fusion_node` fills and `mesh_node` meshes, and the rendezvous
// between them.
//
// **Everything here fails silently and produces a mesh.** A registry that hands
// out a fresh volume per lookup gives `mesh_node` an empty one forever, and an
// empty one is exactly what an unswept room looks like. A chunked copy that drops
// blocks at a chunk boundary gives a surface with holes, and a marching-cubes
// surface has holes anyway. A weight filter that excludes a block the mesher
// wanted changes the geometry, and the geometry is a room-shaped thing either
// way.
//
// None of it is reachable from a gate: `gates/mesh.sh` measures a running
// container, where all of these are working.

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "pimesh_mapping/mesh.hpp"
#include "pimesh_mapping/shared_volume.hpp"

using pimesh_mapping::SharedVolume;
using pimesh_mapping::TsdfVolume;
using pimesh_mapping::VolumeRegistry;
using pimesh_mapping::march_cubes;

namespace
{

constexpr int kWidth = 320;
constexpr int kHeight = 240;

cv::Matx33d test_k()
{
  return cv::Matx33d(250.0, 0.0, 160.0, 0.0, 250.0, 120.0, 0.0, 0.0, 1.0);
}

cv::Mat plane_at(float z)
{
  return cv::Mat(kHeight, kWidth, CV_32FC1, cv::Scalar(z));
}

cv::Mat colour_of(int b, int g, int r)
{
  return cv::Mat(kHeight, kWidth, CV_8UC3, cv::Scalar(b, g, r));
}

/// A configured volume holding a plane, integrated `frames` times.
std::shared_ptr<SharedVolume> filled(int frames = 6, float depth = 1.5F)
{
  auto volume = std::make_shared<SharedVolume>();
  TsdfVolume::Options options;
  options.voxel_size_m = 0.015F;
  options.truncation_voxels = 4;
  options.min_weight = 3.0F;
  volume->configure(options);
  for (int i = 0; i < frames; ++i) {
    volume->with_volume(
      [&](TsdfVolume & v) {
        v.integrate(plane_at(depth), colour_of(40, 80, 160), test_k(), cv::Affine3d::Identity());
      });
    volume->note_integrated();
  }
  return volume;
}

/// A volume holding an old surface and a new one: a plane at 1.5 m seen `old`
/// times and a second at 2.2 m seen `recent` times.
///
/// **A fixture with one plane in it cannot exercise the weight filter at all**,
/// and the first version of this file had one — every block reached the same
/// weight, the filter excluded nothing, and the test failed on its own setup. Two
/// surfaces of different ages is what a real sweep produces and what the filter
/// exists for: the blocks around the wall the camera has been staring at are
/// saturated, and the ones it has just glanced at are not.
std::shared_ptr<SharedVolume> filled_two_planes(int old_frames = 8, int recent_frames = 2)
{
  auto volume = filled(old_frames, 1.5F);
  for (int i = 0; i < recent_frames; ++i) {
    volume->with_volume(
      [](TsdfVolume & v) {
        v.integrate(plane_at(2.2F), colour_of(200, 30, 30), test_k(), cv::Affine3d::Identity());
      });
    volume->note_integrated();
  }
  return volume;
}

/// Are two block maps the same map?
bool same_blocks(const TsdfVolume::BlockMap & a, const TsdfVolume::BlockMap & b)
{
  if (a.size() != b.size()) {return false;}
  for (const auto & entry : a) {
    auto it = b.find(entry.first);
    if (it == b.end()) {return false;}
    for (int i = 0; i < TsdfVolume::kBlockVoxels; ++i) {
      const auto & x = entry.second.voxels[i];
      const auto & y = it->second.voxels[i];
      if (x.sdf != y.sdf || x.weight != y.weight ||
        x.r != y.r || x.g != y.g || x.b != y.b)
      {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

// --- The registry -------------------------------------------------------------

TEST(VolumeRegistry, TheSameKeyIsTheSameVolume)
{
  // **The whole rendezvous.** `fusion_node` fills the volume and `mesh_node`
  // meshes it, and they find each other by this name and nothing else. A registry
  // that constructed a fresh object per call would leave the mesher looking at an
  // empty volume for the life of the session — and an empty volume is exactly
  // what a room nobody has swept looks like, so there would be nothing to see.
  auto first = VolumeRegistry::get("test-same");
  auto second = VolumeRegistry::get("test-same");
  ASSERT_TRUE(first);
  EXPECT_EQ(first.get(), second.get());

  TsdfVolume::Options options;
  first->configure(options);
  EXPECT_TRUE(second->configured()) << "the two names did not reach one object";
}

TEST(VolumeRegistry, DifferentKeysAreDifferentVolumes)
{
  // The other half, and the reason the key is a parameter on both nodes rather
  // than a constant: a mismatch has to be *possible* for the check in the node to
  // be worth anything, and it has to keep the two maps apart when it happens.
  auto a = VolumeRegistry::get("test-a");
  auto b = VolumeRegistry::get("test-b");
  EXPECT_NE(a.get(), b.get());

  TsdfVolume::Options options;
  a->configure(options);
  EXPECT_FALSE(b->configured())
    << "configuring one key configured another — a mesh_node on the wrong key "
       "would silently mesh somebody else's map";
}

TEST(VolumeRegistry, KeysNamesEveryVolumeAndSaysWhetherItIsConfigured)
{
  // The diagnostic `mesh_node` prints when the volume it is waiting for never
  // appears. It is the only thing standing between a key typo and a mesher that
  // waits forever saying nothing useful.
  VolumeRegistry::get("test-listed-unconfigured");
  VolumeRegistry::get("test-listed-configured")->configure(TsdfVolume::Options());

  const std::string keys = VolumeRegistry::keys();
  EXPECT_NE(keys.find("test-listed-unconfigured (not configured)"), std::string::npos)
    << "keys(): " << keys;
  EXPECT_NE(keys.find("test-listed-configured (configured)"), std::string::npos)
    << "keys(): " << keys;
}

// --- Configuration ------------------------------------------------------------

TEST(SharedVolume, AnUnconfiguredVolumeIsEmptyRatherThanBroken)
{
  // `mesh_node` may load before `fusion_node` — component order is the launch
  // file's business, not this class's — so asking an unconfigured volume for a
  // snapshot has to be a normal thing that returns nothing, not a crash.
  SharedVolume volume;
  EXPECT_FALSE(volume.configured());
  const auto snapshot = volume.snapshot();
  EXPECT_TRUE(snapshot.blocks.empty());
  EXPECT_EQ(snapshot.frames_integrated, 0U);
  EXPECT_EQ(volume.reset(), 0U);
}

TEST(SharedVolume, ConfigureCarriesTheGeometryOntoEverySnapshot)
{
  // The mesher reads the voxel size and the weight floor *off the snapshot*,
  // because it never sees the volume. A snapshot that carried defaults instead
  // would place every vertex on a grid of the wrong pitch — a room-shaped thing
  // at the wrong scale, which is indistinguishable from the depth scale being
  // wrong and would send somebody to the tape measure.
  SharedVolume volume;
  TsdfVolume::Options options;
  options.voxel_size_m = 0.02F;
  options.truncation_voxels = 5;
  options.min_weight = 7.0F;
  volume.configure(options);

  const auto snapshot = volume.snapshot();
  EXPECT_FLOAT_EQ(snapshot.voxel_size_m, 0.02F);
  EXPECT_FLOAT_EQ(snapshot.truncation_m, 0.02F * 5.0F);
  EXPECT_FLOAT_EQ(snapshot.min_weight, 7.0F);
}

TEST(SharedVolume, FramesIntegratedCountsUpAndResetsWithTheMap)
{
  auto volume = filled(4);
  EXPECT_EQ(volume->frames_integrated(), 4U);
  EXPECT_EQ(volume->snapshot().frames_integrated, 4U);

  const std::size_t blocks = volume->with_volume(
    [](const TsdfVolume & v) {return v.block_count();});
  ASSERT_GT(blocks, 0U);

  EXPECT_EQ(volume->reset(), blocks) << "reset must report what it threw away";
  EXPECT_EQ(volume->frames_integrated(), 0U)
    << "the frame count survived a reset, so MeshStats would report a map that "
       "does not exist as having been built from a hundred frames";
  EXPECT_TRUE(volume->snapshot().blocks.empty());
}

// --- The chunked snapshot -----------------------------------------------------

TEST(SharedVolumeSnapshot, ChunkingDoesNotChangeWhatIsCopied)
{
  // **The lock is released between chunks, and the copy has to be the same copy
  // anyway.** An off-by-one at a chunk boundary drops blocks, and blocks dropped
  // from a marching-cubes surface are holes in a surface that has holes in it.
  // Nothing downstream would report it and no gate could see it.
  //
  // Chunk sizes chosen to straddle the block count: one at a time, a prime that
  // divides nothing, and one chunk larger than the whole map.
  auto volume = filled();
  const auto whole = volume->snapshot(0);
  ASSERT_GT(whole.blocks.size(), 20U) << "the fixture is too small to chunk";

  for (const std::size_t chunk : {std::size_t(1), std::size_t(7), std::size_t(13),
      whole.blocks.size(), whole.blocks.size() * 4})
  {
    const auto chunked = volume->snapshot(chunk);
    EXPECT_EQ(chunked.blocks.size(), whole.blocks.size()) << "chunk " << chunk;
    EXPECT_TRUE(same_blocks(chunked.blocks, whole.blocks)) << "chunk " << chunk;
  }
}

TEST(SharedVolumeSnapshot, TheCopyIsACopyAndNotAView)
{
  // The integrator goes on writing while the mesher marches. If the snapshot
  // shared storage with the volume, the mesher would read voxels changing under
  // it — and the result would be a surface that is *nearly* right, differently
  // each run, with no error anywhere.
  auto volume = filled(6, 1.5F);
  auto snapshot = volume->snapshot();
  ASSERT_FALSE(snapshot.blocks.empty());

  const std::int64_t key = snapshot.blocks.begin()->first;
  const float before = snapshot.blocks.at(key).voxels[0].sdf;

  for (int i = 0; i < 20; ++i) {
    volume->with_volume(
      [](TsdfVolume & v) {
        v.integrate(plane_at(1.53F), cv::Mat(), test_k(), cv::Affine3d::Identity());
      });
  }

  EXPECT_FLOAT_EQ(snapshot.blocks.at(key).voxels[0].sdf, before)
    << "the snapshot moved when the volume did — it is a view, not a copy";
}

TEST(SharedVolumeSnapshot, TheWeightFilterChangesTheMemoryAndNotTheMesh)
{
  // **The claim the filter is justified by, asserted rather than argued.** The
  // snapshot skips any block with no voxel at or above the meshing weight, on the
  // grounds that such a block has no corner the mesher would believe, so every
  // cell touching it is skipped either way. If that reasoning is wrong the
  // surface loses geometry — quietly, because a mesh with less of a room in it
  // still looks like a room.
  //
  // It saves real memory: on bags/desk1 it took the copy from 200 000 blocks to
  // 37 000, and the integrator had been measured stalling 400 ms while the mesher
  // allocated the difference.
  auto volume = filled_two_planes();
  const float weight = 6.0F;

  const auto unfiltered = volume->snapshot(2048, 0.0F);
  const auto filtered = volume->snapshot(2048, weight);

  ASSERT_FALSE(filtered.blocks.empty()) << "the filter removed everything";
  EXPECT_LT(filtered.blocks.size(), unfiltered.blocks.size())
    << "the filter excluded nothing, so it is buying no memory at all";

  const auto from_unfiltered = march_cubes(unfiltered, weight);
  const auto from_filtered = march_cubes(filtered, weight);

  ASSERT_GT(from_unfiltered.triangle_count(), 100U) << "the fixture meshes to nothing";
  EXPECT_EQ(from_filtered.triangle_count(), from_unfiltered.triangle_count());
  EXPECT_EQ(from_filtered.vertex_count(), from_unfiltered.vertex_count());
}

TEST(SharedVolumeSnapshot, ABlockTheFilterKeepsHasSomethingInItAboveTheWeight)
{
  auto volume = filled_two_planes();
  const float weight = 6.0F;
  const auto filtered = volume->snapshot(2048, weight);

  for (const auto & entry : filtered.blocks) {
    bool any = false;
    for (int i = 0; i < TsdfVolume::kBlockVoxels; ++i) {
      if (entry.second.voxels[i].weight >= weight) {
        any = true;
        break;
      }
    }
    EXPECT_TRUE(any) << "a block with nothing above the weight was copied anyway";
  }
}

TEST(SharedVolumeSnapshot, VoxelsAllocatedIsTheVolumeSCountAndNotTheFilteredCopySs)
{
  // MeshStats publishes `voxels_allocated` beside `voxels_meshed`, and **the gap
  // between them is the whole point of the pair** — it is the weight threshold
  // doing its job. Taking the first number off a copy that had already been
  // filtered by the second would close the gap by construction and report a
  // volume that meshes every voxel it holds, which is what a map full of noise
  // looks like.
  auto volume = filled_two_planes();
  const auto unfiltered = volume->snapshot(2048, 0.0F);
  const auto filtered = volume->snapshot(2048, 6.0F);

  EXPECT_EQ(filtered.voxels_allocated, unfiltered.voxels_allocated);
  EXPECT_GT(filtered.voxels_allocated, filtered.blocks.size() * TsdfVolume::kBlockVoxels)
    << "voxels_allocated came from the filtered copy";
}

// --- with_volume --------------------------------------------------------------

TEST(SharedVolume, WithVolumeHandsBackWhateverTheCallbackReturns)
{
  // The accessor is a callback rather than a lock()/unlock() pair for one reason:
  // a path that returns early while holding the mutex would block the mesher for
  // the rest of the session while the integrator went on looking perfectly
  // healthy. This is the shape working — a value out, the lock released.
  auto volume = filled(4);
  const std::size_t blocks = volume->with_volume(
    [](const TsdfVolume & v) {return v.block_count();});
  EXPECT_GT(blocks, 0U);

  // And it is re-entrant across calls, which the previous line would deadlock on
  // if the lock were not released on return.
  const float size = volume->with_volume([](const TsdfVolume & v) {return v.voxel_size();});
  EXPECT_FLOAT_EQ(size, 0.015F);
}
