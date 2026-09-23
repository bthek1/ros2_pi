#ifndef PIMESH_WORLD__SHARED_VOLUME_HPP_
#define PIMESH_WORLD__SHARED_VOLUME_HPP_

#include <algorithm>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "pimesh_mapping/tsdf_volume.hpp"

namespace pimesh_mapping
{

/// The TSDF, its lock, and the counters that describe it — one object, shared by
/// the node that fills it and the node that meshes it.
///
/// **Why this is shared memory rather than a topic.** `fusion_node` integrates at
/// the depth rate and `mesh_node` extracts a surface every ten seconds, and what
/// passes between them is the *whole volume* — tens of thousands of 6 kB blocks,
/// hundreds of megabytes. Publishing that would be a serialisation of the entire
/// map ten times a minute for a consumer sitting in the same process. This
/// workspace already answers that question everywhere else: the dev-box stages are
/// composed into one container precisely so a frame reaches its consumers as a
/// pointer, and this is the same rule one size up.
///
/// **The consequence, stated plainly: `mesh_node` does not work on its own.** It
/// needs a `fusion_node` in the same process to have created the volume it meshes,
/// and started alone it refuses with a message saying so, the way `depth_node`
/// does on a machine with no ONNX Runtime. That is a real limitation and it is the
/// architecture doing what it says: `pipeline:=false` leaves the container out
/// entirely and there is no half-pipeline in between.
///
/// **The lock discipline is the whole of P6's "no dip at mesh time".** Integration
/// holds the mutex for the ~15 ms it takes; the mesher takes a **copy** of the
/// block map under the same mutex and then marches cubes over the copy with the
/// lock released. Meshing under the lock would stall the integrator for the
/// hundreds of milliseconds marching cubes takes, and `tools/gates/mesh.sh`
/// asserts the integration rate shows no dip for exactly that reason.
class SharedVolume
{
public:
  /// Configure the volume. Called once, by `fusion_node`, before anything
  /// integrates. `mesh_node` may have found this object first and waits on
  /// `configured()`.
  void configure(const TsdfVolume::Options & options)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    volume_ = std::make_unique<TsdfVolume>(options);
    configured_ = true;
    frames_integrated_ = 0;
  }

  bool configured() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return configured_;
  }

  /// Run `fn(TsdfVolume &)` with the volume locked for writing.
  ///
  /// A callback rather than a `lock()`/`unlock()` pair because the one thing that
  /// must never happen here is a path that returns early while holding the mutex:
  /// the mesher would then block for the rest of the session and the integrator
  /// would look perfectly healthy.
  template<typename Fn>
  auto with_volume(Fn && fn) -> decltype(fn(std::declval<TsdfVolume &>()))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return fn(*volume_);
  }

  template<typename Fn>
  auto with_volume(Fn && fn) const -> decltype(fn(std::declval<const TsdfVolume &>()))
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return fn(*volume_);
  }

  /// A copy of the block map and the geometry needed to read it, taken under the
  /// lock and returned by value.
  ///
  /// This is the short lock the class comment is about. The copy is a `memcpy` of
  /// the hash map's buckets — measured in tens of milliseconds for a room — where
  /// marching cubes over the same data is measured in hundreds.
  struct Snapshot
  {
    TsdfVolume::BlockMap blocks;
    float voxel_size_m {0.015F};
    float truncation_m {0.06F};
    float min_weight {3.0F};
    std::uint64_t frames_integrated {0};
    std::size_t voxels_allocated {0};
  };

  /// Copy the map, `chunk` blocks at a time, releasing the lock between chunks.
  ///
  /// **One lock over the whole copy is not a short lock at this size.** A room on
  /// bags/desk1 is ~205 000 blocks, which is 1.3 GB, and a memcpy of that holds
  /// the integrator out for well over a hundred milliseconds — two frames gone,
  /// and visible as exactly the dip in the integration rate that P6's gate
  /// asserts is absent. Chunked, each acquisition is a few milliseconds and the
  /// integrator interleaves with it.
  ///
  /// **The price is that the copy spans two instants**, and it is worth saying
  /// plainly rather than discovering: blocks copied early are a fraction of a
  /// second older than blocks copied late. For a surface that moves by a
  /// millimetre or two per frame — a weighted average with weight already in the
  /// tens — that is invisible, and it is the right trade for not stalling the
  /// stage the whole pipeline feeds.
  ///
  /// The key list is taken first, under its own lock, because `unordered_map`
  /// rehashes on insert and an iterator held across an unlock is a dangling one.
  /// A block added between chunks is simply not in this snapshot; the next
  /// re-mesh gets it.
  /// `min_weight` filters the copy: a block with no voxel at or above it is not
  /// copied at all.
  ///
  /// **This changes nothing about the mesh and a great deal about the memory.** A
  /// block nothing has reached that weight in has no corner the mesher would
  /// believe, so every cell touching it is skipped either way — filtering here and
  /// filtering in `march_cubes` produce the same surface. What it saves is the
  /// copy: on bags/desk1 the volume is ~200 000 blocks and 1.25 GB, the snapshot
  /// doubles that, and this machine had 5 GB free. The integrator was measured
  /// stalling 400 ms against a 56 ms median while the mesher allocated — which is
  /// the very dip P6's gate exists to assert is absent, arriving through memory
  /// pressure rather than through the lock everyone expects it from.
  Snapshot snapshot(std::size_t chunk = 2048, float min_weight = 0.0F) const
  {
    Snapshot out;
    std::vector<std::int64_t> keys;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!configured_) {return out;}
      out.voxel_size_m = volume_->voxel_size();
      out.truncation_m = volume_->truncation_m();
      out.min_weight = volume_->min_weight();
      out.frames_integrated = frames_integrated_;
      out.voxels_allocated = volume_->voxels_allocated();
      keys.reserve(volume_->blocks().size());
      for (const auto & entry : volume_->blocks()) {keys.push_back(entry.first);}
    }

    // Reserved for the worst case rather than grown by rehashing: a rehash of a
    // map holding hundreds of thousands of 6 kB nodes is a second copy of the
    // whole thing, taken while the integrator is trying to allocate.
    out.blocks.reserve(keys.size());
    if (chunk == 0) {chunk = keys.size() ? keys.size() : 1;}
    for (std::size_t start = 0; start < keys.size(); start += chunk) {
      const std::size_t end = std::min(keys.size(), start + chunk);
      std::lock_guard<std::mutex> lock(mutex_);
      const auto & blocks = volume_->blocks();
      for (std::size_t i = start; i < end; ++i) {
        auto it = blocks.find(keys[i]);
        if (it == blocks.end()) {continue;}
        if (min_weight > 0.0F) {
          bool wanted = false;
          for (int v = 0; v < TsdfVolume::kBlockVoxels; ++v) {
            if (it->second.voxels[v].weight >= min_weight) {
              wanted = true;
              break;
            }
          }
          if (!wanted) {continue;}
        }
        out.blocks.emplace(it->first, it->second);
      }
    }
    return out;
  }

  void note_integrated()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++frames_integrated_;
  }

  std::uint64_t frames_integrated() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return frames_integrated_;
  }

  /// Throw the map away. `/world/reset_map` and nothing else.
  std::size_t reset()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {return 0;}
    const std::size_t blocks = volume_->block_count();
    volume_->clear();
    frames_integrated_ = 0;
    return blocks;
  }

private:
  mutable std::mutex mutex_;
  std::unique_ptr<TsdfVolume> volume_ {std::make_unique<TsdfVolume>()};
  bool configured_ {false};
  std::uint64_t frames_integrated_ {0};
};

/// Process-local rendezvous between `fusion_node` and `mesh_node`.
///
/// Keyed by a string so that two pipelines composed into one container — which
/// nothing here does today, and which a test does — get two volumes rather than
/// silently sharing one. The key is a parameter on both nodes and defaults to
/// `world`; a mismatch is the failure this would otherwise hide, so `mesh_node`
/// says which key it is waiting on when it refuses.
///
/// **Process-local, and that is the sharp edge.** Two containers means two
/// volumes, and a `mesh_node` in the wrong container meshes an empty one forever
/// without erroring. `tools/gates/mesh.sh` asserts a triangle count for that
/// reason rather than merely asserting the topic exists.
class VolumeRegistry
{
public:
  static std::shared_ptr<SharedVolume> get(const std::string & key);
  /// Every key currently registered, for the diagnostic `mesh_node` prints when
  /// the volume it is waiting for never becomes configured.
  static std::string keys();
};

}  // namespace pimesh_mapping

#endif  // PIMESH_WORLD__SHARED_VOLUME_HPP_
