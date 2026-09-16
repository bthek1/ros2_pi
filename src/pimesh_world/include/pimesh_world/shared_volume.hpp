#ifndef PIMESH_WORLD__SHARED_VOLUME_HPP_
#define PIMESH_WORLD__SHARED_VOLUME_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "pimesh_world/tsdf_volume.hpp"

namespace pimesh_world
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

  Snapshot snapshot() const
  {
    Snapshot out;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {return out;}
    out.blocks = volume_->blocks();
    out.voxel_size_m = volume_->voxel_size();
    out.truncation_m = volume_->truncation_m();
    out.min_weight = volume_->min_weight();
    out.frames_integrated = frames_integrated_;
    out.voxels_allocated = volume_->voxels_allocated();
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

}  // namespace pimesh_world

#endif  // PIMESH_WORLD__SHARED_VOLUME_HPP_
