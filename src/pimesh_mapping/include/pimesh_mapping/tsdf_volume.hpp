#ifndef PIMESH_WORLD__TSDF_VOLUME_HPP_
#define PIMESH_WORLD__TSDF_VOLUME_HPP_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "opencv2/core.hpp"
#include "opencv2/core/affine.hpp"

namespace pimesh_mapping
{

/// A spatially hashed truncated signed distance volume: the pipeline's
/// accumulator, and the reason a mesh of a noisy monocular depth stream is not
/// just a noisy mesh.
///
/// **What a TSDF actually is, because the name hides it.** Every voxel holds a
/// running *weighted average* of its signed distance to the nearest observed
/// surface — negative behind the surface, positive in front of it, clipped to a
/// truncation band a few voxels thick and useless outside it. The surface is
/// wherever that average crosses zero. Averaging is the whole mechanism: a
/// monocular depth map wobbles by several centimetres frame to frame, and a
/// hundred frames averaged at one voxel converge on the wall while a single frame
/// would have put it anywhere in a 10 cm band. Marching cubes (P6) then reads the
/// zero crossing out; it does no smoothing of its own and does not need to.
///
/// **Hashed, not dense, and that is not an optimisation.** A dense 6 m cube at
/// 15 mm voxels is 64 million voxels — 768 MB before a single frame arrives, for a
/// room that is almost entirely air. Blocks of 8³ voxels are allocated only where
/// a depth measurement has put a surface, so the memory tracks the *observed*
/// surface area rather than the volume of the room. `voxels_allocated()` in
/// MeshStats is that number made visible.
///
/// **No ROS, no GPU, no OpenCV beyond `Mat` and `Affine3d` on purpose.** Every
/// mistake available in this file is a silent one — an off-by-one in the block
/// grid, a sign flip on the signed distance, a projection that uses ray length
/// where the depth image holds z — and each produces a volume that meshes into a
/// plausible-looking room that is wrong. That is exactly the category
/// `test_tsdf_volume` exists for, and it can only exist if this class can be
/// constructed without a container around it.
class TsdfVolume
{
public:
  /// 8, and the block size follows from it: 8 x 15 mm = 12 cm.
  ///
  /// The trade is between hash traffic and wasted voxels. Bigger blocks mean
  /// fewer hash lookups per frame and more voxels allocated far from any surface;
  /// smaller blocks mean the reverse. 8 is the number every voxel-hashing paper
  /// since Nießner 2013 uses, and 512 voxels at 12 bytes is a 6 kB block — one
  /// that a single `memcpy` moves and that the snapshot copy in P6 can afford.
  static constexpr int kBlockSide = 8;
  static constexpr int kBlockVoxels = kBlockSide * kBlockSide * kBlockSide;

  /// One voxel: the running average, its weight, and the colour that goes with it.
  ///
  /// 12 bytes, and the layout is deliberate — `sdf` and `weight` adjacent so the
  /// hot loop touches one cache line per voxel, colour after them because the
  /// mesher reads it and the integrator barely does. **`sdf` is stored normalised
  /// by the truncation distance**, so it lives in [-1, 1] and a voxel's stored
  /// value means the same thing whatever `voxel_size_m` is set to. Storing metres
  /// here instead would make every threshold in the mesher depend on the
  /// resolution.
  struct Voxel
  {
    float sdf {0.0F};
    float weight {0.0F};
    std::uint8_t b {0};
    std::uint8_t g {0};
    std::uint8_t r {0};
    std::uint8_t pad {0};
  };

  struct Block
  {
    Voxel voxels[kBlockVoxels];
  };

  struct Options
  {
    /// 15 mm, from the plan. Small enough that a desk edge survives and large
    /// enough that a room fits in memory at this block size.
    float voxel_size_m {0.015F};
    /// 4 voxels = 6 cm of band either side of the surface. The band has to be
    /// wider than the depth noise it is averaging or observations of the same
    /// wall never meet in the same voxel; it also must not be so wide that the
    /// front and back of a thin object share voxels.
    int truncation_voxels {4};
    /// A voxel is not meshed until this many observations agree about it. The
    /// noise floor: a single frame's flying pixel at an object edge allocates a
    /// voxel with weight 1, and meshing that produces the fringe of debris this
    /// threshold exists to keep out of the surface. The gap between
    /// `voxels_allocated` and `voxels_meshed` is this doing its job.
    float min_weight {3.0F};
    /// The running average saturates here rather than growing without bound, so
    /// the volume can still *change* after a thousand frames. An unbounded weight
    /// is a volume that stops listening — which looks fine until the room does
    /// something, or until P7 moves a pose and the old surface has to give way.
    float max_weight {64.0F};
    /// Beyond this a depth reading is the model's "far away or no idea" clip
    /// (depth_node's `max_range_m`) and carries no information about a surface.
    /// Integrating it would build a shell of the clip plane.
    float max_range_m {6.0F};
    /// Depth readings below this are the model's near failure, not geometry.
    float min_range_m {0.15F};
    /// A hard ceiling on the map, in blocks. At 6 kB a block, 300 000 is about
    /// 1.9 GB — and P6's mesher takes a *copy* of it, so the real figure to hold
    /// in mind is twice that. Past the ceiling, existing blocks keep updating and
    /// no new ones are created; `blocks_refused` counts what was turned away so
    /// the node can say so rather than quietly stopping mapping.
    ///
    /// **It is reached on bags/desk1 today, and that is a symptom rather than a
    /// resolution being too fine.** 145 000 blocks after 40 s is 2090 m² of
    /// surface for a room with perhaps 60 m² in it — about thirty layers of
    /// shingles, laid down by three things this phase does not fix: rotation-only
    /// odometry with no translation (P7), an unpinned `depth_scale` (a tape
    /// measure and a person), and a per-frame scale wobble on a sweeping clip
    /// well past the aligner's 15% clamp. A ceiling is not a fix for any of them;
    /// it is what stops the session dying of memory while they are outstanding.
    std::size_t max_blocks {300000};
    /// Every 8th pixel in each direction decides which blocks exist. At
    /// fx = 953 and 6 m that is 5 cm between samples, well inside a 12 cm block,
    /// so nothing in the frustum is missed — and it is 64x less hash traffic
    /// than allocating from every pixel. **The update pass that follows reads
    /// every pixel**; this stride governs allocation only.
    int allocation_stride {8};
  };

  /// Two constructors rather than one with a default argument: `Options` is a
  /// nested class, and a default argument of `Options()` inside this class body
  /// needs its default member initializers before the enclosing class is
  /// complete, which the standard does not give you.
  TsdfVolume();
  explicit TsdfVolume(const Options & options);

  struct IntegrateResult
  {
    std::size_t blocks_touched {0};
    std::size_t blocks_new {0};
    /// Blocks the frame wanted and could not have, because `max_blocks` is full.
    std::size_t blocks_refused {0};
    std::size_t voxels_updated {0};
    std::size_t pixels_valid {0};
  };

  /// Fold one posed depth map into the volume.
  ///
  /// `depth_m` is CV_32FC1 metres as `/depth` carries them, `bgr` the frame it
  /// was inferred on (empty is allowed — the volume then keeps its colours), `k`
  /// the intrinsics for that resolution and `world_from_camera` the pose of
  /// `camera_optical_frame` in the map frame.
  ///
  /// **The signed distance is projective, and it uses z rather than ray length.**
  /// `sdf = depth(u, v) - z_voxel`, where both are the *perpendicular* distance
  /// from the image plane, because that is what a 32FC1 depth image holds. Using
  /// `|p|` instead is the classic silent error here: it agrees perfectly at the
  /// principal point and is 30% wrong in the frame corners, which bends every
  /// wall into a bowl that looks like lens distortion and is not.
  IntegrateResult integrate(
    const cv::Mat & depth_m, const cv::Mat & bgr,
    const cv::Matx33d & k, const cv::Affine3d & world_from_camera);

  /// Render the depth the volume *expects* to see from this pose.
  ///
  /// This is the instrument the scale aligner measures against, and it is a
  /// ray-cast rather than a re-projection because the volume has no depth map in
  /// it to re-project — only distances. Each ray marches out from the camera
  /// until the interpolated sdf changes sign from positive (in front of the
  /// surface) to negative (behind it), and the crossing is interpolated linearly
  /// between the two samples.
  ///
  /// Output is CV_32FC1 of `size`, in metres, **z rather than ray length** so it
  /// can be compared with an incoming depth map pixel for pixel. 0 means the ray
  /// found nothing — either unobserved space or a voxel that has not reached
  /// `min_weight`. Zero is the honest answer and the aligner treats it as invalid.
  ///
  /// `k` must be scaled to `size` by the caller; this function does no resizing,
  /// because an intrinsics matrix silently applied at the wrong resolution is a
  /// scale error of exactly the ratio, and it looks like a depth scale error.
  void raycast(
    const cv::Matx33d & k, const cv::Affine3d & world_from_camera,
    cv::Size size, cv::Mat & expected_m) const;

  /// Distance from the camera to the surface each ray hits, in metres, or 0.
  ///
  /// Same march as `raycast` for one direction only, in the *camera* frame. Used
  /// by the paired-surface check, where the question is about one ray and
  /// building a whole image would be noise around the answer.
  float raycast_ray(
    const cv::Affine3d & world_from_camera, const cv::Vec3f & direction_camera) const;

  /// Voxel-grid lookup of the stored (normalised) distance at a world point.
  ///
  /// Returns false where no block is allocated or the voxel is under
  /// `min_weight` — "I have not seen this" and "I have seen it once and do not
  /// believe it" are the same answer to a ray-caster and a mesher.
  bool sample(const cv::Vec3f & world_point, float & sdf, float & weight) const;

  void clear();

  std::size_t block_count() const {return blocks_.size();}
  /// Allocated, and of those how many carry enough weight to be meshed. Both go
  /// into MeshStats; the gap between them is the weight threshold working.
  std::size_t voxels_allocated() const {return blocks_.size() * kBlockVoxels;}
  std::size_t voxels_above_weight() const;

  float voxel_size() const {return options_.voxel_size_m;}
  float block_size() const {return options_.voxel_size_m * kBlockSide;}
  float truncation_m() const {return truncation_m_;}
  float min_weight() const {return options_.min_weight;}
  const Options & options() const {return options_;}

  /// The hash map itself, for the mesher.
  ///
  /// Exposed rather than wrapped in an iterator because P6's `mesh_node` takes a
  /// **copy** of the whole map under a short lock and meshes the copy without
  /// holding it — the entire reason the mesh does not stall the integrator — and
  /// a copy needs the container, not a view of it. The key packs the block
  /// coordinate; `key_to_block` unpacks it.
  using BlockMap = std::unordered_map<std::int64_t, Block>;
  const BlockMap & blocks() const {return blocks_;}

  /// Pack and unpack a block coordinate into the hash key.
  ///
  /// 21 bits per axis, signed, which is ±1 048 576 blocks — ±125 km at this block
  /// size. The alternative, a struct key with a hand-written hash, costs a
  /// `std::hash` specialisation and an equality operator to say the same thing.
  static std::int64_t block_key(int bx, int by, int bz);
  static void key_to_block(std::int64_t key, int & bx, int & by, int & bz);

  /// World position of a voxel's centre, given its block key and index within it.
  cv::Vec3f voxel_centre(std::int64_t key, int index) const;

private:
  Block * block_at(int bx, int by, int bz, bool create, bool * created = nullptr);
  const Block * find_block(int bx, int by, int bz) const;

  Options options_;
  float truncation_m_ {0.06F};
  float inv_truncation_ {1.0F / 0.06F};
  BlockMap blocks_;

  /// Reused between frames so the hot path allocates nothing: the block keys this
  /// frame's allocation pass touched. A `vector` plus a sort-unique rather than a
  /// `set`, because the pass produces ~70 000 keys of which ~2000 are distinct and
  /// a node-based container pays an allocation for every one of them.
  std::vector<std::int64_t> touched_;
};

}  // namespace pimesh_mapping

#endif  // PIMESH_WORLD__TSDF_VOLUME_HPP_
