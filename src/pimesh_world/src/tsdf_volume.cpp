// The accumulator. Two passes per frame — decide which blocks exist, then update
// the voxels in them — and a ray-caster that reads the surface back out.
//
// Everything here is float and the inner loops are written out rather than
// expressed with cv::Matx operations, for one measured reason: this runs at the
// depth rate against a 20 ms budget with ~1.5 million voxel updates in it, and a
// `Matx31f` constructed per voxel is a constructor call per voxel. The
// incremental form below computes a voxel's camera-space position with three
// adds, because a block's voxel centres are a regular grid and a rigid transform
// of a regular grid is still one.

#include "pimesh_world/tsdf_volume.hpp"

#include <algorithm>
#include <cmath>

namespace pimesh_world
{
namespace
{

constexpr int kBits = 21;
constexpr std::int64_t kMask = (static_cast<std::int64_t>(1) << kBits) - 1;

/// Floor division towards minus infinity.
///
/// **Not `int(x / s)`**, which truncates towards zero and therefore makes the
/// blocks either side of the origin share a coordinate — a one-block seam through
/// x=0, y=0 and z=0 where two different regions of the room write into the same
/// voxels. The map frame's origin is wherever the camera started, so that seam
/// lands in the middle of the room rather than safely outside it.
inline int floor_div(float value, float step)
{
  return static_cast<int>(std::floor(value / step));
}

}  // namespace

TsdfVolume::TsdfVolume()
: TsdfVolume(Options())
{
}

TsdfVolume::TsdfVolume(const Options & options)
: options_(options)
{
  truncation_m_ = options_.voxel_size_m * static_cast<float>(options_.truncation_voxels);
  inv_truncation_ = 1.0F / truncation_m_;
}

std::int64_t TsdfVolume::block_key(int bx, int by, int bz)
{
  return (static_cast<std::int64_t>(bx) & kMask) |
         ((static_cast<std::int64_t>(by) & kMask) << kBits) |
         ((static_cast<std::int64_t>(bz) & kMask) << (2 * kBits));
}

void TsdfVolume::key_to_block(std::int64_t key, int & bx, int & by, int & bz)
{
  // Sign-extend each 21-bit field: the shift up and arithmetic shift back is the
  // portable spelling, and it is why the fields are packed low-to-high.
  auto extend = [](std::int64_t field) {
      constexpr std::int64_t sign = static_cast<std::int64_t>(1) << (kBits - 1);
      return static_cast<int>((field ^ sign) - sign);
    };
  bx = extend(key & kMask);
  by = extend((key >> kBits) & kMask);
  bz = extend((key >> (2 * kBits)) & kMask);
}

TsdfVolume::Block * TsdfVolume::block_at(int bx, int by, int bz, bool create, bool * created)
{
  const std::int64_t key = block_key(bx, by, bz);
  auto it = blocks_.find(key);
  if (it != blocks_.end()) {
    if (created) {*created = false;}
    return &it->second;
  }
  if (!create) {return nullptr;}
  if (created) {*created = true;}
  return &blocks_.emplace(key, Block{}).first->second;
}

const TsdfVolume::Block * TsdfVolume::find_block(int bx, int by, int bz) const
{
  auto it = blocks_.find(block_key(bx, by, bz));
  return it == blocks_.end() ? nullptr : &it->second;
}

cv::Vec3f TsdfVolume::voxel_centre(std::int64_t key, int index) const
{
  int bx = 0;
  int by = 0;
  int bz = 0;
  key_to_block(key, bx, by, bz);
  const int i = index % kBlockSide;
  const int j = (index / kBlockSide) % kBlockSide;
  const int k = index / (kBlockSide * kBlockSide);
  const float vs = options_.voxel_size_m;
  const float bs = vs * kBlockSide;
  return cv::Vec3f(
    static_cast<float>(bx) * bs + (static_cast<float>(i) + 0.5F) * vs,
    static_cast<float>(by) * bs + (static_cast<float>(j) + 0.5F) * vs,
    static_cast<float>(bz) * bs + (static_cast<float>(k) + 0.5F) * vs);
}

void TsdfVolume::clear()
{
  blocks_.clear();
}

std::size_t TsdfVolume::voxels_above_weight() const
{
  std::size_t count = 0;
  for (const auto & entry : blocks_) {
    for (int i = 0; i < kBlockVoxels; ++i) {
      if (entry.second.voxels[i].weight >= options_.min_weight) {++count;}
    }
  }
  return count;
}

bool TsdfVolume::sample(const cv::Vec3f & world_point, float & sdf, float & weight) const
{
  const float vs = options_.voxel_size_m;
  const float bs = vs * kBlockSide;
  const int bx = floor_div(world_point[0], bs);
  const int by = floor_div(world_point[1], bs);
  const int bz = floor_div(world_point[2], bs);
  const Block * block = find_block(bx, by, bz);
  if (!block) {return false;}

  // Index within the block, from the voxel coordinate rather than from a second
  // division of the world point: `(vx - bx * side)` is exact, where
  // `floor((p - block_origin) / vs)` can land on `side` at a block boundary
  // through float rounding and index one past the end.
  const int vx = floor_div(world_point[0], vs) - bx * kBlockSide;
  const int vy = floor_div(world_point[1], vs) - by * kBlockSide;
  const int vz = floor_div(world_point[2], vs) - bz * kBlockSide;
  if (vx < 0 || vx >= kBlockSide || vy < 0 || vy >= kBlockSide ||
    vz < 0 || vz >= kBlockSide)
  {
    return false;
  }
  const Voxel & voxel = block->voxels[(vz * kBlockSide + vy) * kBlockSide + vx];
  if (voxel.weight < options_.min_weight) {return false;}
  sdf = voxel.sdf;
  weight = voxel.weight;
  return true;
}

TsdfVolume::IntegrateResult TsdfVolume::integrate(
  const cv::Mat & depth_m, const cv::Mat & bgr,
  const cv::Matx33d & k, const cv::Affine3d & world_from_camera)
{
  IntegrateResult result;
  if (depth_m.empty() || depth_m.type() != CV_32FC1) {return result;}
  const bool have_colour = !bgr.empty() && bgr.type() == CV_8UC3 && bgr.size() == depth_m.size();

  const int width = depth_m.cols;
  const int height = depth_m.rows;
  const float fx = static_cast<float>(k(0, 0));
  const float fy = static_cast<float>(k(1, 1));
  const float cx = static_cast<float>(k(0, 2));
  const float cy = static_cast<float>(k(1, 2));
  if (!(fx > 0.0F) || !(fy > 0.0F)) {return result;}

  const float vs = options_.voxel_size_m;
  const float bs = vs * kBlockSide;
  const float trunc = truncation_m_;

  const cv::Matx33f rotation(
    static_cast<float>(world_from_camera.rotation()(0, 0)),
    static_cast<float>(world_from_camera.rotation()(0, 1)),
    static_cast<float>(world_from_camera.rotation()(0, 2)),
    static_cast<float>(world_from_camera.rotation()(1, 0)),
    static_cast<float>(world_from_camera.rotation()(1, 1)),
    static_cast<float>(world_from_camera.rotation()(1, 2)),
    static_cast<float>(world_from_camera.rotation()(2, 0)),
    static_cast<float>(world_from_camera.rotation()(2, 1)),
    static_cast<float>(world_from_camera.rotation()(2, 2)));
  const cv::Vec3f translation(
    static_cast<float>(world_from_camera.translation()[0]),
    static_cast<float>(world_from_camera.translation()[1]),
    static_cast<float>(world_from_camera.translation()[2]));

  // --- Pass one: which blocks does this frame have anything to say about? -----
  //
  // A band along each sampled ray, from one truncation in front of the surface to
  // one behind it. Sampling the band rather than only the surface voxel is what
  // gives the later pass somewhere to write the *positive* side of the signed
  // distance — a volume allocated only at the surface has no free space in it and
  // marching cubes has no sign change to find.
  touched_.clear();
  const int stride = std::max(1, options_.allocation_stride);
  // Half a truncation: the band is 2 * trunc long, so this is 5 samples, and no
  // gap in it can exceed half a block.
  const float band_step = std::max(trunc * 0.5F, bs * 0.4F);

  for (int v = 0; v < height; v += stride) {
    const float * row = depth_m.ptr<float>(v);
    const float ny = (static_cast<float>(v) - cy) / fy;
    for (int u = 0; u < width; u += stride) {
      const float d = row[u];
      if (!std::isfinite(d) || d < options_.min_range_m || d > options_.max_range_m) {continue;}
      ++result.pixels_valid;
      const float nx = (static_cast<float>(u) - cx) / fx;
      // z = t by construction, because the ray's third component is 1. That is
      // the same convention the depth image itself uses and keeping them the same
      // is what makes `sdf = d - z` below correct.
      for (float t = d - trunc; t <= d + trunc + 1e-6F; t += band_step) {
        if (t <= 0.0F) {continue;}
        const cv::Vec3f p_cam(nx * t, ny * t, t);
        const cv::Vec3f p = rotation * p_cam + translation;
        touched_.push_back(
          block_key(floor_div(p[0], bs), floor_div(p[1], bs), floor_div(p[2], bs)));
      }
    }
  }

  std::sort(touched_.begin(), touched_.end());
  touched_.erase(std::unique(touched_.begin(), touched_.end()), touched_.end());
  result.blocks_touched = touched_.size();

  // --- Pass two: update every voxel of every touched block --------------------
  //
  // The camera-from-world transform, used incrementally. A block's 512 voxel
  // centres are a regular grid; a rigid transform of one is a regular grid too,
  // so the position of voxel (i, j, k) in the camera frame is
  // `base + i*dx + j*dy + k*dz` and the inner loop is three adds rather than a
  // matrix multiply.
  const cv::Matx33f rot_inv = rotation.t();
  const cv::Vec3f trans_inv = -(rot_inv * translation);
  const cv::Vec3f dx(rot_inv(0, 0) * vs, rot_inv(1, 0) * vs, rot_inv(2, 0) * vs);
  const cv::Vec3f dy(rot_inv(0, 1) * vs, rot_inv(1, 1) * vs, rot_inv(2, 1) * vs);
  const cv::Vec3f dz(rot_inv(0, 2) * vs, rot_inv(1, 2) * vs, rot_inv(2, 2) * vs);

  // **A block is only inserted once a voxel in it is actually written.** The
  // allocation pass is deliberately generous — it samples a band around a ray and
  // rounds outwards — so some of what it names turns out to have no valid depth
  // behind it. Inserting those anyway would inflate `voxels_allocated`, which is
  // a number MeshStats publishes and a person reads as "how much room have I
  // seen". Staging into one reused buffer costs a 6 kB copy per genuinely new
  // block and nothing per existing one.
  Block staging;

  for (const std::int64_t key : touched_) {
    int bx = 0;
    int by = 0;
    int bz = 0;
    key_to_block(key, bx, by, bz);

    auto it = blocks_.find(key);
    const bool is_new = it == blocks_.end();
    if (is_new && blocks_.size() >= options_.max_blocks) {
      // Full. Existing blocks keep being updated — the parts of the room already
      // mapped stay live — and nothing new is taken on. Counted rather than
      // silent: a map that has quietly stopped growing and a camera pointed at a
      // wall look identical from outside.
      ++result.blocks_refused;
      continue;
    }
    Block * block = nullptr;
    if (is_new) {
      staging = Block{};
      block = &staging;
    } else {
      block = &it->second;
    }

    const cv::Vec3f origin(
      static_cast<float>(bx) * bs + 0.5F * vs,
      static_cast<float>(by) * bs + 0.5F * vs,
      static_cast<float>(bz) * bs + 0.5F * vs);
    const cv::Vec3f base = rot_inv * origin + trans_inv;

    std::size_t written = 0;
    for (int kz = 0; kz < kBlockSide; ++kz) {
      const cv::Vec3f row_z = base + dz * static_cast<float>(kz);
      for (int jy = 0; jy < kBlockSide; ++jy) {
        cv::Vec3f p = row_z + dy * static_cast<float>(jy);
        Voxel * voxels = &block->voxels[(kz * kBlockSide + jy) * kBlockSide];
        for (int ix = 0; ix < kBlockSide; ++ix, p += dx) {
          const float z = p[2];
          if (z <= 0.0F) {continue;}
          const float inv_z = 1.0F / z;
          const int u = static_cast<int>(fx * p[0] * inv_z + cx + 0.5F);
          if (u < 0 || u >= width) {continue;}
          const int v = static_cast<int>(fy * p[1] * inv_z + cy + 0.5F);
          if (v < 0 || v >= height) {continue;}

          const float d = depth_m.at<float>(v, u);
          if (!std::isfinite(d) || d < options_.min_range_m || d > options_.max_range_m) {
            continue;
          }

          // Projective signed distance, in metres, along z. Positive is in front
          // of the surface (between the camera and the wall), negative behind it.
          const float sdf = d - z;
          // **Behind the surface by more than the truncation: say nothing.** Not
          // "empty", not "occupied" — this voxel is occluded, and a camera has no
          // opinion about what is behind a wall. Writing free space here is what
          // erodes a surface from the far side over a long sweep.
          if (sdf < -trunc) {continue;}

          const float normalised = std::min(1.0F, sdf * inv_truncation_);

          Voxel & voxel = voxels[ix];
          const float w = voxel.weight;
          const float w_new = std::min(options_.max_weight, w + 1.0F);
          // The running weighted average, in the incremental form. The new sample
          // carries weight 1 whatever its distance: a confidence that falls off
          // with range is a refinement this has not measured and would silently
          // change every number below.
          voxel.sdf = (voxel.sdf * w + normalised) / (w + 1.0F);
          voxel.weight = w_new;

          if (have_colour) {
            const cv::Vec3b & c = bgr.at<cv::Vec3b>(v, u);
            // Colour is averaged with the same weight as the distance, so a voxel
            // seen once under a hand's shadow is outvoted by the frames that saw
            // it properly. Rounded rather than truncated: repeated truncation of a
            // running average walks a grey wall darker over a thousand frames.
            voxel.b = static_cast<std::uint8_t>(
              (static_cast<float>(voxel.b) * w + static_cast<float>(c[0])) / (w + 1.0F) + 0.5F);
            voxel.g = static_cast<std::uint8_t>(
              (static_cast<float>(voxel.g) * w + static_cast<float>(c[1])) / (w + 1.0F) + 0.5F);
            voxel.r = static_cast<std::uint8_t>(
              (static_cast<float>(voxel.r) * w + static_cast<float>(c[2])) / (w + 1.0F) + 0.5F);
          }
          ++written;
        }
      }
    }

    result.voxels_updated += written;
    if (is_new && written > 0) {
      blocks_.emplace(key, staging);
      ++result.blocks_new;
    }
  }

  return result;
}

float TsdfVolume::raycast_ray(
  const cv::Affine3d & world_from_camera, const cv::Vec3f & direction_camera) const
{
  if (blocks_.empty()) {return 0.0F;}
  const cv::Matx33d & r = world_from_camera.rotation();
  const cv::Vec3f origin(
    static_cast<float>(world_from_camera.translation()[0]),
    static_cast<float>(world_from_camera.translation()[1]),
    static_cast<float>(world_from_camera.translation()[2]));
  const cv::Vec3f dir(
    static_cast<float>(
      r(0, 0) * direction_camera[0] + r(0, 1) * direction_camera[1] +
      r(0, 2) * direction_camera[2]),
    static_cast<float>(
      r(1, 0) * direction_camera[0] + r(1, 1) * direction_camera[1] +
      r(1, 2) * direction_camera[2]),
    static_cast<float>(
      r(2, 0) * direction_camera[0] + r(2, 1) * direction_camera[1] +
      r(2, 2) * direction_camera[2]));

  const float bs = options_.voxel_size_m * kBlockSide;
  // Inside an allocated block, step a fraction of the truncation so the sign
  // change cannot be stepped over. Outside one, skip straight to where the ray
  // *leaves that block*, which is what makes this affordable: a 6 m ray at 3 cm
  // steps is 200 hash lookups and most rays are mostly air.
  const float fine_step = truncation_m_ * 0.5F;

  // **The skip has to be exact, and a fixed coarse step is not.** This was
  // written as `t += bs * 0.75` and it read a plane as no surface at all whenever
  // the ray was oblique: the positive side of the band is one truncation thick —
  // 6 cm, half a block — so a 9 cm stride lands past it, the first sample inside
  // the band is already *behind* the surface, and there is no sign change left to
  // find. The ray then reports 0 and the aligner counts the pixel as unobserved.
  // Seen as a panned camera finding nothing where a camera at the same place
  // looking straight ahead found the wall (test_tsdf_volume). A slab test costs
  // three divisions and cannot skip a band whatever the geometry.
  auto distance_to_block_exit = [&](const cv::Vec3f & p) {
      float best = bs;
      for (int axis = 0; axis < 3; ++axis) {
        const float d = dir[axis];
        if (std::fabs(d) < 1e-9F) {continue;}
        const float edge =
          (static_cast<float>(floor_div(p[axis], bs)) + (d > 0.0F ? 1.0F : 0.0F)) * bs;
        const float step = (edge - p[axis]) / d;
        if (step > 0.0F && step < best) {best = step;}
      }
      // Never zero: a point exactly on a boundary would otherwise loop forever.
      return std::max(best, 1e-4F);
    };

  const float vs = options_.voxel_size_m;

  // **One hash lookup per block entered, not two per sample.** The first version
  // asked the map twice at every step — once for "is this block allocated" and
  // again inside `sample()` — and a march is ~100 steps that mostly stay inside
  // the same block. That is ~200 lookups per ray into a hash map big enough to
  // miss cache on every one of them, and it showed: measured in the container,
  // the alignment ray-cast cost 23.9 ms at 35 000 blocks and 36.0 ms at 145 000,
  // growing with the map rather than with the picture. Caching the block the
  // march is currently inside makes the lookup happen when the ray crosses a
  // block boundary and at no other time.
  const Block * block = nullptr;
  int cached_x = 0;
  int cached_y = 0;
  int cached_z = 0;
  bool have_cache = false;

  float t_prev = 0.0F;
  float sdf_prev = 0.0F;
  bool have_prev = false;

  for (float t = options_.min_range_m; t <= options_.max_range_m; ) {
    const cv::Vec3f p = origin + dir * t;
    const int bx = floor_div(p[0], bs);
    const int by = floor_div(p[1], bs);
    const int bz = floor_div(p[2], bs);
    if (!have_cache || bx != cached_x || by != cached_y || bz != cached_z) {
      block = find_block(bx, by, bz);
      cached_x = bx;
      cached_y = by;
      cached_z = bz;
      have_cache = true;
    }
    if (!block) {
      have_prev = false;
      t += distance_to_block_exit(p) + 1e-4F;
      continue;
    }

    const int vx = floor_div(p[0], vs) - bx * kBlockSide;
    const int vy = floor_div(p[1], vs) - by * kBlockSide;
    const int vz = floor_div(p[2], vs) - bz * kBlockSide;
    if (vx < 0 || vx >= kBlockSide || vy < 0 || vy >= kBlockSide ||
      vz < 0 || vz >= kBlockSide)
    {
      have_prev = false;
      t += fine_step;
      continue;
    }
    const Voxel & voxel = block->voxels[(vz * kBlockSide + vy) * kBlockSide + vx];
    if (voxel.weight < options_.min_weight) {
      // Allocated but under weight: unknown, not free. Breaking the chain here
      // stops a crossing being interpolated across a gap of voxels nobody has
      // confirmed.
      have_prev = false;
      t += fine_step;
      continue;
    }
    const float sdf = voxel.sdf;
    if (have_prev && sdf_prev > 0.0F && sdf <= 0.0F) {
      const float span = sdf_prev - sdf;
      const float hit = span > 1e-6F ? t_prev + (t - t_prev) * (sdf_prev / span) : t;
      // The direction's z is 1 for the rays this is called with, so `t` is already
      // the z-depth. A caller that hands in a normalised direction gets ray length
      // instead and has to know it — which is why the header says which.
      return hit;
    }
    t_prev = t;
    sdf_prev = sdf;
    have_prev = true;
    t += fine_step;
  }
  return 0.0F;
}

void TsdfVolume::raycast(
  const cv::Matx33d & k, const cv::Affine3d & world_from_camera,
  cv::Size size, cv::Mat & expected_m) const
{
  expected_m.create(size, CV_32FC1);
  expected_m.setTo(0.0F);
  if (blocks_.empty()) {return;}

  const float fx = static_cast<float>(k(0, 0));
  const float fy = static_cast<float>(k(1, 1));
  const float cx = static_cast<float>(k(0, 2));
  const float cy = static_cast<float>(k(1, 2));
  if (!(fx > 0.0F) || !(fy > 0.0F)) {return;}

  for (int v = 0; v < size.height; ++v) {
    float * row = expected_m.ptr<float>(v);
    const float ny = (static_cast<float>(v) - cy) / fy;
    for (int u = 0; u < size.width; ++u) {
      const float nx = (static_cast<float>(u) - cx) / fx;
      // Third component 1, so what comes back is z and is directly comparable
      // with the incoming depth image.
      row[u] = raycast_ray(world_from_camera, cv::Vec3f(nx, ny, 1.0F));
    }
  }
}

}  // namespace pimesh_world
