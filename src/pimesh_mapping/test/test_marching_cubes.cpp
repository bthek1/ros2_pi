// Marching cubes, against a surface whose shape is known analytically.
//
// **The point of this file is the 256-entry lookup table.** It is copied constant
// data, and a single wrong entry produces a surface that is wrong only for one
// arrangement of corner signs — which on a real room is a scattering of small
// holes that reads as scanning noise, and which no gate measuring a triangle
// count would ever notice. A closed surface has *no boundary edges at all*, and
// that one assertion over a sphere exercises the overwhelming majority of the
// table's cases at once: a wrong entry leaves an edge belonging to one triangle
// instead of two.

#include <cmath>
#include <map>
#include <set>

#include "gtest/gtest.h"
#include "pimesh_mapping/mesh.hpp"

using pimesh_mapping::Mesh;
using pimesh_mapping::SharedVolume;
using pimesh_mapping::TsdfVolume;
using pimesh_mapping::march_cubes;

namespace
{

constexpr float kVoxel = 0.015F;
constexpr float kTruncation = 0.06F;

/// A snapshot holding an analytic sphere, in the volume's own sign convention:
/// **positive outside**, where the free space the camera looked through is, and
/// negative inside the solid.
///
/// Every voxel within `band` voxels of the surface is given full weight and the
/// rest are left untouched — which is what a real integration produces, and it
/// means the cells at the band's edge have all-known corners of one sign and are
/// skipped rather than meshed.
SharedVolume::Snapshot sphere(float radius, int band = 6)
{
  SharedVolume::Snapshot snapshot;
  snapshot.voxel_size_m = kVoxel;
  snapshot.truncation_m = kTruncation;
  snapshot.min_weight = 3.0F;

  constexpr int side = TsdfVolume::kBlockSide;
  const int reach = static_cast<int>(radius / kVoxel) + band + side;
  const int block_reach = reach / side + 1;

  for (int bz = -block_reach; bz <= block_reach; ++bz) {
    for (int by = -block_reach; by <= block_reach; ++by) {
      for (int bx = -block_reach; bx <= block_reach; ++bx) {
        TsdfVolume::Block block;
        bool any = false;
        for (int lz = 0; lz < side; ++lz) {
          for (int ly = 0; ly < side; ++ly) {
            for (int lx = 0; lx < side; ++lx) {
              const float x = (bx * side + lx + 0.5F) * kVoxel;
              const float y = (by * side + ly + 0.5F) * kVoxel;
              const float z = (bz * side + lz + 0.5F) * kVoxel;
              const float distance = std::sqrt(x * x + y * y + z * z) - radius;
              if (std::fabs(distance) > band * kVoxel) {continue;}
              auto & voxel = block.voxels[(lz * side + ly) * side + lx];
              voxel.sdf = std::max(-1.0F, std::min(1.0F, distance / kTruncation));
              voxel.weight = 10.0F;
              voxel.r = 200;
              voxel.g = 100;
              voxel.b = 50;
              any = true;
            }
          }
        }
        if (any) {snapshot.blocks.emplace(TsdfVolume::block_key(bx, by, bz), block);}
      }
    }
  }
  snapshot.voxels_allocated = snapshot.blocks.size() * TsdfVolume::kBlockVoxels;
  return snapshot;
}

/// Edges that belong to exactly one triangle. Zero of them is a closed surface.
std::size_t boundary_edge_count(const Mesh & mesh)
{
  std::map<std::pair<int, int>, int> uses;
  for (const auto & tri : mesh.triangles) {
    const int v[3] = {tri[0], tri[1], tri[2]};
    for (int i = 0; i < 3; ++i) {
      const int a = v[i];
      const int b = v[(i + 1) % 3];
      ++uses[{std::min(a, b), std::max(a, b)}];
    }
  }
  std::size_t open = 0;
  for (const auto & entry : uses) {
    if (entry.second == 1) {++open;}
  }
  return open;
}

}  // namespace

TEST(MarchingCubes, ASphereComesOutClosed)
{
  // **The table check.** A closed surface has every edge shared by exactly two
  // triangles. A wrong entry in the 256-case table leaves an edge used once, for
  // that one arrangement of corner signs — a hole that looks like noise on a real
  // scan and is unmistakable here.
  const Mesh mesh = march_cubes(sphere(0.30F), 3.0F);
  ASSERT_GT(mesh.triangle_count(), 1000U) << "the sphere produced almost no surface";
  EXPECT_EQ(boundary_edge_count(mesh), 0U)
    << "the surface is not closed, which for a sphere fully inside the known "
       "region means a case in kTriTable is wrong";
}

TEST(MarchingCubes, EveryVertexLandsOnTheSphere)
{
  // The interpolation. Rounding to the nearest corner instead of interpolating
  // still produces a closed, sphere-shaped surface — visibly faceted, and
  // *accurate to half a voxel at best*, where interpolation is accurate to a
  // small fraction of one. On a 15 mm grid that is the difference between a flat
  // wall and a corrugated one.
  const float radius = 0.30F;
  const Mesh mesh = march_cubes(sphere(radius), 3.0F);
  ASSERT_FALSE(mesh.empty());

  double worst = 0.0;
  double sum = 0.0;
  for (const auto & v : mesh.vertices) {
    const double r = std::sqrt(
      static_cast<double>(v[0]) * v[0] + static_cast<double>(v[1]) * v[1] +
      static_cast<double>(v[2]) * v[2]);
    const double error = std::fabs(r - radius);
    worst = std::max(worst, error);
    sum += error;
  }
  const double mean = sum / static_cast<double>(mesh.vertex_count());
  EXPECT_LT(worst, 0.5 * kVoxel) << "worst vertex is " << worst << " m off the sphere";
  EXPECT_LT(mean, 0.1 * kVoxel) << "mean radial error " << mean
                                << " m — this is nearest-corner, not interpolation";
}

TEST(MarchingCubes, TrianglesAreWoundWithTheirNormalsInTheFreeSpace)
{
  // The winding, and it is not cosmetic. A surface wound inside out renders as a
  // shell lit from within — every wall dark, every normal pointing into the
  // plaster — and marching cubes finds the identical zero crossing either way, so
  // nothing about the geometry says which happened.
  //
  // Our convention: **positive is the free space the camera looked through**. So
  // for this sphere the normals must point outwards, away from the centre.
  const Mesh mesh = march_cubes(sphere(0.30F), 3.0F);
  ASSERT_FALSE(mesh.empty());

  std::size_t outward = 0;
  for (const auto & tri : mesh.triangles) {
    const cv::Vec3f & a = mesh.vertices[tri[0]];
    const cv::Vec3f & b = mesh.vertices[tri[1]];
    const cv::Vec3f & c = mesh.vertices[tri[2]];
    const cv::Vec3f normal = (b - a).cross(c - a);
    const cv::Vec3f centroid = (a + b + c) / 3.0F;
    if (normal.dot(centroid) > 0.0F) {++outward;}
  }
  EXPECT_EQ(outward, mesh.triangle_count())
    << outward << " of " << mesh.triangle_count()
    << " triangles face outwards — the winding is inconsistent or inverted";
}

TEST(MarchingCubes, VerticesAreSharedBetweenTheCellsThatMeetAtThem)
{
  // **Sharing is the connectivity, not an optimisation.** Everything the cleanup
  // does — find components, walk boundary loops, tell a hole from a frontier —
  // asks which triangles share an edge. If every cell made its own coincident
  // vertices, every edge would belong to exactly one triangle, the whole surface
  // would read as boundary, and `fill_interior_holes` would have nothing to
  // classify. A closed triangulated surface has roughly half as many vertices as
  // triangles (Euler); a soup has exactly three times as many.
  const Mesh mesh = march_cubes(sphere(0.30F), 3.0F);
  ASSERT_FALSE(mesh.empty());
  EXPECT_EQ(mesh.vertices.size(), mesh.colours.size());
  EXPECT_LT(mesh.vertex_count(), mesh.triangle_count())
    << "there are more vertices than triangles, so nothing is being shared";
  EXPECT_NEAR(
    static_cast<double>(mesh.vertex_count()) / static_cast<double>(mesh.triangle_count()),
    0.5, 0.1);
}

TEST(MarchingCubes, AnUnobservedVoxelIsNotASurface)
{
  // **The failure this guards is the one that looks most like success.** An
  // untouched voxel holds sdf exactly 0.0 — the iso-value itself — so a mesher
  // that reads it puts a surface at the boundary of every unobserved region. The
  // room comes out sealed inside a shell of invented geometry that, from within,
  // looks like walls.
  //
  // Here every voxel is allocated and none has enough weight, so the honest
  // answer is an empty mesh rather than a box.
  SharedVolume::Snapshot snapshot;
  snapshot.voxel_size_m = kVoxel;
  snapshot.truncation_m = kTruncation;
  snapshot.min_weight = 3.0F;
  TsdfVolume::Block block;
  for (int i = 0; i < TsdfVolume::kBlockVoxels; ++i) {
    block.voxels[i].weight = 1.0F;      // seen once: under the threshold
    block.voxels[i].sdf = 0.0F;
  }
  for (int z = -1; z <= 1; ++z) {
    for (int y = -1; y <= 1; ++y) {
      for (int x = -1; x <= 1; ++x) {
        snapshot.blocks.emplace(TsdfVolume::block_key(x, y, z), block);
      }
    }
  }

  const Mesh mesh = march_cubes(snapshot, 3.0F);
  EXPECT_TRUE(mesh.empty())
    << mesh.triangle_count() << " triangles came out of voxels nobody has confirmed";
}

TEST(MarchingCubes, TheMeshingThresholdIsHonouredAboveTheVolumeSOwn)
{
  // The volume's threshold is the noise floor for ray-casting; meshing asks a
  // different question, and a voxel real enough to stop a ray can still be one of
  // the shingles a sweep lays down. Raising it must actually raise it.
  SharedVolume::Snapshot snapshot = sphere(0.30F);
  for (auto & entry : snapshot.blocks) {
    for (int i = 0; i < TsdfVolume::kBlockVoxels; ++i) {
      if (entry.second.voxels[i].weight > 0.0F) {entry.second.voxels[i].weight = 5.0F;}
    }
  }
  EXPECT_FALSE(march_cubes(snapshot, 5.0F).empty());
  EXPECT_TRUE(march_cubes(snapshot, 6.0F).empty())
    << "a threshold above every voxel's weight still produced a surface";
}

TEST(MarchingCubes, ColourComesFromTheVoxelsTheVertexSitsBetween)
{
  const Mesh mesh = march_cubes(sphere(0.30F), 3.0F);
  ASSERT_FALSE(mesh.empty());
  for (const auto & c : mesh.colours) {
    // The sphere is one colour throughout, so every interpolated vertex must be
    // that colour — a channel swap would show as the wrong one, and a colour read
    // from the wrong voxel as a scatter.
    EXPECT_NEAR(c[0], 200.0F / 255.0F, 1e-4) << "red channel";
    EXPECT_NEAR(c[1], 100.0F / 255.0F, 1e-4) << "green channel";
    EXPECT_NEAR(c[2], 50.0F / 255.0F, 1e-4) << "blue channel";
  }
}

TEST(MarchingCubes, AnEmptySnapshotIsAnEmptyMesh)
{
  SharedVolume::Snapshot snapshot;
  EXPECT_TRUE(march_cubes(snapshot, 3.0F).empty());
}
