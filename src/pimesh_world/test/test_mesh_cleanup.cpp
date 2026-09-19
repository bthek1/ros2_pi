// The cleanup policy, on surfaces whose right answer is obvious by construction.
//
// **Every failure available here is a mesh that renders.** A frontier filled is a
// sealed box — the most seductive false positive in this project, because a room
// with no doorway looks *more* finished than one with a hole in it. Debris left
// in place is a haze that reads as scanning noise. Decimation by subsampling
// looks like decimation until you notice the holes. None of them throws.

#include <cmath>
#include <string>
#include <map>
#include <set>

#include "gtest/gtest.h"
#include "pimesh_world/mesh.hpp"

using pimesh_world::Mesh;
using pimesh_world::boundary_loops;
using pimesh_world::decimate;
using pimesh_world::fill_interior_holes;
using pimesh_world::prune_small_components;

namespace
{

/// An `n` by `n` grid of vertices triangulated into a flat square sheet in z = 0.
///
/// A sheet has exactly one boundary loop — its rim — and that rim is a frontier:
/// it is the edge of the surface, not a hole in it.
Mesh sheet(int n, float spacing = 0.05F, float x0 = 0.0F, float y0 = 0.0F)
{
  Mesh mesh;
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      mesh.vertices.emplace_back(
        x0 + static_cast<float>(i) * spacing, y0 + static_cast<float>(j) * spacing, 0.0F);
      mesh.colours.emplace_back(0.5F, 0.25F, 0.75F);
    }
  }
  auto index = [n](int i, int j) {return j * n + i;};
  for (int j = 0; j + 1 < n; ++j) {
    for (int i = 0; i + 1 < n; ++i) {
      mesh.triangles.emplace_back(index(i, j), index(i + 1, j), index(i + 1, j + 1));
      mesh.triangles.emplace_back(index(i, j), index(i + 1, j + 1), index(i, j + 1));
    }
  }
  return mesh;
}

/// Remove the triangles covering one interior quad, leaving a hole in the middle.
void punch_hole(Mesh & mesh, int n, int i, int j)
{
  auto index = [n](int a, int b) {return b * n + a;};
  const int a = index(i, j);
  const int b = index(i + 1, j);
  const int c = index(i + 1, j + 1);
  const int d = index(i, j + 1);
  std::vector<cv::Vec3i> kept;
  for (const auto & tri : mesh.triangles) {
    std::set<int> corners {tri[0], tri[1], tri[2]};
    const bool is_lower = corners == std::set<int>{a, b, c};
    const bool is_upper = corners == std::set<int>{a, c, d};
    if (!is_lower && !is_upper) {kept.push_back(tri);}
  }
  mesh.triangles.swap(kept);
}

/// A unique path under /tmp for this process.
///
/// Not `std::tmpnam`, which the linker warns about on both distros and which
/// deserves the warning: it returns a name, and anything may create that file
/// between the return and the open. The pid plus a counter is unique enough for a
/// test and has no window in it.
/// Append `other`'s geometry to `mesh` as a separate component.
void append(Mesh & mesh, const Mesh & other)
{
  const int offset = static_cast<int>(mesh.vertices.size());
  mesh.vertices.insert(mesh.vertices.end(), other.vertices.begin(), other.vertices.end());
  mesh.colours.insert(mesh.colours.end(), other.colours.begin(), other.colours.end());
  for (const auto & tri : other.triangles) {
    mesh.triangles.emplace_back(tri[0] + offset, tri[1] + offset, tri[2] + offset);
  }
}

}  // namespace

// --- Boundary loops ----------------------------------------------------------

TEST(BoundaryLoops, AFlatSheetHasExactlyOneLoopAndItIsItsRim)
{
  const Mesh mesh = sheet(6);
  const auto loops = boundary_loops(mesh);
  ASSERT_EQ(loops.size(), 1U);
  // A 6x6 grid's rim is 4 * 5 = 20 vertices.
  EXPECT_EQ(loops[0].size(), 20U);
}

TEST(BoundaryLoops, ASheetWithAHoleInItHasTwo)
{
  Mesh mesh = sheet(6);
  punch_hole(mesh, 6, 2, 2);
  const auto loops = boundary_loops(mesh);
  ASSERT_EQ(loops.size(), 2U) << "the rim and the hole are two separate loops";

  std::size_t small = 0;
  for (const auto & loop : loops) {
    if (loop.size() == 4) {++small;}
  }
  EXPECT_EQ(small, 1U) << "the hole is a four-vertex loop";
}

TEST(BoundaryLoops, AClosedSurfaceHasNone)
{
  // Two sheets are still two rims; a tetrahedron is closed. The assertion that
  // matters is that a surface with no boundary edges produces no loops at all
  // rather than a spurious one, because a spurious loop is one the filler would
  // fan over a surface that was already complete.
  Mesh mesh;
  mesh.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  mesh.colours.assign(4, cv::Vec3f(1, 1, 1));
  mesh.triangles = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
  EXPECT_TRUE(boundary_loops(mesh).empty());
}

// --- Pruning ------------------------------------------------------------------

TEST(PruneSmallComponents, DebrisGoesAndTheSurfaceStays)
{
  Mesh mesh = sheet(8);                       // 98 triangles
  const std::size_t big = mesh.triangles.size();
  Mesh flake = sheet(2, 0.01F, 5.0F, 5.0F);   // 2 triangles, far away
  append(mesh, flake);
  Mesh flake2 = sheet(2, 0.01F, -5.0F, 5.0F);
  append(mesh, flake2);

  EXPECT_EQ(prune_small_components(mesh, 30), 2U);
  EXPECT_EQ(mesh.triangles.size(), big);
  // And the vertices the flakes used are gone, not merely unreferenced: a mesh
  // that keeps them publishes a Marker full of points nothing draws.
  EXPECT_EQ(mesh.vertices.size(), 64U);
}

TEST(PruneSmallComponents, ThePolicyIsPerComponentAndNotPerTriangle)
{
  // A component *made of* small triangles is not debris. Pruning by triangle size
  // rather than by component size eats the detailed parts of a real surface,
  // which is precisely backwards.
  Mesh mesh = sheet(10, 0.002F);
  const std::size_t before = mesh.triangles.size();
  EXPECT_EQ(prune_small_components(mesh, 30), 0U);
  EXPECT_EQ(mesh.triangles.size(), before);
}

TEST(PruneSmallComponents, AThresholdOfOneIsANoOp)
{
  Mesh mesh = sheet(4);
  const std::size_t before = mesh.triangles.size();
  EXPECT_EQ(prune_small_components(mesh, 1), 0U);
  EXPECT_EQ(mesh.triangles.size(), before);
}

// --- Filling -------------------------------------------------------------------

TEST(FillInteriorHoles, AHoleIsClosedAndTheRimIsNot)
{
  // **The assertion this whole file exists for.** The rim is the frontier — the
  // edge of what the camera saw — and closing it invents unseen space. A sealed
  // box with no openings looks more finished than a correct scan, which is why it
  // has to be a test rather than something anyone would notice.
  Mesh mesh = sheet(6);
  punch_hole(mesh, 6, 2, 2);
  ASSERT_EQ(boundary_loops(mesh).size(), 2U);

  EXPECT_EQ(fill_interior_holes(mesh, 0.25), 1U);

  const auto loops = boundary_loops(mesh);
  ASSERT_EQ(loops.size(), 1U) << "either the hole is still open or the rim was closed";
  EXPECT_EQ(loops[0].size(), 20U) << "the surviving loop is not the rim — the frontier was filled";
}

TEST(FillInteriorHoles, TheFillIsWoundToMatchTheSurfaceAroundIt)
{
  // A patch wound the other way renders as a dark disc in the middle of a lit
  // wall, and marching cubes cannot tell you which way round is right — only the
  // surface it is patching can.
  Mesh mesh = sheet(6);
  punch_hole(mesh, 6, 2, 2);
  const cv::Vec3f before = (mesh.vertices[mesh.triangles[0][1]] - mesh.vertices[mesh.triangles[0][0]])
    .cross(mesh.vertices[mesh.triangles[0][2]] - mesh.vertices[mesh.triangles[0][0]]);

  const std::size_t filled = mesh.triangles.size();
  ASSERT_EQ(fill_interior_holes(mesh, 0.25), 1U);

  for (std::size_t t = filled; t < mesh.triangles.size(); ++t) {
    const auto & tri = mesh.triangles[t];
    const cv::Vec3f normal =
      (mesh.vertices[tri[1]] - mesh.vertices[tri[0]])
      .cross(mesh.vertices[tri[2]] - mesh.vertices[tri[0]]);
    EXPECT_GT(normal.dot(before), 0.0F) << "patch triangle " << t << " faces backwards";
  }
}

TEST(FillInteriorHoles, AHoleWiderThanTheGuardStaysOpen)
{
  Mesh mesh = sheet(6);
  punch_hole(mesh, 6, 2, 2);
  // The hole's radius is ~0.035 m at 0.05 m spacing; a 0.01 m guard excludes it.
  EXPECT_EQ(fill_interior_holes(mesh, 0.01), 0U);
  EXPECT_EQ(boundary_loops(mesh).size(), 2U);
}

TEST(FillInteriorHoles, EachComponentKeepsItsOwnFrontier)
{
  // Two separate sheets, each with a hole. Four loops in, two out — and the two
  // that survive must be the two rims. A filler that exempted only the single
  // largest loop *in the whole mesh* would seal the smaller sheet completely.
  Mesh mesh = sheet(6);
  punch_hole(mesh, 6, 2, 2);
  Mesh second = sheet(5, 0.05F, 10.0F, 0.0F);
  punch_hole(second, 5, 1, 1);
  append(mesh, second);

  ASSERT_EQ(boundary_loops(mesh).size(), 4U);
  EXPECT_EQ(fill_interior_holes(mesh, 0.25), 2U);
  const auto loops = boundary_loops(mesh);
  ASSERT_EQ(loops.size(), 2U);
  for (const auto & loop : loops) {
    EXPECT_GT(loop.size(), 4U) << "a four-vertex loop survived, so a rim was filled "
                                  "and a hole was not";
  }
}

TEST(FillInteriorHoles, AClosedSurfaceIsLeftAlone)
{
  Mesh mesh;
  mesh.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  mesh.colours.assign(4, cv::Vec3f(1, 1, 1));
  mesh.triangles = {{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}};
  const std::size_t before = mesh.triangles.size();
  EXPECT_EQ(fill_interior_holes(mesh, 1.0), 0U);
  EXPECT_EQ(mesh.triangles.size(), before);
}

// --- Decimation -----------------------------------------------------------------

TEST(Decimate, TheCapIsActuallyReached)
{
  Mesh mesh = sheet(40);          // 3042 triangles
  ASSERT_GT(mesh.triangles.size(), 500U);
  EXPECT_GT(decimate(mesh, 500), 0U);
  EXPECT_LE(mesh.triangles.size(), 500U);
  EXPECT_GT(mesh.triangles.size(), 0U);
}

TEST(Decimate, AMeshAlreadyUnderTheCapIsUntouched)
{
  Mesh mesh = sheet(6);
  const std::size_t before = mesh.triangles.size();
  EXPECT_EQ(decimate(mesh, 10000), 0U);
  EXPECT_EQ(mesh.triangles.size(), before);
}

TEST(Decimate, AFlatSheetStaysFlat)
{
  // **What quadric decimation buys over subsampling, stated as a measurement.**
  // Every vertex of this sheet lies in z = 0, and collapsing an edge on a flat
  // surface costs nothing — so a correct implementation removes 90% of the
  // triangles and moves the surface by *zero*. Subsampling every Nth triangle
  // would leave holes, and collapsing to an arbitrary target would lift vertices
  // off the plane.
  Mesh mesh = sheet(30);
  ASSERT_GT(decimate(mesh, 200), 0U);
  ASSERT_LE(mesh.triangles.size(), 200U);

  double worst = 0.0;
  for (const auto & v : mesh.vertices) {
    worst = std::max(worst, std::fabs(static_cast<double>(v[2])));
  }
  EXPECT_LT(worst, 1e-5) << "decimation lifted the surface " << worst << " m off its own plane";
}

TEST(Decimate, TheSurfaceKeepsItsExtentRatherThanShrinkingInFromTheEdges)
{
  // A decimator with no regard for the boundary eats the rim first, because a
  // boundary vertex has fewer triangles constraining it — and the surface shrinks
  // from the outside in, which on a room reads as the walls receding.
  Mesh mesh = sheet(30);
  double before_max = 0.0;
  for (const auto & v : mesh.vertices) {before_max = std::max(before_max, double(v[0]));}

  decimate(mesh, 200);

  double after_max = 0.0;
  for (const auto & v : mesh.vertices) {after_max = std::max(after_max, double(v[0]));}
  EXPECT_NEAR(after_max, before_max, 0.1 * before_max)
    << "the surface lost " << (before_max - after_max) << " m of extent";
}

TEST(Decimate, NoDegenerateTrianglesSurvive)
{
  // A collapse turns every triangle that had both endpoints into a sliver with
  // two identical corners. Left in, they contribute nothing to the picture and
  // break the boundary walk — a zero-area triangle has two identical edges and
  // the walk follows one of them forever.
  Mesh mesh = sheet(30);
  decimate(mesh, 200);
  for (const auto & tri : mesh.triangles) {
    EXPECT_NE(tri[0], tri[1]);
    EXPECT_NE(tri[1], tri[2]);
    EXPECT_NE(tri[0], tri[2]);
    EXPECT_GE(tri[0], 0);
    EXPECT_LT(static_cast<std::size_t>(tri[0]), mesh.vertices.size());
    EXPECT_LT(static_cast<std::size_t>(tri[1]), mesh.vertices.size());
    EXPECT_LT(static_cast<std::size_t>(tri[2]), mesh.vertices.size());
  }
  EXPECT_EQ(mesh.vertices.size(), mesh.colours.size());
}

// PLY moved to test_mesh_io.cpp on 2026-09-19. It lived here because `decimate`
// was the first thing that wanted a file written, and a round trip was all it
// asserted — which is the one shape of test that cannot see a writer and a
// reader being wrong together. The new suite reads the bytes as well.
