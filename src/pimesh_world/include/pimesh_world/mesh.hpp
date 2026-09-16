#ifndef PIMESH_WORLD__MESH_HPP_
#define PIMESH_WORLD__MESH_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "opencv2/core.hpp"
#include "pimesh_world/shared_volume.hpp"

namespace pimesh_world
{

/// A triangle surface: positions, per-vertex colour, and an index buffer.
///
/// **Indexed, not a triangle soup**, all the way until the Marker is built. The
/// cleanup in `mesh_cleanup.hpp` is entirely about *connectivity* — which
/// triangles share an edge, which loops are open — and connectivity does not
/// exist in a soup: two triangles meeting at a corner would have two coincident
/// vertices with different indices and every boundary test would say the surface
/// is one triangle wide. `visualization_msgs/Marker` TRIANGLE_LIST does want the
/// soup, and that flattening is the last thing that happens.
struct Mesh
{
  std::vector<cv::Vec3f> vertices;
  /// Per vertex, each channel in [0, 1]. Same length as `vertices`, always.
  std::vector<cv::Vec3f> colours;
  /// Indices into `vertices`, three per triangle.
  std::vector<cv::Vec3i> triangles;

  bool empty() const {return triangles.empty();}
  std::size_t vertex_count() const {return vertices.size();}
  std::size_t triangle_count() const {return triangles.size();}
  void clear()
  {
    vertices.clear();
    colours.clear();
    triangles.clear();
  }
};

/// Marching cubes over a snapshot of the volume.
///
/// **The classic algorithm, and the one thing worth stating about it here is what
/// it does when it does not know.** A cell whose eight corners are not *all* at
/// or above `min_weight` is skipped entirely rather than meshed with the missing
/// corners read as zero — and zero is exactly the value an untouched voxel holds,
/// so reading it would put a surface at the boundary of every unobserved region.
/// The room would come out sealed in a shell of invented geometry that looks,
/// from inside, like walls.
///
/// The sign convention follows the volume's: **positive is in front of the
/// surface**, in the free space between the camera and the wall, and negative is
/// behind it. So "inside" for the lookup tables is `sdf < 0`, and the triangles
/// come out wound with their normals pointing into the space the camera was in.
///
/// Cells at a block's far edge reach into the neighbouring block, which is what
/// makes the surface continuous across block boundaries instead of a grid of
/// 12 cm tiles with seams between them. A neighbour that is not allocated simply
/// fails the weight test above.
///
/// `min_weight` overrides the snapshot's own threshold when it is larger. That is
/// not a tuning knob for prettiness: the volume's threshold is the noise floor
/// for *ray-casting*, and meshing has a different question to answer — a voxel
/// seen three times is real enough to stop a ray and thin enough to be one of the
/// shingles a sweep lays down.
Mesh march_cubes(const SharedVolume::Snapshot & snapshot, float min_weight);

// --- Cleanup ----------------------------------------------------------------
//
// The P0 census in the predecessor found that "holes" in a marching-cubes surface
// are three different things and only one of them should be filled. That finding
// is the shape of everything below.

/// Drop connected components with fewer than `min_triangles` triangles.
///
/// **Debris islands**: hundreds of tiny disconnected flakes, each a few triangles
/// of noise. Their outer boundaries masquerade as holes, and the honest fix is to
/// remove the flakes rather than cap them into blobs. Returns how many components
/// were dropped; vertices are re-indexed densely.
std::size_t prune_small_components(Mesh & mesh, std::size_t min_triangles);

/// Boundary loops, as lists of vertex indices in the winding order the
/// surrounding surface implies.
///
/// A boundary edge belongs to exactly one triangle. Following each *directed*
/// boundary edge a->b chains the loops in an orientation a fill patch can match.
///
/// **The walk consumes directed edges, not vertices**, and that is not a detail:
/// marching-cubes surfaces pinch loops through shared, non-manifold vertices, and
/// a vertex-keyed walk silently drops every loop that crosses one. The
/// predecessor measured that as small holes its filler never saw. At a pinch the
/// greedy choice may fuse two loops into one figure-eight; it still gets
/// classified and filled.
std::vector<std::vector<int>> boundary_loops(const Mesh & mesh);

/// Close interior boundary loops; leave every component's frontier open.
///
/// Three kinds of loop and only one gets filled:
///
///  - **Frontiers.** Every connected component's *largest* boundary loop is the
///    edge of what the camera has seen. Filling it would invent unseen space, so
///    each component keeps its largest loop open. **This is the rule that must
///    never be relaxed**: a sealed box with no openings is the failure it
///    prevents, and it is the one that looks most like success.
///  - **Interior holes**, bounded by observed surface on all sides. These are
///    filled by a fan to the loop's centroid, coloured from the ring — the
///    surface assumed from its surroundings.
///  - **Anything wider than `max_radius_m`** stays open whatever it is, as a
///    guard against silently bridging something frontier-sized that the
///    component test missed.
///
/// Returns how many loops were filled.
std::size_t fill_interior_holes(Mesh & mesh, double max_radius_m);

/// Reduce to at most `max_triangles` by **quadric error decimation**.
///
/// **Never by subsampling**, which is what P6 says and what the difference
/// actually costs. Dropping every second triangle punches holes in a surface;
/// dropping every second *vertex* tears it. Quadric decimation collapses the edge
/// whose removal moves the surface least, measured as the summed squared distance
/// to the planes of the triangles meeting at it — so a flat wall loses almost all
/// its triangles and a desk edge keeps its own.
///
/// Returns the number of edge collapses performed. A mesh already under the cap
/// is returned untouched, which is the normal case for a saved PLY: the cap
/// exists because a `Marker` is rebuilt and re-serialised on every publish, not
/// because the geometry is too detailed.
std::size_t decimate(Mesh & mesh, std::size_t max_triangles);

// --- Files ------------------------------------------------------------------

/// Write a binary little-endian PLY with per-vertex colour.
///
/// Hand-written rather than through a library, which is this project's habit for
/// a format it only needs one direction of — and here it also keeps the save path
/// free of anything the unit tests would have to have installed. Binary rather
/// than ASCII because a full-detail room mesh is millions of triangles and ASCII
/// is about five times the bytes and considerably slower to write.
///
/// Returns false and leaves nothing behind if the file cannot be written.
bool write_ply(const Mesh & mesh, const std::string & path, std::string & error);

/// Read one back. Only the subset `write_ply` produces, and it exists so that
/// `tools/gates/mesh.sh` can assert on a file that was actually written rather
/// than on the counts the node reported about it.
bool read_ply(const std::string & path, Mesh & mesh, std::string & error);

}  // namespace pimesh_world

#endif  // PIMESH_WORLD__MESH_HPP_
