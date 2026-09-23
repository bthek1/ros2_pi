// Debris, frontiers, interior holes, and the triangle budget.
//
// The policy here is the predecessor's, which arrived at it by counting what was
// actually wrong with a live scan: "holes" in a marching-cubes surface are three
// different things and only one of them should be filled. Everything in this file
// is connectivity — which triangles share an edge, which loops are closed — and
// none of it works on a triangle soup, which is why `march_cubes` shares its
// vertices between the cells that meet at them.

#include <algorithm>
#include <cmath>
#include <numeric>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "pimesh_mapping/mesh.hpp"

namespace pimesh_mapping
{
namespace
{

/// Union-find with path halving, over vertices joined by triangle edges.
class Components
{
public:
  explicit Components(std::size_t n)
  : parent_(n)
  {
    std::iota(parent_.begin(), parent_.end(), 0);
  }

  int find(int x)
  {
    while (parent_[x] != x) {
      parent_[x] = parent_[parent_[x]];
      x = parent_[x];
    }
    return x;
  }

  void join(int a, int b)
  {
    a = find(a);
    b = find(b);
    if (a != b) {parent_[b] = a;}
  }

private:
  std::vector<int> parent_;
};

Components vertex_components(const Mesh & mesh)
{
  Components components(mesh.vertices.size());
  for (const auto & tri : mesh.triangles) {
    components.join(tri[0], tri[1]);
    components.join(tri[0], tri[2]);
  }
  return components;
}

std::int64_t undirected_key(int a, int b)
{
  const int low = std::min(a, b);
  const int high = std::max(a, b);
  return (static_cast<std::int64_t>(low) << 32) | static_cast<std::uint32_t>(high);
}

/// Drop vertices nothing references and renumber what is left.
void compact(Mesh & mesh)
{
  std::vector<int> remap(mesh.vertices.size(), -1);
  std::vector<cv::Vec3f> vertices;
  std::vector<cv::Vec3f> colours;
  vertices.reserve(mesh.vertices.size());
  colours.reserve(mesh.colours.size());

  for (auto & tri : mesh.triangles) {
    for (int i = 0; i < 3; ++i) {
      int & index = tri[i];
      if (remap[index] < 0) {
        remap[index] = static_cast<int>(vertices.size());
        vertices.push_back(mesh.vertices[index]);
        colours.push_back(mesh.colours[index]);
      }
      index = remap[index];
    }
  }
  mesh.vertices.swap(vertices);
  mesh.colours.swap(colours);
}

// --- Quadrics ---------------------------------------------------------------

/// A symmetric 4x4 in its ten distinct entries: the summed squared distance from
/// a point to the planes of the triangles that met at a vertex.
struct Quadric
{
  double q[10] {};

  void add_plane(double a, double b, double c, double d, double weight)
  {
    q[0] += weight * a * a;
    q[1] += weight * a * b;
    q[2] += weight * a * c;
    q[3] += weight * a * d;
    q[4] += weight * b * b;
    q[5] += weight * b * c;
    q[6] += weight * b * d;
    q[7] += weight * c * c;
    q[8] += weight * c * d;
    q[9] += weight * d * d;
  }

  Quadric & operator+=(const Quadric & other)
  {
    for (int i = 0; i < 10; ++i) {q[i] += other.q[i];}
    return *this;
  }

  /// v^T Q v — the error of putting the collapsed vertex here.
  double error(const cv::Vec3f & v) const
  {
    const double x = v[0];
    const double y = v[1];
    const double z = v[2];
    return q[0] * x * x + 2 * q[1] * x * y + 2 * q[2] * x * z + 2 * q[3] * x +
           q[4] * y * y + 2 * q[5] * y * z + 2 * q[6] * y +
           q[7] * z * z + 2 * q[8] * z + q[9];
  }
};

struct Candidate
{
  double cost;
  int a;
  int b;
  /// The version each endpoint had when this was costed. A collapse elsewhere
  /// changes a vertex's quadric and its neighbourhood, so an entry costed before
  /// that is stale — and lazily discarding stale entries is what lets this use a
  /// plain heap instead of a priority queue that supports decrease-key.
  std::uint32_t version_a;
  std::uint32_t version_b;

  bool operator<(const Candidate & other) const
  {
    // Reversed: std::priority_queue is a max-heap and the cheapest collapse is
    // the one to take.
    return cost > other.cost;
  }
};

}  // namespace

std::size_t prune_small_components(Mesh & mesh, std::size_t min_triangles)
{
  if (min_triangles <= 1 || mesh.triangles.empty()) {return 0;}

  Components components = vertex_components(mesh);
  std::unordered_map<int, std::size_t> triangles_per_root;
  for (const auto & tri : mesh.triangles) {
    ++triangles_per_root[components.find(tri[0])];
  }

  std::unordered_set<int> keep;
  std::size_t pruned = 0;
  for (const auto & entry : triangles_per_root) {
    if (entry.second >= min_triangles) {
      keep.insert(entry.first);
    } else {
      ++pruned;
    }
  }
  if (pruned == 0) {return 0;}

  std::vector<cv::Vec3i> survivors;
  survivors.reserve(mesh.triangles.size());
  for (const auto & tri : mesh.triangles) {
    if (keep.count(components.find(tri[0]))) {survivors.push_back(tri);}
  }
  mesh.triangles.swap(survivors);
  compact(mesh);
  return pruned;
}

std::vector<std::vector<int>> boundary_loops(const Mesh & mesh)
{
  std::vector<std::vector<int>> loops;
  if (mesh.triangles.empty()) {return loops;}

  // A boundary edge is one that belongs to exactly one triangle.
  std::unordered_map<std::int64_t, int> uses;
  uses.reserve(mesh.triangles.size() * 3);
  for (const auto & tri : mesh.triangles) {
    for (int i = 0; i < 3; ++i) {
      ++uses[undirected_key(tri[i], tri[(i + 1) % 3])];
    }
  }

  // **Directed edges, not vertices.** Marching-cubes surfaces pinch loops through
  // shared non-manifold vertices, and a vertex-keyed walk silently drops every
  // loop that crosses one — which the predecessor measured as small holes its
  // filler never saw. Following a -> b in the winding order the surrounding
  // triangle implies also gives the fill patch an orientation to match.
  std::unordered_map<int, std::vector<int>> successors;
  std::vector<std::pair<int, int>> edges;
  for (const auto & tri : mesh.triangles) {
    for (int i = 0; i < 3; ++i) {
      const int a = tri[i];
      const int b = tri[(i + 1) % 3];
      auto it = uses.find(undirected_key(a, b));
      if (it != uses.end() && it->second == 1) {
        successors[a].push_back(b);
        edges.emplace_back(a, b);
      }
    }
  }

  std::unordered_set<std::int64_t> walked;
  for (const auto & start : edges) {
    const std::int64_t start_key =
      (static_cast<std::int64_t>(start.first) << 32) | static_cast<std::uint32_t>(start.second);
    if (walked.count(start_key)) {continue;}

    std::vector<int> loop {start.first};
    std::pair<int, int> current = start;
    while (true) {
      const std::int64_t key =
        (static_cast<std::int64_t>(current.first) << 32) |
        static_cast<std::uint32_t>(current.second);
      if (walked.count(key)) {break;}
      walked.insert(key);
      loop.push_back(current.second);

      int next = -1;
      auto it = successors.find(current.second);
      if (it != successors.end()) {
        for (const int candidate : it->second) {
          const std::int64_t candidate_key =
            (static_cast<std::int64_t>(current.second) << 32) |
            static_cast<std::uint32_t>(candidate);
          if (!walked.count(candidate_key)) {
            next = candidate;
            break;
          }
        }
      }
      if (next < 0) {break;}
      current = {current.second, next};
    }

    // Closed if the walk came back to where it started. At a pinch the greedy
    // choice above may fuse two loops into one figure-eight; it still gets
    // classified and filled, which is better than being dropped.
    if (loop.size() >= 4 && loop.front() == loop.back()) {
      loop.pop_back();
      loops.push_back(std::move(loop));
    }
  }
  return loops;
}

std::size_t fill_interior_holes(Mesh & mesh, double max_radius_m)
{
  const auto loops = boundary_loops(mesh);
  if (loops.empty()) {return 0;}

  Components components = vertex_components(mesh);

  // **Each component's largest loop is its frontier and must stay open.** It is
  // the edge of what the camera has seen; closing it invents unseen space, and a
  // sealed box with no openings is the failure that looks most like success.
  std::vector<double> radii(loops.size(), 0.0);
  std::unordered_map<int, std::pair<double, std::size_t>> frontier;
  for (std::size_t i = 0; i < loops.size(); ++i) {
    cv::Vec3f centroid(0.0F, 0.0F, 0.0F);
    for (const int index : loops[i]) {centroid += mesh.vertices[index];}
    centroid /= static_cast<float>(loops[i].size());
    double radius = 0.0;
    for (const int index : loops[i]) {
      radius = std::max(radius, cv::norm(mesh.vertices[index] - centroid));
    }
    radii[i] = radius;

    const int root = components.find(loops[i].front());
    auto it = frontier.find(root);
    if (it == frontier.end() || radius > it->second.first) {
      frontier[root] = {radius, i};
    }
  }
  std::unordered_set<std::size_t> exempt;
  for (const auto & entry : frontier) {exempt.insert(entry.second.second);}

  std::size_t filled = 0;
  for (std::size_t i = 0; i < loops.size(); ++i) {
    if (exempt.count(i)) {continue;}
    // The radius guard, against silently bridging something frontier-sized that
    // the component test missed — a loop that is the edge of the scan but not the
    // largest one on its component, because the component happens to include two.
    if (radii[i] > max_radius_m) {continue;}

    const auto & loop = loops[i];
    cv::Vec3f centroid(0.0F, 0.0F, 0.0F);
    cv::Vec3f colour(0.0F, 0.0F, 0.0F);
    for (const int index : loop) {
      centroid += mesh.vertices[index];
      colour += mesh.colours[index];
    }
    centroid /= static_cast<float>(loop.size());
    colour /= static_cast<float>(loop.size());

    // A fan to the loop's centroid: the surface assumed from its surroundings,
    // positioned and coloured by the ring around it.
    const int hub = static_cast<int>(mesh.vertices.size());
    mesh.vertices.push_back(centroid);
    mesh.colours.push_back(colour);
    for (std::size_t j = 0; j < loop.size(); ++j) {
      const int a = loop[j];
      const int b = loop[(j + 1) % loop.size()];
      // (b, a, hub) rather than (a, b, hub): the boundary edge runs a -> b in the
      // winding of the triangle it came from, so the patch has to run the other
      // way to face the same direction as the surface it is closing.
      mesh.triangles.emplace_back(b, a, hub);
    }
    ++filled;
  }
  return filled;
}

std::size_t decimate(Mesh & mesh, std::size_t max_triangles)
{
  if (mesh.triangles.size() <= max_triangles || max_triangles == 0) {return 0;}

  const std::size_t vertex_count = mesh.vertices.size();

  // --- Quadrics, one per vertex, area-weighted -------------------------------
  //
  // Area weighting is what makes a large flat triangle harder to move than a
  // small one. Without it, decimation eats the big flat wall first — because each
  // of its few triangles counts the same as each of the many tiny ones on a desk
  // edge — which is the opposite of what is wanted.
  std::vector<Quadric> quadrics(vertex_count);
  for (const auto & tri : mesh.triangles) {
    const cv::Vec3f & a = mesh.vertices[tri[0]];
    const cv::Vec3f & b = mesh.vertices[tri[1]];
    const cv::Vec3f & c = mesh.vertices[tri[2]];
    cv::Vec3f normal = (b - a).cross(c - a);
    const double area = cv::norm(normal) * 0.5;
    if (area < 1e-14) {continue;}
    normal /= static_cast<float>(cv::norm(normal));
    const double d = -normal.dot(a);
    for (int i = 0; i < 3; ++i) {
      quadrics[tri[i]].add_plane(normal[0], normal[1], normal[2], d, area);
    }
  }

  std::vector<std::vector<int>> incident(vertex_count);
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    for (int i = 0; i < 3; ++i) {incident[mesh.triangles[t][i]].push_back(static_cast<int>(t));}
  }

  std::vector<std::uint32_t> version(vertex_count, 0);
  std::vector<bool> dead_vertex(vertex_count, false);
  std::vector<bool> dead_triangle(mesh.triangles.size(), false);
  std::vector<int> collapsed_to(vertex_count);
  std::iota(collapsed_to.begin(), collapsed_to.end(), 0);

  auto resolve = [&collapsed_to](int v) {
      while (collapsed_to[v] != v) {
        collapsed_to[v] = collapsed_to[collapsed_to[v]];
        v = collapsed_to[v];
      }
      return v;
    };

  // The collapse target is whichever of the two endpoints or their midpoint costs
  // least. Solving the 4x4 for the true optimum is the textbook version and it
  // puts the new vertex somewhere neither endpoint was — which on an open surface
  // can push it off the edge of the scan. Three candidates is within a few
  // percent on the error and never invents a position.
  auto best_target = [&](int a, int b, cv::Vec3f & target) {
      Quadric sum = quadrics[a];
      sum += quadrics[b];
      const cv::Vec3f mid = (mesh.vertices[a] + mesh.vertices[b]) * 0.5F;
      const double ca = sum.error(mesh.vertices[a]);
      const double cb = sum.error(mesh.vertices[b]);
      const double cm = sum.error(mid);
      if (ca <= cb && ca <= cm) {
        target = mesh.vertices[a];
        return ca;
      }
      if (cb <= cm) {
        target = mesh.vertices[b];
        return cb;
      }
      target = mid;
      return cm;
    };

  // A vector-backed heap with the capacity reserved up front. Every collapse
  // re-costs the edges around the surviving vertex and pushes them, so the queue
  // sees several times the edge count over a run — and growing it by doubling
  // from empty, at millions of entries, is a series of reallocations each copying
  // the last.
  std::vector<Candidate> heap_storage;
  heap_storage.reserve(mesh.triangles.size() * 4);
  std::priority_queue<Candidate, std::vector<Candidate>> queue(
    std::less<Candidate>(), std::move(heap_storage));
  auto push_edge = [&](int a, int b) {
      if (a == b) {return;}
      cv::Vec3f target;
      const double cost = best_target(a, b, target);
      queue.push({cost, a, b, version[a], version[b]});
    };

  {
    std::unordered_set<std::int64_t> seen;
    seen.reserve(mesh.triangles.size() * 3);
    for (const auto & tri : mesh.triangles) {
      for (int i = 0; i < 3; ++i) {
        const int a = tri[i];
        const int b = tri[(i + 1) % 3];
        if (seen.insert(undirected_key(a, b)).second) {push_edge(a, b);}
      }
    }
  }

  std::size_t live_triangles = mesh.triangles.size();
  std::size_t collapses = 0;
  std::vector<int> neighbour_scratch;
  std::vector<int> live_scratch;
  neighbour_scratch.reserve(64);
  live_scratch.reserve(64);

  while (live_triangles > max_triangles && !queue.empty()) {
    const Candidate candidate = queue.top();
    queue.pop();
    const int a = candidate.a;
    const int b = candidate.b;
    if (dead_vertex[a] || dead_vertex[b]) {continue;}
    // Stale: something adjacent has collapsed since this was costed, so the cost
    // in hand is not the cost of doing it now.
    if (version[a] != candidate.version_a || version[b] != candidate.version_b) {continue;}

    cv::Vec3f target;
    best_target(a, b, target);

    // Collapse b into a. Triangles that had both become degenerate and die;
    // the rest just refer to a instead.
    for (const int t : incident[b]) {
      if (dead_triangle[t]) {continue;}
      auto & tri = mesh.triangles[t];
      int hits = 0;
      for (int i = 0; i < 3; ++i) {
        if (tri[i] == b) {tri[i] = a;}
        if (tri[i] == a) {++hits;}
      }
      if (hits > 1) {
        dead_triangle[t] = true;
        --live_triangles;
      } else {
        incident[a].push_back(t);
      }
    }

    mesh.vertices[a] = target;
    // The surviving vertex takes the mean colour. A collapse merges two points on
    // a surface a few millimetres apart, so this is a blend of neighbours rather
    // than of anything distant.
    mesh.colours[a] = (mesh.colours[a] + mesh.colours[b]) * 0.5F;
    quadrics[a] += quadrics[b];
    dead_vertex[b] = true;
    collapsed_to[b] = a;
    ++version[a];
    ++collapses;

    // Re-cost every edge that now touches `a`. The version bump above is what
    // makes the entries already in the queue for those edges stale.
    //
    // **Two reused buffers rather than two containers built per collapse.** This
    // loop runs once per collapse and a room-sized mesh needs hundreds of
    // thousands of them; an `unordered_set<int>` constructed here is a heap
    // allocation, a bucket array and a node per neighbour, every time, for a set
    // that never holds more than a dozen entries. Measured as a re-mesh that did
    // not finish at all — the container logged nothing for 25 s and was killed
    // still working on its first extraction. A sort-and-unique over six elements
    // is free by comparison.
    neighbour_scratch.clear();
    live_scratch.clear();
    for (const int t : incident[a]) {
      if (dead_triangle[t]) {continue;}
      live_scratch.push_back(t);
      for (int i = 0; i < 3; ++i) {
        const int v = mesh.triangles[t][i];
        if (v != a && !dead_vertex[v]) {neighbour_scratch.push_back(v);}
      }
    }
    incident[a].swap(live_scratch);
    std::sort(neighbour_scratch.begin(), neighbour_scratch.end());
    neighbour_scratch.erase(
      std::unique(neighbour_scratch.begin(), neighbour_scratch.end()), neighbour_scratch.end());
    for (const int n : neighbour_scratch) {push_edge(a, n);}
  }

  std::vector<cv::Vec3i> survivors;
  survivors.reserve(live_triangles);
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    if (dead_triangle[t]) {continue;}
    cv::Vec3i tri = mesh.triangles[t];
    for (int i = 0; i < 3; ++i) {tri[i] = resolve(tri[i]);}
    if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2]) {continue;}
    survivors.push_back(tri);
  }
  mesh.triangles.swap(survivors);
  compact(mesh);
  return collapses;
}

}  // namespace pimesh_mapping
