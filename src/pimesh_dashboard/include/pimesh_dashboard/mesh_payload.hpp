#ifndef PIMESH_DASHBOARD__MESH_PAYLOAD_HPP_
#define PIMESH_DASHBOARD__MESH_PAYLOAD_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/color_rgba.hpp"

namespace pimesh_dashboard
{

/// The one payload on this socket with a layout rather than a format, and the
/// only one whose reader is in a browser.
///
/// **The layout is agreed by two files that share no code**: this one packs it,
/// `web/app.js` unpacks it with hand-written offsets, and nothing relates them.
/// That is the `volume_key` shape CLAUDE.md names — two things that have to hold
/// the same value and agree only because somebody typed it twice — with the
/// added distinction that a mismatch here does not fail. A stride read one byte
/// out still produces floats, so the page draws a triangle soup of plausible
/// magnitude in the wrong places: a mesh that looks like a bad reconstruction
/// rather than like a bug, which is the worst possible way for it to be wrong,
/// because *this project already expects the mesh to look imperfect*.
///
/// It lived inside `DashboardNode::on_mesh` until 2026-09-21 — a member function
/// taking a `Marker` and ending in a `broadcast`, so reaching it from a test
/// meant standing up a node and a socket. Pure arithmetic over two vectors is
/// what it actually is.
///
/// The layout, which `test_mesh_payload` pins byte for byte:
///
/// ```text
///   offset 0      uint32  vertex count, little-endian
///   offset 4      float32 x,y,z per vertex   (12 bytes each)
///   offset 4+12n  uint8   r,g,b per vertex   (3 bytes each)
/// ```
///
/// No index array: a TRIANGLE_LIST Marker is already three points per triangle
/// with no reuse, so the indices would be 0,1,2,… and a third of the payload
/// saying nothing.

/// Bytes the page wants per vertex: three floats of position.
constexpr std::size_t kMeshPositionStride = 3 * sizeof(float);
/// Bytes the page wants per vertex: three bytes of colour, not three floats.
constexpr std::size_t kMeshColourStride = 3;
/// The little-endian `uint32` vertex count the payload opens with.
constexpr std::size_t kMeshHeaderBytes = 4;

/// Grey for a surface that arrived with no colours, matching `mesh_node`'s own
/// default rather than black — an unlit black mesh and an empty canvas are the
/// same picture.
constexpr std::uint8_t kMeshDefaultColour[3] = {180, 180, 190};

/// How long the payload for `vertices` vertices is.
inline std::size_t mesh_payload_size(std::size_t vertices)
{
  return kMeshHeaderBytes + vertices * (kMeshPositionStride + kMeshColourStride);
}

/// Pack a triangle soup into the page's wire layout.
///
/// Returns an empty vector for input the page cannot draw — no points, or a
/// count that is not a whole number of triangles. **Empty rather than partial**:
/// a truncated payload is read by `app.js` as however many vertices the header
/// claims, so half a triangle list would be drawn as a mesh with a torn edge,
/// which is again a plausible-looking wrong answer rather than a visible one.
///
/// `colours` is used only when there is exactly one per vertex. A Marker may
/// legally carry none, and a Marker carrying *some* is one this node does not
/// understand — pairing them up by index would silently colour the wrong
/// vertices.
inline std::vector<std::uint8_t> pack_mesh(
  const std::vector<geometry_msgs::msg::Point> & points,
  const std::vector<std_msgs::msg::ColorRGBA> & colours)
{
  const std::size_t vertices = points.size();
  if (vertices == 0 || vertices % 3 != 0) {return {};}

  std::vector<std::uint8_t> payload(mesh_payload_size(vertices));

  const std::uint32_t count = static_cast<std::uint32_t>(vertices);
  std::memcpy(payload.data(), &count, kMeshHeaderBytes);

  float * position = reinterpret_cast<float *>(payload.data() + kMeshHeaderBytes);
  std::uint8_t * colour =
    payload.data() + kMeshHeaderBytes + vertices * kMeshPositionStride;
  const bool have_colours = colours.size() == vertices;

  for (std::size_t i = 0; i < vertices; ++i) {
    position[i * 3 + 0] = static_cast<float>(points[i].x);
    position[i * 3 + 1] = static_cast<float>(points[i].y);
    position[i * 3 + 2] = static_cast<float>(points[i].z);
    if (have_colours) {
      // Marker colours are 0..1 floats and a Marker is not obliged to keep them
      // there. Clamping before the multiply is what stops a 1.2 becoming 51 —
      // an overflowing cast wraps rather than saturates, so an over-bright
      // vertex would come out *dark* and the mesh would be mottled.
      colour[i * 3 + 0] = static_cast<std::uint8_t>(std::clamp(colours[i].r, 0.0F, 1.0F) * 255.0F);
      colour[i * 3 + 1] = static_cast<std::uint8_t>(std::clamp(colours[i].g, 0.0F, 1.0F) * 255.0F);
      colour[i * 3 + 2] = static_cast<std::uint8_t>(std::clamp(colours[i].b, 0.0F, 1.0F) * 255.0F);
    } else {
      colour[i * 3 + 0] = kMeshDefaultColour[0];
      colour[i * 3 + 1] = kMeshDefaultColour[1];
      colour[i * 3 + 2] = kMeshDefaultColour[2];
    }
  }
  return payload;
}

}  // namespace pimesh_dashboard

#endif  // PIMESH_DASHBOARD__MESH_PAYLOAD_HPP_
