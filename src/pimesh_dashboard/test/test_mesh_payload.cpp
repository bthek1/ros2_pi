// The layout the 3D view is decoded with, pinned byte for byte.
//
// **Nothing relates the two ends of this contract.** `pack_mesh` writes the
// bytes and `web/app.js` reads them back with hand-written offsets — `+ 4`,
// `count * 12`, `count * 3` — and neither file mentions the other. That is the
// `volume_key` failure shape, with one addition that makes it worse: a stride
// read one byte out still yields floats, so the page draws a triangle soup of
// believable magnitude in the wrong places. It looks like a poor
// reconstruction, and this project has spent two milestones expecting the mesh
// to look like a poor reconstruction.
//
// So the assertions here are literal offsets rather than a round trip through a
// decoder written beside the encoder. `test_mesh_io` is the precedent: a writer
// and a reader that are wrong together round-trip perfectly, and P6's evidence
// came out of exactly that pair.

#include <cstdint>
#include <cstring>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "gtest/gtest.h"
#include "pimesh_dashboard/mesh_payload.hpp"
#include "std_msgs/msg/color_rgba.hpp"

using pimesh_dashboard::kMeshColourStride;
using pimesh_dashboard::kMeshHeaderBytes;
using pimesh_dashboard::kMeshPositionStride;
using pimesh_dashboard::mesh_payload_size;
using pimesh_dashboard::pack_mesh;

namespace
{

geometry_msgs::msg::Point point(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std_msgs::msg::ColorRGBA colour(float r, float g, float b)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = 1.0F;
  return c;
}

/// One triangle, with distinct coordinates in every slot so a transposed or
/// interleaved write cannot pass by symmetry.
std::vector<geometry_msgs::msg::Point> one_triangle()
{
  return {point(1.0, 2.0, 3.0), point(4.0, 5.0, 6.0), point(-7.0, -8.0, -9.0)};
}

/// Read the payload the way `web/app.js` does, from the offsets written there,
/// not from the constants the packer used.
std::uint32_t header_count(const std::vector<std::uint8_t> & payload)
{
  std::uint32_t count = 0;
  std::memcpy(&count, payload.data(), 4);
  return count;
}

float position_at(const std::vector<std::uint8_t> & payload, std::size_t index)
{
  float value = 0.0F;
  std::memcpy(&value, payload.data() + 4 + index * 4, 4);
  return value;
}

std::uint8_t colour_at(
  const std::vector<std::uint8_t> & payload, std::uint32_t count, std::size_t index)
{
  return payload[4 + count * 12 + index];
}

}  // namespace

// ---------------------------------------------------------------------------
// The layout itself
// ---------------------------------------------------------------------------

TEST(MeshPayload, HasTheStridesTheBrowserAssumes)
{
  // These three numbers appear in web/app.js as the literals 4, 12 and 3. A
  // float that stopped being 4 bytes, or a colour promoted to float to "keep it
  // simple", would quadruple the payload and draw nothing.
  EXPECT_EQ(kMeshHeaderBytes, 4u);
  EXPECT_EQ(kMeshPositionStride, 12u);
  EXPECT_EQ(kMeshColourStride, 3u);
  EXPECT_EQ(sizeof(float), 4u) << "the wire format is float32 on both machines";
}

TEST(MeshPayload, IsExactlyAsLongAsTheHeaderPromises)
{
  // The page slices [4, 4 + count*12) and [4 + count*12, 4 + count*12 + count*3).
  // A payload one byte short of the second slice yields a zero-length colour
  // array and an untextured mesh; one byte long is silently ignored, so only the
  // exact length can be asserted.
  for (std::uint32_t triangles : {1u, 2u, 17u, 40000u}) {
    const std::uint32_t vertices = triangles * 3;
    std::vector<geometry_msgs::msg::Point> points(vertices, point(0.0, 0.0, 0.0));
    const auto payload = pack_mesh(points, {});
    ASSERT_EQ(payload.size(), mesh_payload_size(vertices));
    EXPECT_EQ(payload.size(), 4u + vertices * 12u + vertices * 3u);
    EXPECT_EQ(header_count(payload), vertices);
  }
}

TEST(MeshPayload, WritesTheCountLittleEndian)
{
  // The page asks DataView for `getUint32(0, true)`. Both machines are
  // little-endian, so a memcpy is correct and this test is what says the wire
  // format is little-endian rather than native — the distinction that matters
  // the day something big-endian reads this.
  std::vector<geometry_msgs::msg::Point> points(258, point(0.0, 0.0, 0.0));
  const auto payload = pack_mesh(points, {});
  EXPECT_EQ(payload[0], 0x02);
  EXPECT_EQ(payload[1], 0x01);
  EXPECT_EQ(payload[2], 0x00);
  EXPECT_EQ(payload[3], 0x00);
}

TEST(MeshPayload, PositionsAreXyzInOrderAndNotInterleavedWithColour)
{
  // Positions are one planar block and colours another. Interleaving them —
  // twelve bytes of position then three of colour, per vertex — is the obvious
  // alternative layout, it is the same total length, and the page would draw it
  // as noise. This is `test_depth_model`'s planes-not-interleaved assertion in a
  // different costume.
  const auto payload = pack_mesh(one_triangle(), {});
  ASSERT_EQ(header_count(payload), 3u);

  const float expected[9] = {1, 2, 3, 4, 5, 6, -7, -8, -9};
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_FLOAT_EQ(position_at(payload, i), expected[i]) << "float " << i;
  }
}

TEST(MeshPayload, ColoursAreBytesAfterEveryPosition)
{
  const auto payload = pack_mesh(
    one_triangle(),
    {colour(1.0F, 0.0F, 0.0F), colour(0.0F, 1.0F, 0.0F), colour(0.0F, 0.0F, 1.0F)});
  ASSERT_EQ(header_count(payload), 3u);

  const std::uint8_t expected[9] = {255, 0, 0, 0, 255, 0, 0, 0, 255};
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_EQ(colour_at(payload, 3, i), expected[i]) << "colour byte " << i;
  }
}

TEST(MeshPayload, ColourChannelsAreRgbAndNotBgr)
{
  // cv2's channel order has cost this workspace a test before (test_mesh_render).
  // WebGL wants RGB, and a red mesh rendering blue is a picture nobody looks at
  // twice.
  const auto payload = pack_mesh(one_triangle(), std::vector<std_msgs::msg::ColorRGBA>(
      3, colour(1.0F, 0.5F, 0.0F)));
  EXPECT_EQ(colour_at(payload, 3, 0), 255);            // r
  EXPECT_EQ(colour_at(payload, 3, 1), 127);            // g, 0.5 * 255 truncated
  EXPECT_EQ(colour_at(payload, 3, 2), 0);              // b
}

// ---------------------------------------------------------------------------
// The refusals
// ---------------------------------------------------------------------------

TEST(MeshPayload, ClampsBeforeTheCastRatherThanAfter)
{
  // **An out-of-range float cast to uint8_t wraps, it does not saturate**, so a
  // vertex at 1.2 would come out at (306 & 0xff) = 50: an over-bright triangle
  // rendering *dark*. Marker colours are 0..1 by convention and by nothing else.
  const auto payload = pack_mesh(one_triangle(), std::vector<std_msgs::msg::ColorRGBA>(
      3, colour(1.2F, -0.3F, 1.0F)));
  EXPECT_EQ(colour_at(payload, 3, 0), 255);
  EXPECT_EQ(colour_at(payload, 3, 1), 0);
  EXPECT_EQ(colour_at(payload, 3, 2), 255);
}

TEST(MeshPayload, GreysASurfaceThatArrivedWithoutColours)
{
  // Not black. An unlit black mesh and an empty canvas are the same picture, and
  // "the volume is empty" is a conclusion this project reaches often enough that
  // it must not be counterfeited by a missing colour array.
  const auto payload = pack_mesh(one_triangle(), {});
  EXPECT_EQ(colour_at(payload, 3, 0), 180);
  EXPECT_EQ(colour_at(payload, 3, 1), 180);
  EXPECT_EQ(colour_at(payload, 3, 2), 190);
  EXPECT_NE(colour_at(payload, 3, 0), 0) << "black is indistinguishable from no mesh";
}

TEST(MeshPayload, IgnoresAColourArrayThatDoesNotPairUp)
{
  // Some-but-not-all colours is a Marker this node does not understand. Using
  // them by index would colour the wrong vertices and leave the rest
  // uninitialised — which reads as a mesh with a corrupted patch.
  const auto payload = pack_mesh(one_triangle(), {colour(1.0F, 0.0F, 0.0F)});
  ASSERT_EQ(payload.size(), mesh_payload_size(3));
  for (std::size_t i = 0; i < 9; ++i) {
    EXPECT_EQ(colour_at(payload, 3, i), (i % 3 == 2) ? 190 : 180) << "byte " << i;
  }
}

TEST(MeshPayload, RefusesWhatIsNotAWholeNumberOfTriangles)
{
  // A TRIANGLE_LIST Marker with a count that is not a multiple of three is
  // malformed, and packing it would send a payload the page draws with its last
  // triangle reading past the vertices it was given.
  EXPECT_TRUE(pack_mesh({}, {}).empty());
  EXPECT_TRUE(pack_mesh({point(0, 0, 0)}, {}).empty());
  EXPECT_TRUE(pack_mesh({point(0, 0, 0), point(1, 1, 1)}, {}).empty());
  EXPECT_FALSE(pack_mesh(one_triangle(), {}).empty());
}

TEST(MeshPayload, AnEmptyRefusalIsNotAZeroVertexPayload)
{
  // The distinction the caller depends on: `on_mesh` returns without touching
  // `mesh_versions_` on an empty vector. A four-byte payload saying "zero
  // vertices" would instead *clear the page's mesh*, so a single malformed
  // Marker would blank a surface that was fine.
  EXPECT_EQ(pack_mesh({}, {}).size(), 0u);
  EXPECT_NE(pack_mesh({}, {}).size(), kMeshHeaderBytes);
}

TEST(MeshPayload, HandlesTheSizeTheMarkerCapActuallyAllows)
{
  // mesh_node decimates to 120 000 triangles for the Marker. That is 360 000
  // vertices at 15 bytes each: **5 400 004 bytes**, 64% of the server's 8 MiB
  // send limit. dashboard_node.cpp carried ~4.3 MB here until 2026-09-21, which
  // is the position array with the colour bytes left out; the margin is worth
  // stating correctly because exceeding the limit is a **silent drop** — the
  // frame is counted and discarded and the page simply keeps the old surface.
  const std::size_t vertices = 120000 * 3;
  std::vector<geometry_msgs::msg::Point> points(vertices, point(1.0, 2.0, 3.0));
  const auto payload = pack_mesh(points, {});
  ASSERT_EQ(payload.size(), mesh_payload_size(vertices));
  EXPECT_EQ(header_count(payload), vertices);
  EXPECT_LT(payload.size(), 8u * 1024u * 1024u) << "over the default send_limit_bytes";
  // The last vertex is intact: a length computed with a 32-bit intermediate
  // would truncate long before here, but the indices must also not.
  EXPECT_FLOAT_EQ(position_at(payload, vertices * 3 - 1), 3.0F);
  EXPECT_EQ(colour_at(payload, static_cast<std::uint32_t>(vertices), vertices * 3 - 1), 190);
}
