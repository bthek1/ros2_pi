// The saved surface, checked against the bytes and not only against itself.
//
// **A round trip is the test that cannot see a shared mistake**, and this file
// exists because everything downstream of `write_ply` reads the file through
// `read_ply`. Swap x and z in both, put colour before position in both, claim
// `binary_little_endian` and write native doubles in both — every one of those
// round-trips perfectly and produces a file that MeshLab, Blender and the next
// person's script open as a fan of garbage. `tools/gates/mesh.sh` would report a
// healthy triangle count throughout, because it asks `read_ply`.
//
// So the writer is also read here by hand, out of the raw bytes, against the
// layout the header promises: 15 bytes a vertex as three little-endian floats
// then three bytes of colour, 13 bytes a face as a literal 3 then three
// little-endian ints. That is the one assertion in this file with an opinion
// from outside the pair.
//
// The rest is refusals. `read_ply` is a *gate instrument* — P6's evidence is a
// triangle count taken off a file — and the dangerous failure is not a crash but
// a reader that returns true having understood less than it thinks: a truncated
// file read as a short mesh, or an ASCII PLY whose digits are read as floats.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include "gtest/gtest.h"
#include "pimesh_world/mesh.hpp"

using pimesh_world::Mesh;
using pimesh_world::read_ply;
using pimesh_world::write_ply;

namespace
{

std::string temp_path(const char * tag)
{
  static int counter = 0;
  return std::string("/tmp/pimesh_test_") + tag + "_" +
         std::to_string(static_cast<long>(getpid())) + "_" +
         std::to_string(++counter) + ".ply";
}

/// An `n` by `n` grid triangulated into a flat sheet, with a colour per vertex.
///
/// The colours are a ramp rather than a constant: a writer that emitted the same
/// vertex's colour for every vertex would satisfy a constant-coloured mesh
/// exactly, and a room scan is mostly one colour.
Mesh sheet(int n)
{
  Mesh mesh;
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i < n; ++i) {
      mesh.vertices.emplace_back(
        static_cast<float>(i) * 0.05F, static_cast<float>(j) * 0.05F, 0.25F);
      mesh.colours.emplace_back(
        static_cast<float>(i) / static_cast<float>(n),
        static_cast<float>(j) / static_cast<float>(n),
        0.5F);
    }
  }
  for (int j = 0; j + 1 < n; ++j) {
    for (int i = 0; i + 1 < n; ++i) {
      const int a = j * n + i;
      mesh.triangles.emplace_back(a, a + 1, a + n);
      mesh.triangles.emplace_back(a + 1, a + n + 1, a + n);
    }
  }
  return mesh;
}

std::vector<char> slurp(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  return std::vector<char>((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

/// Byte offset one past `end_header\n`, or 0 if it is not there.
std::size_t payload_offset(const std::vector<char> & bytes)
{
  static const std::string marker = "end_header\n";
  const std::string text(bytes.begin(), bytes.end());
  const std::size_t at = text.find(marker);
  return at == std::string::npos ? 0 : at + marker.size();
}

/// Read a little-endian value out of the payload without going through read_ply.
///
/// Assembled byte by byte rather than memcpy'd, so that this is a statement about
/// *the file's* byte order rather than about this machine's. On a big-endian
/// build the writer and reader would agree with each other and disagree with
/// every other PLY tool in the world; this is the check that would notice.
template<typename T>
T little_endian_at(const std::vector<char> & bytes, std::size_t offset)
{
  std::uint64_t raw = 0;
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    raw |= static_cast<std::uint64_t>(static_cast<unsigned char>(bytes[offset + i])) << (8 * i);
  }
  T out;
  std::memcpy(&out, &raw, sizeof(T));
  return out;
}

constexpr std::size_t kVertexBytes = 3 * sizeof(float) + 3;   // xyz + rgb
constexpr std::size_t kFaceBytes = 1 + 3 * sizeof(std::int32_t);

}  // namespace

// --- The bytes ---------------------------------------------------------------

TEST(PlyBytes, TheFileIsExactlyTheLayoutItsHeaderPromises)
{
  const Mesh mesh = sheet(4);
  const std::string path = temp_path("layout");
  std::string error;
  ASSERT_TRUE(write_ply(mesh, path, error)) << error;

  const auto bytes = slurp(path);
  const std::size_t payload = payload_offset(bytes);
  ASSERT_GT(payload, 0U) << "no end_header — nothing else in this test means anything";

  // Size first: a stray field, a padding byte or a forgotten count shows here as
  // a number that is off by a multiple of the element count, which is far easier
  // to read than the garbage it would produce downstream.
  EXPECT_EQ(
    bytes.size(),
    payload + kVertexBytes * mesh.vertices.size() + kFaceBytes * mesh.triangles.size())
    << "the payload is not 15 bytes a vertex and 13 bytes a face";

  // Then the first vertex, assembled from the file's own bytes.
  EXPECT_FLOAT_EQ(little_endian_at<float>(bytes, payload + 0), mesh.vertices[0][0]);
  EXPECT_FLOAT_EQ(little_endian_at<float>(bytes, payload + 4), mesh.vertices[0][1]);
  EXPECT_FLOAT_EQ(little_endian_at<float>(bytes, payload + 8), mesh.vertices[0][2]);
  EXPECT_EQ(static_cast<unsigned char>(bytes[payload + 12]), 0U) << "red of a 0.0 channel";

  // And the first face: the literal 3 that says "this is a triangle", then three
  // little-endian ints. A writer that omitted the 3 would shift every index by a
  // byte, which read back through its own reader would still be self-consistent.
  const std::size_t faces_at = payload + kVertexBytes * mesh.vertices.size();
  EXPECT_EQ(static_cast<unsigned char>(bytes[faces_at]), 3U);
  EXPECT_EQ(little_endian_at<std::int32_t>(bytes, faces_at + 1), mesh.triangles[0][0]);
  EXPECT_EQ(little_endian_at<std::int32_t>(bytes, faces_at + 5), mesh.triangles[0][1]);
  EXPECT_EQ(little_endian_at<std::int32_t>(bytes, faces_at + 9), mesh.triangles[0][2]);

  std::remove(path.c_str());
}

TEST(PlyBytes, TheHeaderCountsAreTheRealCounts)
{
  const Mesh mesh = sheet(5);
  const std::string path = temp_path("counts");
  std::string error;
  ASSERT_TRUE(write_ply(mesh, path, error)) << error;

  const auto bytes = slurp(path);
  const std::string header(bytes.begin(), bytes.begin() + static_cast<long>(payload_offset(bytes)));

  EXPECT_NE(header.find("format binary_little_endian 1.0\n"), std::string::npos);
  EXPECT_NE(
    header.find("element vertex " + std::to_string(mesh.vertices.size()) + "\n"),
    std::string::npos) << header;
  EXPECT_NE(
    header.find("element face " + std::to_string(mesh.triangles.size()) + "\n"),
    std::string::npos) << header;
  // Vertex element before face element: PLY payloads are in header order, and a
  // file whose header declares them the other way round is read by any correct
  // reader as faces first — which is 13 bytes of vertex data per "face".
  EXPECT_LT(header.find("element vertex"), header.find("element face"));
}

// --- The round trip ----------------------------------------------------------

TEST(Ply, WhatIsWrittenIsWhatComesBack)
{
  Mesh mesh = sheet(5);
  mesh.colours[0] = cv::Vec3f(1.0F, 0.0F, 0.0F);
  mesh.colours[1] = cv::Vec3f(0.0F, 1.0F, 0.0F);
  mesh.colours[2] = cv::Vec3f(0.0F, 0.0F, 1.0F);

  const std::string path = temp_path("roundtrip");
  std::string error;
  ASSERT_TRUE(write_ply(mesh, path, error)) << error;

  Mesh back;
  ASSERT_TRUE(read_ply(path, back, error)) << error;
  ASSERT_EQ(back.vertices.size(), mesh.vertices.size());
  ASSERT_EQ(back.triangles.size(), mesh.triangles.size());

  for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
    EXPECT_NEAR(back.vertices[i][0], mesh.vertices[i][0], 1e-6);
    EXPECT_NEAR(back.vertices[i][1], mesh.vertices[i][1], 1e-6);
    EXPECT_NEAR(back.vertices[i][2], mesh.vertices[i][2], 1e-6);
    // 8-bit round trip, so within half a level either way.
    for (int c = 0; c < 3; ++c) {
      EXPECT_NEAR(back.colours[i][c], mesh.colours[i][c], 1.0 / 255.0)
        << "vertex " << i << " channel " << c
        << " — a channel swap here paints the whole room the wrong colour";
    }
  }
  for (std::size_t i = 0; i < mesh.triangles.size(); ++i) {
    EXPECT_EQ(back.triangles[i], mesh.triangles[i]);
  }
  std::remove(path.c_str());
}

TEST(Ply, AnEmptyMeshIsAFileAndNotAFailure)
{
  // `mesh_node` saves whatever it has. An empty volume is a legitimate state —
  // nothing swept yet — and it must produce a readable file with nothing in it
  // rather than an error, so that "no surface" and "the save broke" stay
  // distinguishable in the gate's output.
  const Mesh mesh;
  const std::string path = temp_path("empty");
  std::string error;
  ASSERT_TRUE(write_ply(mesh, path, error)) << error;

  Mesh back;
  ASSERT_TRUE(read_ply(path, back, error)) << error;
  EXPECT_TRUE(back.empty());
  EXPECT_EQ(back.vertices.size(), 0U);
  std::remove(path.c_str());
}

TEST(Ply, ReadingIntoAMeshReplacesItRatherThanAppending)
{
  // The reader is handed a caller's Mesh, and `mesh_node` reuses one. Without
  // the clear the second read returns the first surface with the second welded
  // on to it: twice the triangles, every index in the second half pointing at
  // the wrong vertices, and a gate counting triangles that would call it growth.
  const std::string path = temp_path("reuse");
  std::string error;
  ASSERT_TRUE(write_ply(sheet(4), path, error)) << error;

  Mesh back;
  ASSERT_TRUE(read_ply(path, back, error)) << error;
  const std::size_t once = back.triangles.size();
  ASSERT_GT(once, 0U);

  ASSERT_TRUE(read_ply(path, back, error)) << error;
  EXPECT_EQ(back.triangles.size(), once) << "the second read appended to the first";
  EXPECT_EQ(back.vertices.size(), back.colours.size());

  // And a read that *fails* leaves the caller holding nothing, not the previous
  // surface. This is the half that the resizes above cannot stand in for — they
  // shrink the mesh to the new header's counts, so a same-sized re-read looks
  // correct whether or not anything was cleared. A caller that logs the error and
  // carries on would otherwise re-mesh, re-save and re-publish geometry from a
  // file it just failed to read, with every count looking healthy.
  const std::string nonsense = temp_path("nonsense");
  {
    std::ofstream out(nonsense);
    out << "not a ply at all\n";
  }
  EXPECT_FALSE(read_ply(nonsense, back, error));
  EXPECT_TRUE(back.empty()) << "a failed read left the previous surface in place";
  EXPECT_EQ(back.vertices.size(), 0U);
  std::remove(nonsense.c_str());
  std::remove(path.c_str());
}

TEST(Ply, ColourIsClampedRatherThanWrapped)
{
  // march_cubes should never produce a channel outside [0, 1], and if it ever
  // does, the difference between clamping and wrapping is a wall that goes
  // *black* at its brightest. 255.0F + 0.5F truncated into a uint8 is 0.
  Mesh mesh = sheet(2);
  mesh.colours[0] = cv::Vec3f(1.5F, -0.5F, 1.0F);
  mesh.colours[1] = cv::Vec3f(1.0F, 0.0F, 0.5F);

  const std::string path = temp_path("clamp");
  std::string error;
  ASSERT_TRUE(write_ply(mesh, path, error)) << error;

  const auto bytes = slurp(path);
  const std::size_t payload = payload_offset(bytes);
  EXPECT_EQ(static_cast<unsigned char>(bytes[payload + 12]), 255U) << "1.5 wrapped";
  EXPECT_EQ(static_cast<unsigned char>(bytes[payload + 13]), 0U) << "-0.5 wrapped";
  EXPECT_EQ(static_cast<unsigned char>(bytes[payload + 14]), 255U);
  // Rounds to nearest rather than truncating: 0.5 * 255 + 0.5 = 128.
  EXPECT_EQ(static_cast<unsigned char>(bytes[payload + kVertexBytes + 14]), 128U);
  std::remove(path.c_str());
}

// --- Refusals ----------------------------------------------------------------

TEST(Ply, AMeshWhoseColoursDoNotMatchItsVerticesIsRefused)
{
  // Every vertex has a colour, always — it is the invariant Mesh documents and
  // the one thing the writer cannot paper over, because the payload is
  // interleaved. Writing the shorter of the two would read back as a mesh whose
  // geometry stops early; writing past the end is a buffer overrun.
  Mesh mesh = sheet(3);
  mesh.colours.pop_back();

  const std::string path = temp_path("mismatch");
  std::string error;
  EXPECT_FALSE(write_ply(mesh, path, error));
  EXPECT_NE(error.find("colours"), std::string::npos) << error;
  EXPECT_FALSE(std::ifstream(path).good()) << "a refused write left a file behind";
}

TEST(Ply, AnUnwritablePathFailsAndLeavesNothing)
{
  Mesh mesh = sheet(3);
  std::string error;
  EXPECT_FALSE(write_ply(mesh, "/definitely/not/a/directory/x.ply", error));
  EXPECT_FALSE(error.empty()) << "a refusal has to say why — the service returns this";
  EXPECT_FALSE(std::ifstream("/definitely/not/a/directory/x.ply.partial").good());
}

TEST(Ply, ASuccessfulWriteLeavesNoPartialBehind)
{
  // The save is written to <path>.partial and renamed into place, so that an
  // interrupted save leaves the previous mesh rather than half of a new one. The
  // rename is the part that is easy to lose: a writer that opened the
  // destination directly passes every other test in this file.
  const std::string path = temp_path("partial");
  std::string error;
  ASSERT_TRUE(write_ply(sheet(3), path, error)) << error;

  EXPECT_TRUE(std::ifstream(path).good());
  EXPECT_FALSE(std::ifstream(path + ".partial").good()) << "the temporary was not renamed";
  std::remove(path.c_str());
}

TEST(Ply, AFailedRenameCleansUpItsTemporary)
{
  // The other half of the save: the rename can fail after every byte is safely
  // written. A destination that is a non-empty directory is the portable way to
  // force it — no permissions, no root question, no filesystem assumptions.
  //
  // **What neither this nor the test above asserts** is the claim the mechanism
  // exists for: that a save killed *mid-write* leaves the previous mesh intact.
  // Interrupting a write takes a signal, which is a gate's instrument and not a
  // unit test's. What is covered here is that a failure at either end leaves no
  // half-file lying around with a mesh's name on it.
  const std::string dir = temp_path("renamefail");
  ASSERT_EQ(mkdir(dir.c_str(), 0755), 0);
  {
    std::ofstream occupant(dir + "/occupied");
    occupant << "x";
  }

  std::string error;
  EXPECT_FALSE(write_ply(sheet(3), dir, error));
  EXPECT_FALSE(error.empty());
  EXPECT_FALSE(std::ifstream(dir + ".partial").good()) << "the temporary outlived its failure";

  std::remove((dir + "/occupied").c_str());
  rmdir(dir.c_str());
}

TEST(Ply, AnAsciiPlyIsRefusedRatherThanMisread)
{
  // The dangerous one, and the reason `read_ply` checks the format line at all.
  // An ASCII PLY has a perfectly valid header, so a reader that skipped to
  // end_header and started reading floats would consume the digits "0.1 0.2" as
  // four bytes of IEEE-754 and return `true` with a mesh made of 1e-38s. The gate
  // would print a triangle count for it.
  const std::string path = temp_path("ascii");
  {
    std::ofstream out(path);
    out << "ply\nformat ascii 1.0\n"
        << "element vertex 1\n"
        << "property float x\nproperty float y\nproperty float z\n"
        << "element face 0\nproperty list uchar int vertex_indices\n"
        << "end_header\n0.1 0.2 0.3\n";
  }
  Mesh mesh;
  std::string error;
  EXPECT_FALSE(read_ply(path, mesh, error));
  EXPECT_NE(error.find("binary_little_endian"), std::string::npos) << error;
  EXPECT_TRUE(mesh.empty());
  std::remove(path.c_str());
}

TEST(Ply, ATruncatedFileIsRefusedRatherThanReadShort)
{
  // A save killed part way through, or a disk that filled. The reader must not
  // return the triangles it managed — P6's evidence is a count off this file, and
  // a short read that reported success is a smaller mesh with nothing anywhere
  // saying so.
  const std::string path = temp_path("truncated");
  std::string error;
  ASSERT_TRUE(write_ply(sheet(5), path, error)) << error;

  const auto bytes = slurp(path);
  const std::size_t payload = payload_offset(bytes);
  ASSERT_GT(bytes.size(), payload + 40U);

  // Both blocks, because they are two separate checks in the reader and a cut at
  // the end of the file only ever reaches the second one. Measured 2026-09-19: a
  // reader that accepts a short *vertex* read passes this test when the only
  // truncation tried lands among the faces.
  // Each cut names the block it lands in, and the error has to name the same one.
  // Refusing is not enough on its own: measured 2026-09-19, a reader that ran on
  // past a short *vertex* block still returned false — from the face loop, a few
  // lines later, complaining about faces. Same answer, and it sends whoever reads
  // the log to the wrong half of the file. (It is also a real difference in
  // behaviour where a mesh has vertices and no faces: there is no second loop to
  // catch it, and the read succeeds short.)
  const struct {std::size_t cut; const char * names;} cases[] = {
    {payload + 4, "vertices"},
    {bytes.size() - 20, "faces"},
  };
  for (const auto & one : cases) {
    {
      std::ofstream out(path, std::ios::binary | std::ios::trunc);
      out.write(bytes.data(), static_cast<std::streamsize>(one.cut));
    }
    Mesh mesh;
    std::string why;
    EXPECT_FALSE(read_ply(path, mesh, why)) << "read " << one.cut << " bytes happily";
    EXPECT_NE(why.find(std::string("ended after")), std::string::npos) << why;
    EXPECT_NE(why.find(std::string(" ") + one.names), std::string::npos)
      << "cut inside the " << one.names << " block, but the reader said: " << why;
  }
  std::remove(path.c_str());
}

TEST(Ply, AFaceThatIsNotATriangleIsRefused)
{
  // Quads are legal PLY and this reader does not do them — `Mesh::triangles` is
  // three indices wide. Reading a quad's leading 4 as a 3 would take three of its
  // four indices and then interpret the fourth as the *next* face's vertex count,
  // shredding everything after it into a plausible-looking mesh.
  const std::string path = temp_path("quad");
  {
    std::ofstream out(path, std::ios::binary);
    out << "ply\nformat binary_little_endian 1.0\n"
        << "element vertex 0\n"
        << "property float x\nproperty float y\nproperty float z\n"
        << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        << "element face 1\nproperty list uchar int vertex_indices\n"
        << "end_header\n";
    const unsigned char count = 4;
    const std::int32_t indices[4] = {0, 1, 2, 3};
    out.write(reinterpret_cast<const char *>(&count), 1);
    out.write(reinterpret_cast<const char *>(indices), sizeof(indices));
  }
  Mesh mesh;
  std::string error;
  EXPECT_FALSE(read_ply(path, mesh, error));
  EXPECT_NE(error.find("only triangles"), std::string::npos) << error;
  std::remove(path.c_str());
}

TEST(Ply, ReadingSomethingThatIsNotAPlyFails)
{
  const std::string path = temp_path("notaply");
  {
    std::ofstream out(path);
    out << "this is not a ply file\n";
  }
  Mesh mesh;
  std::string error;
  EXPECT_FALSE(read_ply(path, mesh, error));
  EXPECT_FALSE(error.empty());
  std::remove(path.c_str());
}

TEST(Ply, ReadingAFileThatIsNotThereFails)
{
  Mesh mesh;
  std::string error;
  EXPECT_FALSE(read_ply("/tmp/pimesh_no_such_mesh_at_all.ply", mesh, error));
  EXPECT_FALSE(error.empty());
}
