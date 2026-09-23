// PLY in and out, hand-written.
//
// Written by hand rather than through a library for the reason this project
// hand-writes every format it needs exactly one direction of: it is sixty lines,
// it has no dependency a unit test would have to have installed, and the
// alternative is a library on the save path of a node that must not stall.
//
// **Binary little-endian, not ASCII.** A full-detail room mesh is millions of
// triangles; ASCII is about five times the bytes and considerably slower to
// write, and the saved file is the full-detail one — the 120 k cap belongs to the
// Marker, which is rebuilt and re-serialised on every publish, not to the
// geometry.
//
// Written to a temporary beside the destination and renamed into place, so an
// interrupted save leaves the previous file rather than half of a new one. A mesh
// somebody asked for and did not get is better than one that opens and is wrong.

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "pimesh_mapping/mesh.hpp"

namespace pimesh_mapping
{
namespace
{

std::uint8_t to_byte(float channel)
{
  const float scaled = channel * 255.0F + 0.5F;
  if (scaled <= 0.0F) {return 0;}
  if (scaled >= 255.0F) {return 255;}
  return static_cast<std::uint8_t>(scaled);
}

}  // namespace

bool write_ply(const Mesh & mesh, const std::string & path, std::string & error)
{
  error.clear();
  if (mesh.vertices.size() != mesh.colours.size()) {
    error = "mesh has " + std::to_string(mesh.vertices.size()) + " vertices and " +
      std::to_string(mesh.colours.size()) + " colours";
    return false;
  }

  const std::string temporary = path + ".partial";
  {
    std::ofstream out(temporary, std::ios::binary);
    if (!out) {
      error = "cannot open " + temporary + " for writing";
      return false;
    }

    out << "ply\n"
        << "format binary_little_endian 1.0\n"
        << "comment written by pimesh_mapping\n"
        << "element vertex " << mesh.vertices.size() << "\n"
        << "property float x\nproperty float y\nproperty float z\n"
        << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        << "element face " << mesh.triangles.size() << "\n"
        << "property list uchar int vertex_indices\n"
        << "end_header\n";

    // 15 bytes a vertex, staged through one buffer rather than a stream insertion
    // per field: at a million vertices the difference is seconds.
    std::vector<char> buffer;
    buffer.reserve(mesh.vertices.size() * 15);
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
      const float xyz[3] = {mesh.vertices[i][0], mesh.vertices[i][1], mesh.vertices[i][2]};
      const char * bytes = reinterpret_cast<const char *>(xyz);
      buffer.insert(buffer.end(), bytes, bytes + sizeof(xyz));
      // PLY calls them red/green/blue and Mesh::colours is in that order too —
      // the bgr8 swap happens once, in march_cubes, where the voxel is read.
      buffer.push_back(static_cast<char>(to_byte(mesh.colours[i][0])));
      buffer.push_back(static_cast<char>(to_byte(mesh.colours[i][1])));
      buffer.push_back(static_cast<char>(to_byte(mesh.colours[i][2])));
    }
    out.write(buffer.data(), static_cast<std::streamsize>(buffer.size()));

    buffer.clear();
    buffer.reserve(mesh.triangles.size() * 13);
    for (const auto & tri : mesh.triangles) {
      buffer.push_back(3);
      const int indices[3] = {tri[0], tri[1], tri[2]};
      const char * bytes = reinterpret_cast<const char *>(indices);
      buffer.insert(buffer.end(), bytes, bytes + sizeof(indices));
    }
    out.write(buffer.data(), static_cast<std::streamsize>(buffer.size()));

    out.flush();
    if (!out) {
      error = "write failed on " + temporary;
      std::remove(temporary.c_str());
      return false;
    }
  }

  if (std::rename(temporary.c_str(), path.c_str()) != 0) {
    error = "cannot rename " + temporary + " to " + path;
    std::remove(temporary.c_str());
    return false;
  }
  return true;
}

bool read_ply(const std::string & path, Mesh & mesh, std::string & error)
{
  error.clear();
  mesh.clear();

  std::ifstream in(path, std::ios::binary);
  if (!in) {
    error = "cannot open " + path;
    return false;
  }

  std::string line;
  std::size_t vertices = 0;
  std::size_t faces = 0;
  bool binary_le = false;
  bool in_header = true;
  std::string element;

  while (in_header && std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {line.pop_back();}
    std::istringstream fields(line);
    std::string token;
    fields >> token;
    if (token == "format") {
      std::string format;
      fields >> format;
      binary_le = (format == "binary_little_endian");
    } else if (token == "element") {
      fields >> element;
      if (element == "vertex") {
        fields >> vertices;
      } else if (element == "face") {
        fields >> faces;
      }
    } else if (token == "end_header") {
      in_header = false;
    }
  }

  if (in_header) {
    error = path + " has no end_header";
    return false;
  }
  if (!binary_le) {
    // Deliberately narrow: this reads back what write_ply produces and nothing
    // else. A partial reader that silently mis-parses another PLY dialect would
    // make the gate assert on numbers it invented.
    error = path + " is not binary_little_endian, which is the only form this reads";
    return false;
  }

  mesh.vertices.resize(vertices);
  mesh.colours.resize(vertices);
  for (std::size_t i = 0; i < vertices; ++i) {
    float xyz[3];
    std::uint8_t rgb[3];
    in.read(reinterpret_cast<char *>(xyz), sizeof(xyz));
    in.read(reinterpret_cast<char *>(rgb), sizeof(rgb));
    if (!in) {
      error = path + " ended after " + std::to_string(i) + " of " +
        std::to_string(vertices) + " vertices";
      return false;
    }
    mesh.vertices[i] = cv::Vec3f(xyz[0], xyz[1], xyz[2]);
    mesh.colours[i] = cv::Vec3f(
      static_cast<float>(rgb[0]) / 255.0F,
      static_cast<float>(rgb[1]) / 255.0F,
      static_cast<float>(rgb[2]) / 255.0F);
  }

  mesh.triangles.resize(faces);
  for (std::size_t i = 0; i < faces; ++i) {
    std::uint8_t count = 0;
    in.read(reinterpret_cast<char *>(&count), 1);
    if (!in || count != 3) {
      error = path + " has a face with " + std::to_string(count) +
        " vertices; only triangles are written";
      return false;
    }
    int indices[3];
    in.read(reinterpret_cast<char *>(indices), sizeof(indices));
    if (!in) {
      error = path + " ended after " + std::to_string(i) + " of " +
        std::to_string(faces) + " faces";
      return false;
    }
    mesh.triangles[i] = cv::Vec3i(indices[0], indices[1], indices[2]);
  }
  return true;
}

}  // namespace pimesh_mapping
