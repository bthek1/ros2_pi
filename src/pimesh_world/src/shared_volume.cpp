#include "pimesh_world/shared_volume.hpp"

#include <map>
#include <memory>
#include <string>

namespace pimesh_world
{
namespace
{

// Function-local statics rather than namespace-scope ones: this library is
// *dlopened* into a component container, and a namespace-scope static's
// initialisation order relative to the loader is a question nobody should have to
// answer. Function-local initialisation is guaranteed on first call and is
// thread-safe since C++11, which is the whole requirement here.
std::mutex & registry_mutex()
{
  static std::mutex mutex;
  return mutex;
}

std::map<std::string, std::shared_ptr<SharedVolume>> & registry()
{
  static std::map<std::string, std::shared_ptr<SharedVolume>> map;
  return map;
}

}  // namespace

std::shared_ptr<SharedVolume> VolumeRegistry::get(const std::string & key)
{
  std::lock_guard<std::mutex> lock(registry_mutex());
  auto & map = registry();
  auto it = map.find(key);
  if (it == map.end()) {
    it = map.emplace(key, std::make_shared<SharedVolume>()).first;
  }
  return it->second;
}

std::string VolumeRegistry::keys()
{
  std::lock_guard<std::mutex> lock(registry_mutex());
  std::string out;
  for (const auto & entry : registry()) {
    if (!out.empty()) {out += ", ";}
    out += entry.first;
    out += entry.second->configured() ? " (configured)" : " (not configured)";
  }
  return out.empty() ? std::string("none") : out;
}

}  // namespace pimesh_world
