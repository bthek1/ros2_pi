#ifndef PIMESH_WORLD__MESH_NODE_HPP_
#define PIMESH_WORLD__MESH_NODE_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "pimesh_msgs/msg/mesh_stats.hpp"
#include "pimesh_msgs/msg/pipeline_stats.hpp"
#include "pimesh_msgs/srv/reset_map.hpp"
#include "pimesh_msgs/srv/save_mesh.hpp"
#include "pimesh_world/mesh.hpp"
#include "pimesh_world/shared_volume.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace pimesh_world
{

/// Marching cubes over the volume, on a timer, without stalling the integrator.
///
/// **The whole design is that sentence's last clause.** Fusion integrates at
/// ~16 Hz against a 20 ms budget; extracting a surface from a room-sized volume
/// costs seconds. Those two cannot share a thread and they cannot share a lock
/// for the duration of the work, so:
///
///  - the re-mesh runs on its own thread, woken by a timer rather than done by
///    it, because a timer callback that takes four seconds blocks this node's
///    executor and every other callback it owns;
///  - it works on a **copy** of the block map, taken in chunks so the integrator
///    is never held out for more than a few milliseconds at a time (see
///    `SharedVolume::snapshot`), and marches over the copy holding no lock at
///    all.
///
/// `tools/gates/mesh.sh` asserts the integration rate shows no dip at mesh time
/// for exactly this reason — the failure mode is a mesh that is perfectly correct
/// and a pipeline that hitches every ten seconds, and nothing in either node's
/// log would say which of them was responsible.
///
/// **It needs a `fusion_node` in the same process and says so when it has not got
/// one.** The volume is hundreds of megabytes and its only consumer is here;
/// publishing it would be a serialisation of the entire map ten times a minute
/// between two components that share an address space. So they rendezvous through
/// `VolumeRegistry`, which is process-local, and a `mesh_node` started alone —
/// or in a different container, or under a different `volume_key` — refuses with
/// a message naming the key it was looking for. That is a real limitation and it
/// is the architecture doing what it says: `pipeline:=false` leaves the container
/// out entirely and there is no half-pipeline in between.
///
/// **Two meshes exist at once and they are not the same mesh.** `/world/mesh` is
/// capped at `max_triangles` by quadric decimation, because a `Marker` is rebuilt
/// and re-serialised on every publish and a million triangles is 120 MB of it.
/// `/world/save_mesh` writes the **full-detail** surface, because a file is
/// written once and read by something that can afford it. Saving the decimated
/// one would quietly throw away the detail the volume paid to accumulate.
class MeshNode : public rclcpp::Node
{
public:
  explicit MeshNode(const rclcpp::NodeOptions & options);
  ~MeshNode() override;

private:
  void request_remesh();
  void work();
  void remesh();
  void publish_marker(const Mesh & mesh, const rclcpp::Time & stamp);
  void publish_stats(
    const Mesh & mesh, const SharedVolume::Snapshot & snapshot,
    std::size_t voxels_meshed, double duration_ms, const rclcpp::Time & stamp);
  void on_save(
    const std::shared_ptr<pimesh_msgs::srv::SaveMesh::Request> request,
    std::shared_ptr<pimesh_msgs::srv::SaveMesh::Response> response);
  void on_reset(
    const std::shared_ptr<pimesh_msgs::srv::ResetMap::Request> request,
    std::shared_ptr<pimesh_msgs::srv::ResetMap::Response> response);

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
  rclcpp::Publisher<pimesh_msgs::msg::MeshStats>::SharedPtr stats_pub_;
  rclcpp::Publisher<pimesh_msgs::msg::PipelineStats>::SharedPtr pipeline_pub_;
  rclcpp::Service<pimesh_msgs::srv::SaveMesh>::SharedPtr save_service_;
  rclcpp::Service<pimesh_msgs::srv::ResetMap>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<SharedVolume> volume_;
  std::string volume_key_ {"world"};
  std::thread worker_;
  std::mutex wake_mutex_;
  std::condition_variable wake_;
  bool wanted_ {false};
  bool stopping_ {false};

  /// The full-detail surface from the last extraction, kept so that a save does
  /// not have to re-mesh. Guarded because the service callback reads it on an
  /// executor thread while the worker writes it.
  std::mutex mesh_mutex_;
  Mesh full_mesh_;

  std::string world_frame_ {"map"};
  std::string marker_ns_ {"world_mesh"};
  std::string save_dir_;
  float mesh_min_weight_ {3.0F};
  std::size_t min_component_triangles_ {30};
  double max_hole_radius_m_ {0.25};
  std::size_t max_triangles_ {120000};
  std::size_t snapshot_chunk_ {2048};
  int worker_nice_ {10};

  std::atomic<std::uint64_t> remeshes_ {0};
  std::atomic<double> last_duration_ms_ {0.0};
  std::atomic<bool> warned_no_volume_ {false};
};

}  // namespace pimesh_world

#endif  // PIMESH_WORLD__MESH_NODE_HPP_
