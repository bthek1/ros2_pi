#ifndef PIMESH_FRONTEND__TRACKING_STATE_HPP_
#define PIMESH_FRONTEND__TRACKING_STATE_HPP_

#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace pimesh_frontend
{

/// Whether the tracker trusts its pose (#13's P19).
enum class Tracking : std::uint8_t
{
  Ok,
  Lost,
};

inline const char * tracking_name(Tracking state)
{
  return state == Tracking::Ok ? "OK" : "LOST";
}

/// `OK` / `LOST`, decided from a run of per-frame outcomes.
///
/// **This is the difference between odometry and SLAM, and it is small on
/// purpose.** `odometry_node` already refuses a pose it cannot fit and holds the
/// last one, and that is the right behaviour for one frame: the camera barely
/// moved in 57 ms and the held pose is nearly right. Held for a second while the
/// camera keeps moving it is badly wrong, and `fusion_node` integrates every one of
/// those frames into voxels the TSDF cannot take back (`test_tsdf_volume` pins
/// that a surface moved past the truncation band leaves a ghost). So the tracker
/// says, explicitly, when it has stopped knowing where it is.
///
/// **On a run of holds, never on one.** `bags/desk1` holds ~20% of its depth
/// frames — scattered singles while the camera is moving fast, not stretches — so
/// a state that went LOST on the first hold would refuse a fifth of the room. It is
/// the *length* of the run that separates a missed frame from a blackout, which is
/// why the counter resets on every fit and does not accumulate.
///
/// **And it counts its own holds**, rather than reading the node's. `odometry_node`
/// keeps a consecutive-hold counter of its own for the stall rule, and that counter
/// is reset to zero every time the stall rule takes a new reference view — every
/// `max_hold_frames`, 5 by default. Driven from that counter, a `lost_after_holds`
/// above 5 would be a state that can never be entered, and a never-entered LOST has
/// a perfect record: no bad frame fused, ever.
///
/// Recovery needs `recover_after_fits` fits in a row, so that one lucky fit against
/// a reference taken in the dark does not reopen the volume.
///
/// **With a loaded map (#13's P20) fits never recover at all.** OK then means "posed
/// in the saved map", and a fresh session's odom has no relation to that map until a
/// relocalisation supplies one — so the monitor starts LOST, and leaves it only
/// through `relocalised()`. The same holds after a later blackout: the tracker's
/// odometric recovery resumes from a held pose, and the `map <- odom` it would be
/// read through is stale by however far the camera moved in the dark.
///
/// No ROS here, so the transitions are a unit test that runs on both machines.
class TrackingMonitor
{
public:
  struct Config
  {
    std::size_t lost_after_holds {5};
    std::size_t recover_after_fits {2};
    /// #13's P20: start LOST, and leave it only through relocalised().
    bool recover_by_relocalisation {false};
  };

  explicit TrackingMonitor(Config config)
  : config_(config)
  {
    // A threshold of zero is either LOST from the first frame or OK on no evidence,
    // and both are a state that reads as information and carries none.
    if (config_.lost_after_holds == 0 || config_.recover_after_fits == 0) {
      throw std::invalid_argument("TrackingMonitor thresholds must be at least 1");
    }
    if (config_.recover_by_relocalisation) {state_ = Tracking::Lost;}
  }

  /// One depth frame: `posed` is whether its pose came from a fit. Returns true when
  /// this frame changed the state.
  bool observe(bool posed)
  {
    ++frames_;
    if (posed) {
      ++fits_;
      holds_ = 0;
    } else {
      ++holds_;
      fits_ = 0;
      if (holds_ > longest_hold_run_) {longest_hold_run_ = holds_;}
    }

    bool changed = false;
    if (state_ == Tracking::Ok && holds_ >= config_.lost_after_holds) {
      state_ = Tracking::Lost;
      ++entered_lost_;
      changed = true;
    } else if (state_ == Tracking::Lost && !config_.recover_by_relocalisation &&
      fits_ >= config_.recover_after_fits)
    {
      state_ = Tracking::Ok;
      ++recovered_;
      changed = true;
    }
    if (state_ == Tracking::Lost) {++lost_frames_;}
    return changed;
  }

  /// A relocalisation was accepted. Leaves LOST and returns true; does nothing and
  /// returns false when already OK — a late answer to a query asked while LOST must
  /// not move a map frame that has since been fixed.
  bool relocalised()
  {
    if (state_ != Tracking::Lost) {return false;}
    state_ = Tracking::Ok;
    ++recovered_;
    ++relocalisations_;
    holds_ = 0;
    return true;
  }

  Tracking state() const {return state_;}
  std::uint64_t relocalisations() const {return relocalisations_;}
  /// Consecutive frames without a fit, ending at the latest. 0 after a fit.
  std::size_t holds() const {return holds_;}
  /// Consecutive fits, ending at the latest. 0 after a hold.
  std::size_t fits() const {return fits_;}
  std::uint64_t frames() const {return frames_;}
  std::uint64_t entered_lost() const {return entered_lost_;}
  std::uint64_t recovered() const {return recovered_;}
  /// Frames observed while LOST, including the one that entered it.
  std::uint64_t lost_frames() const {return lost_frames_;}
  std::size_t longest_hold_run() const {return longest_hold_run_;}
  const Config & config() const {return config_;}

private:
  Config config_;
  Tracking state_ {Tracking::Ok};
  std::size_t holds_ {0};
  std::size_t fits_ {0};
  std::size_t longest_hold_run_ {0};
  std::uint64_t frames_ {0};
  std::uint64_t entered_lost_ {0};
  std::uint64_t recovered_ {0};
  std::uint64_t lost_frames_ {0};
  std::uint64_t relocalisations_ {0};
};

}  // namespace pimesh_frontend

#endif  // PIMESH_FRONTEND__TRACKING_STATE_HPP_
