#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace agx_robot_task_runtime {

using StateRevision = std::uint64_t;
enum class ObjectIdentity { Registered, RunLocal };
struct ObjectRef {
  std::string id;
  ObjectIdentity identity = ObjectIdentity::RunLocal;
  std::string container_profile;
};
enum class HolderKind { Gripper, FixtureSlot };
struct HolderRef {
  HolderKind kind = HolderKind::Gripper;
  std::string fixture_instance, slot;
};
struct Evidence {
  std::string source;
  double source_stamp_s = 0.0;  // ROS source time, including simulated ROS time.
  double receipt_steady_s = 0.0;  // Monotonic receipt time, never a read time.
  std::string run_id, task_id, operation_id;
};
struct PendingTransfer {
  std::string operation_id;
  ObjectRef object;
  std::optional<HolderRef> source, destination;
};
enum class LocationCertainty { Unknown, Confirmed, Uncertain };
struct ObjectLocation {
  ObjectRef object;
  std::optional<HolderRef> last_confirmed_holder;
  std::optional<Evidence> last_confirmed_evidence;
  LocationCertainty certainty = LocationCertainty::Unknown;
  std::optional<Evidence> current_evidence;
  std::optional<PendingTransfer> pending;
  StateRevision revision = 0;
  bool fresh = false;
};
enum class GripperOccupancy { Unknown, Empty, Holding };
struct GripperView {
  GripperOccupancy occupancy = GripperOccupancy::Unknown;
  std::optional<ObjectRef> object;
  std::optional<Evidence> evidence;
  StateRevision revision = 0;
  bool fresh = false;
};
enum class TransportLoad { Unknown, Empty, SingleHeld, Unsupported };
enum class TravelEligibility { Unknown, CapturedInFixture, SafeClear, Unsupported };
struct TransportView {
  TransportLoad load = TransportLoad::Unknown;
  TravelEligibility eligibility = TravelEligibility::Unknown;
  std::optional<ObjectRef> object;
  std::optional<Evidence> custody_evidence, clearance_evidence;
  bool pending = false, uncertain = true, ready = false;
  StateRevision revision = 0;  // Semantic transport revision, stable across fresh observation refresh.
};
enum class EffectKind { GripperEmpty, GripperHolding, Released, ObjectAtHolder, TravelEligibility, GripperUncertain };
// Only verified physical observations belong here. Intent and task outcomes are not effects.
struct PhysicalEffect {
  std::string id;
  EffectKind kind = EffectKind::GripperEmpty;
  std::optional<ObjectRef> object;
  std::optional<HolderRef> holder;
  TravelEligibility eligibility = TravelEligibility::Unknown;
};
enum class UpdateStatus { Applied, Duplicate, Rejected, Conflict };
struct StateUpdate {
  UpdateStatus status = UpdateStatus::Rejected;
  StateRevision revision = 0;
  bool halt = false;
  std::string detail;
};
struct EffectRecord {
  PhysicalEffect effect;
  Evidence evidence;
  std::string operation_id;
  StateRevision expected_revision = 0, resulting_revision = 0;
  UpdateStatus status = UpdateStatus::Applied;
};
struct RobotStateSnapshot {
  std::map<std::string, ObjectLocation> objects;
  GripperView gripper;
  TransportView transport;
  std::vector<EffectRecord> effects;
  StateRevision revision = 0;
  bool halted = false;
};

class RobotStateStore {
 public:
  using Clock = std::function<double()>;
  // Production adapters must inject ROS time and std::chrono::steady_clock time.
  RobotStateStore(Clock source_clock, Clock steady_clock, double max_evidence_age_s = 1.0);
  // Returns a detached value. No reference to mutable store data escapes.
  RobotStateSnapshot Snapshot() const;
  StateUpdate BeginTransfer(StateRevision expected_revision, const std::string& operation_id,
                            const ObjectRef& object, std::optional<HolderRef> source,
                            std::optional<HolderRef> destination);
  StateUpdate ApplyEffect(const PhysicalEffect& effect, const Evidence& evidence,
                          const std::string& operation_id, StateRevision expected_revision);

 private:
  bool Fresh(const Evidence& evidence, double source_now, double steady_now) const;
  RobotStateSnapshot SnapshotLocked(double source_now, double steady_now) const;
  StateUpdate Conflict(const PhysicalEffect&, const Evidence&, const std::string& operation_id,
                       StateRevision expected_revision, const std::string& detail);
  Clock source_clock_, steady_clock_;
  double max_evidence_age_s_;
  mutable std::mutex mutex_;
  RobotStateSnapshot state_;
  std::map<std::string, Evidence> custody_high_water_;
  std::optional<Evidence> gripper_high_water_, clearance_high_water_;
  std::set<std::string> transfer_operations_;
};
}  // namespace agx_robot_task_runtime
