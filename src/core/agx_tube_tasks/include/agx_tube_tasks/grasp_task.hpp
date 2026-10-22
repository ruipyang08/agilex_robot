#pragma once
#include <array>
#include <cstddef>
#include <memory>
#include <mutex>
#include "agx_robot_task_runtime/robot_state.hpp"
#include "agx_task_workflow/runner.hpp"
#include "agx_xarm_pick/tube_effects.hpp"

namespace agx_tube_tasks {
// Passive detached observations. Application and snapshot are separate reads:
// a later snapshot must never replace the actual update returned by the store.
struct GraspEffectObservation {
  std::uint64_t observation_sequence = 0;
  std::string run_id, task_id, operation_id;
  agx_robot_task_runtime::ObjectRef object;
  agxcore::TubeEffect source_event;
  std::optional<agx_robot_task_runtime::StateUpdate> application;
  bool adapter_accepted = false;
  agx_robot_task_runtime::StateRevision snapshot_revision = 0;
  bool snapshot_halted = false;
  agx_robot_task_runtime::GripperView gripper;
  std::optional<agx_robot_task_runtime::ObjectLocation> object_location;
  agx_robot_task_runtime::TransportView transport;
  std::optional<agx_robot_task_runtime::EffectRecord> matched_effect;
};
struct GraspEffectObservations {
  static constexpr std::size_t kCapacity = 64;
  std::array<GraspEffectObservation, kCapacity> records;
  std::size_t count = 0;  // Only records[0..count) are populated.
  bool gap = false;     // Sticky for this adapter, including after drain.
  std::uint64_t last_sequence = 0;
};
class GraspEffectAdapter : public std::enable_shared_from_this<GraspEffectAdapter> {
 public:
  static constexpr std::size_t kObservationCapacity = GraspEffectObservations::kCapacity;
  static constexpr std::size_t kObservationStringBytes = 512;
  GraspEffectAdapter(std::shared_ptr<agx_robot_task_runtime::RobotStateStore> state, std::string run_id,
      std::string task_id, std::string operation_id, std::string container_profile,
      agx_robot_task_runtime::StateRevision expected_revision);
  agxcore::TubeEffectSink Sink();
  bool Apply(const agxcore::TubeEffect& effect);
  agx_task_workflow::StepResult Finish(const agxcore::TubeMissionOutcome& outcome) const;
  agx_robot_task_runtime::ObjectRef Object() const { return object_; }
  // Optional and disabled by default. Enable once before any effect delivery.
  bool EnableObservationJournal();
  // Drains detached rows; never snapshots the store or resets sequence/gap.
  GraspEffectObservations TakeObservations();
  // Nullopt means acquisition failed (including spurious failure), not empty.
  // Unexpected lock errors propagate; no observation changes without the lock.
  std::optional<GraspEffectObservations> TryTakeObservations();
 private:
  // Caller holds mutex_; moves only existing rows and clears occupancy.
  GraspEffectObservations TakeObservationsLocked();
  void CaptureObservation(const agxcore::TubeEffect& event,
                          const agx_robot_task_runtime::StateUpdate* update, bool returned) noexcept;
  std::shared_ptr<agx_robot_task_runtime::RobotStateStore> state_;
  std::string run_id_, task_id_, operation_id_;
  agx_robot_task_runtime::ObjectRef object_;
  mutable std::mutex mutex_;
  agx_robot_task_runtime::StateRevision revision_;
  std::vector<std::string> applied_ids_;
  std::vector<agxcore::TubeEffect> received_;
  bool accepted_ = true;
  std::string detail_;
  bool delivery_started_ = false;
  bool observation_enabled_ = false;
  GraspEffectObservations observations_;
};
}  // namespace agx_tube_tasks
