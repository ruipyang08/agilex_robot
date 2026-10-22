#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace agx_tube_tasks {
struct GraspEffectObservation;

namespace detail {
// Encodes only this detached row. The caller must not concurrently mutate it.
// Invalid identity/UTF-8/length throws invalid_argument; allocation errors propagate.
std::string EncodeGraspEffectObservation(const GraspEffectObservation& row);

// Optional transport metadata; never a control/admission state.
enum class GraspJournalStage { Progress, FinalDrain, Unavailable, NotStarted };
enum class GraspJournalUnavailable {
  PreparationFailed, IdentityInvalid, EnableFailed, BindingUnavailable
};
struct GraspJournalStatus {
  std::string run_id, task_id, operation_id;
  std::uint64_t recipe_index=0;
  GraspJournalStage stage=GraspJournalStage::Progress;
  bool entered=false, adapter_bound=false, monitor_returned=false;
  std::optional<bool> worker_joined, exit_required;
  bool finish_completed=false, runner_terminal_seen=false, final_acquired=false;
  std::optional<std::uint64_t> last_sequence;
  std::optional<bool> capture_gap;
  std::uint64_t rows_publish_returned=0, failed_rows=0, drain_error_count=0,
      prior_status_error_count=0;
  std::optional<std::uint64_t> first_failed_sequence, last_failed_sequence;
  bool transport_gap=false, counter_overflow=false, active=false;
  std::optional<GraspJournalUnavailable> unavailable_reason;
};
// Fixed projection with the same strict identity/string rules; no source access.
std::string EncodeGraspJournalStatus(const GraspJournalStatus& status);
}  // namespace detail
}  // namespace agx_tube_tasks
