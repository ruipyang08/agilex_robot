#include "agx_tube_tasks/grasp_observation_json.hpp"

#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "agx_tube_tasks/grasp_task.hpp"

namespace agx_tube_tasks {
namespace detail {
namespace {
static_assert(sizeof(double) == 8 && sizeof(std::uint64_t) == 8 &&
              std::numeric_limits<double>::is_iec559 &&
              std::numeric_limits<double>::digits == 53 &&
              std::numeric_limits<double>::max_exponent == 1024,
              "Grasp observation JSON requires IEEE-754 binary64");
constexpr char kHex[] = "0123456789abcdef";

void ValidateString(const std::string& text) {
  if (text.size() > GraspEffectAdapter::kObservationStringBytes)
    throw std::invalid_argument("grasp observation string exceeds 512 bytes");
  for (std::size_t i = 0; i < text.size();) {
    const auto lead = static_cast<unsigned char>(text[i]);
    if (lead <= 0x7f) { ++i; continue; }
    std::size_t count = 0;
    unsigned char second_min = 0x80, second_max = 0xbf;
    if (lead >= 0xc2 && lead <= 0xdf) {
      count = 2;
    } else if (lead >= 0xe0 && lead <= 0xef) {
      count = 3;
      if (lead == 0xe0) second_min = 0xa0;  // No overlong encoding.
      if (lead == 0xed) second_max = 0x9f;  // No surrogate scalar.
    } else if (lead >= 0xf0 && lead <= 0xf4) {
      count = 4;
      if (lead == 0xf0) second_min = 0x90;
      if (lead == 0xf4) second_max = 0x8f;  // At most U+10FFFF.
    } else {
      throw std::invalid_argument("grasp observation string is not valid UTF-8");
    }
    if (text.size() - i < count)
      throw std::invalid_argument("grasp observation string is not valid UTF-8");
    const auto second = static_cast<unsigned char>(text[i + 1]);
    if (second < second_min || second > second_max)
      throw std::invalid_argument("grasp observation string is not valid UTF-8");
    for (std::size_t j = 2; j < count; ++j) {
      const auto byte = static_cast<unsigned char>(text[i + j]);
      if (byte < 0x80 || byte > 0xbf)
        throw std::invalid_argument("grasp observation string is not valid UTF-8");
    }
    i += count;
  }
}

const char* Name(agxcore::TubeEffectKind value) {
  switch (value) {
    case agxcore::TubeEffectKind::TransferBegun: return "TransferBegun";
    case agxcore::TubeEffectKind::KnownEmpty: return "KnownEmpty";
    case agxcore::TubeEffectKind::Captured: return "Captured";
    case agxcore::TubeEffectKind::Retained: return "Retained";
    case agxcore::TubeEffectKind::RetentionUncertain: return "RetentionUncertain";
    case agxcore::TubeEffectKind::ExtractedSafeClear: return "ExtractedSafeClear";
    case agxcore::TubeEffectKind::MeasuredStop: return "MeasuredStop";
    case agxcore::TubeEffectKind::EmptySafeClear: return "EmptySafeClear";
  }
  return nullptr;
}
const char* Name(agx_robot_task_runtime::ObjectIdentity value) {
  switch (value) {
    case agx_robot_task_runtime::ObjectIdentity::Registered: return "Registered";
    case agx_robot_task_runtime::ObjectIdentity::RunLocal: return "RunLocal";
  }
  return nullptr;
}
const char* Name(agx_robot_task_runtime::HolderKind value) {
  switch (value) {
    case agx_robot_task_runtime::HolderKind::Gripper: return "Gripper";
    case agx_robot_task_runtime::HolderKind::FixtureSlot: return "FixtureSlot";
  }
  return nullptr;
}
const char* Name(agx_robot_task_runtime::LocationCertainty value) {
  switch (value) {
    case agx_robot_task_runtime::LocationCertainty::Unknown: return "Unknown";
    case agx_robot_task_runtime::LocationCertainty::Confirmed: return "Confirmed";
    case agx_robot_task_runtime::LocationCertainty::Uncertain: return "Uncertain";
  }
  return nullptr;
}
const char* Name(agx_robot_task_runtime::GripperOccupancy value) {
  switch (value) {
    case agx_robot_task_runtime::GripperOccupancy::Unknown: return "Unknown";
    case agx_robot_task_runtime::GripperOccupancy::Empty: return "Empty";
    case agx_robot_task_runtime::GripperOccupancy::Holding: return "Holding";
  }
  return nullptr;
}
const char* Name(agx_robot_task_runtime::TransportLoad value) {
  switch (value) {
    case agx_robot_task_runtime::TransportLoad::Unknown: return "Unknown";
    case agx_robot_task_runtime::TransportLoad::Empty: return "Empty";
    case agx_robot_task_runtime::TransportLoad::SingleHeld: return "SingleHeld";
    case agx_robot_task_runtime::TransportLoad::Unsupported: return "Unsupported";
  }
  return nullptr;
}
const char* Name(agx_robot_task_runtime::TravelEligibility value) {
  switch (value) {
    case agx_robot_task_runtime::TravelEligibility::Unknown: return "Unknown";
    case agx_robot_task_runtime::TravelEligibility::CapturedInFixture: return "CapturedInFixture";
    case agx_robot_task_runtime::TravelEligibility::SafeClear: return "SafeClear";
    case agx_robot_task_runtime::TravelEligibility::Unsupported: return "Unsupported";
  }
  return nullptr;
}
const char* Name(agx_robot_task_runtime::EffectKind value) {
  switch (value) {
    case agx_robot_task_runtime::EffectKind::GripperEmpty: return "GripperEmpty";
    case agx_robot_task_runtime::EffectKind::GripperHolding: return "GripperHolding";
    case agx_robot_task_runtime::EffectKind::Released: return "Released";
    case agx_robot_task_runtime::EffectKind::ObjectAtHolder: return "ObjectAtHolder";
    case agx_robot_task_runtime::EffectKind::TravelEligibility: return "TravelEligibility";
    case agx_robot_task_runtime::EffectKind::GripperUncertain: return "GripperUncertain";
  }
  return nullptr;
}
const char* Name(agx_robot_task_runtime::UpdateStatus value) {
  switch (value) {
    case agx_robot_task_runtime::UpdateStatus::Applied: return "Applied";
    case agx_robot_task_runtime::UpdateStatus::Duplicate: return "Duplicate";
    case agx_robot_task_runtime::UpdateStatus::Rejected: return "Rejected";
    case agx_robot_task_runtime::UpdateStatus::Conflict: return "Conflict";
  }
  return nullptr;
}

const char* StatusName(GraspJournalStage value) {
  switch(value) {
    case GraspJournalStage::Progress:return "progress";
    case GraspJournalStage::FinalDrain:return "final_drain";
    case GraspJournalStage::Unavailable:return "unavailable";
    case GraspJournalStage::NotStarted:return "not_started";
  }
  throw std::invalid_argument("unknown grasp journal stage");
}
const char* StatusName(GraspJournalUnavailable value) {
  switch(value) {
    case GraspJournalUnavailable::PreparationFailed:return "preparation_failed";
    case GraspJournalUnavailable::IdentityInvalid:return "identity_invalid";
    case GraspJournalUnavailable::EnableFailed:return "enable_failed";
    case GraspJournalUnavailable::BindingUnavailable:return "binding_unavailable";
  }
  throw std::invalid_argument("unknown grasp journal unavailable reason");
}

// Local writer for two fixed diagnostic projections; no stream or state access.
class RowJson {
 public:
  std::string Encode(const GraspEffectObservation& row, const std::string& invocation) {
    text_ = "{\"schema_version\":1,\"kind\":\"grasp_effect_observation\"";
    Next("run_id", row.run_id);
    Next("task_id", row.task_id);
    Key("invocation_id");
    Quoted(invocation);  // Derived from validated strings, not another captured field.
    Next("operation_id", row.operation_id);
    Next("observation_sequence", row.observation_sequence);
    Next("object", row.object);
    Next("source_event", row.source_event);
    Next("application", row.application);
    Next("adapter_accepted", row.adapter_accepted);
    Next("snapshot_revision", row.snapshot_revision);
    Next("snapshot_halted", row.snapshot_halted);
    Next("gripper", row.gripper);
    Next("object_location", row.object_location);
    Next("transport", row.transport);
    Next("matched_effect", row.matched_effect);
    text_ += '}';
    return std::move(text_);
  }

  std::string Encode(const GraspJournalStatus& value,const std::string& invocation) {
    text_="{\"schema_version\":1,\"kind\":\"grasp_effect_journal_status\"";
    Next("run_id",value.run_id);Next("task_id",value.task_id);
    Key("invocation_id");Quoted(invocation);Next("operation_id",value.operation_id);
    Next("recipe_index",value.recipe_index);Next("stage",std::string(StatusName(value.stage)));
    Key("producer");First("entered",value.entered);Next("adapter_bound",value.adapter_bound);
    Next("monitor_returned",value.monitor_returned);Next("worker_joined",value.worker_joined);
    Next("exit_required",value.exit_required);Next("finish_completed",value.finish_completed);
    Next("runner_terminal_seen",value.runner_terminal_seen);text_+='}';
    Key("drain");First("final_acquired",value.final_acquired);Next("last_sequence",value.last_sequence);
    Next("capture_gap",value.capture_gap);text_+='}';
    Key("transport");First("rows_publish_returned",value.rows_publish_returned);
    Next("failed_rows",value.failed_rows);Next("first_failed_sequence",value.first_failed_sequence);
    Next("last_failed_sequence",value.last_failed_sequence);Next("transport_gap",value.transport_gap);
    Next("drain_error_count",value.drain_error_count);Next("prior_status_error_count",value.prior_status_error_count);
    Next("counter_overflow",value.counter_overflow);text_+='}';
    Key("availability");First("active",value.active);
    if(value.unavailable_reason)Next("unavailable_reason",std::string(StatusName(*value.unavailable_reason)));
    else Next("unavailable_reason",std::optional<std::string>{});
    text_+="}}";
    return std::move(text_);
  }

 private:
  void Key(const char* name, char prefix = ',') {
    text_ += prefix;
    text_ += '"';
    text_ += name;  // Only the fixed ASCII keys below reach this method.
    text_ += "\":";
  }
  template <typename T> void First(const char* key, const T& value) {
    Key(key, '{');
    Value(value);
  }
  template <typename T> void Next(const char* key, const T& value) {
    Key(key);
    Value(value);
  }
  void Quoted(const std::string& text) {
    text_ += '"';
    for (const unsigned char byte : text) {
      if (byte == '"' || byte == '\\') {
        text_ += '\\';
        text_ += static_cast<char>(byte);
      } else if (byte < 0x20) {
        text_ += "\\u00";
        text_ += kHex[byte >> 4];
        text_ += kHex[byte & 0xf];
      } else {
        text_ += static_cast<char>(byte);
      }
    }
    text_ += '"';
  }
  void Value(const std::string& value) { ValidateString(value); Quoted(value); }
  void Value(bool value) { text_ += value ? "true" : "false"; }
  void Value(std::uint64_t value) { text_ += std::to_string(value); }
  void Value(const double& value) {
    std::uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    // Classify bits before any floating operation, including signaling NaNs.
    if ((bits & UINT64_C(0x7ff0000000000000)) == UINT64_C(0x7ff0000000000000)) {
      const char* classification = (bits & UINT64_C(0x000fffffffffffff)) ? "nan" :
          (bits >> 63) ? "negative_infinity" : "positive_infinity";
      text_ += "{\"classification\":\"";
      text_ += classification;
      text_ += "\",\"ieee754_bits\":\"";
      for (int shift = 60; shift >= 0; shift -= 4) text_ += kHex[(bits >> shift) & 0xf];
      text_ += "\"}";
    } else if ((bits & UINT64_C(0x7fffffffffffffff)) == 0) {
      text_ += (bits >> 63) ? "-0.0" : "0.0";
    } else {
      std::ostringstream scalar;
      scalar.exceptions(std::ios::badbit | std::ios::failbit);
      scalar.imbue(std::locale::classic());
      scalar << std::defaultfloat << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
      const auto token = scalar.str();
      text_ += token;
      if (token.find_first_of(".eE") == std::string::npos) text_ += ".0";
    }
  }
  template <typename E, std::enable_if_t<std::is_enum<E>::value, int> = 0>
  void Value(E value) {
    text_ += "{\"name\":";
    if (const char* name = Name(value)) {
      text_ += '"'; text_ += name; text_ += '"';
    } else {
      text_ += "null";
    }
    text_ += ",\"value\":";
    text_ += std::to_string(static_cast<std::underlying_type_t<E>>(value));
    text_ += '}';
  }
  template <typename T> void Value(const std::optional<T>& value) {
    if (value) Value(*value);
    else text_ += "null";
  }
  void Value(const agx_robot_task_runtime::ObjectRef& value) {
    First("id", value.id);
    Next("identity", value.identity);
    Next("container_profile", value.container_profile);
    text_ += '}';
  }
  void Value(const agx_robot_task_runtime::HolderRef& value) {
    First("kind", value.kind);
    Next("fixture_instance", value.fixture_instance);
    Next("slot", value.slot);
    text_ += '}';
  }
  void Value(const agxcore::TubeEvidence& value) {
    First("source", value.source);
    Next("source_stamp_s", value.source_stamp_s);
    Next("receipt_steady_s", value.receipt_steady_s);
    text_ += '}';
  }
  void Value(const agx_robot_task_runtime::Evidence& value) {
    First("source", value.source);
    Next("source_stamp_s", value.source_stamp_s);
    Next("receipt_steady_s", value.receipt_steady_s);
    Next("run_id", value.run_id);
    Next("task_id", value.task_id);
    Next("operation_id", value.operation_id);
    text_ += '}';
  }
  void Value(const agxcore::TubeEffect& value) {
    First("sequence", value.sequence);
    Next("kind", value.kind);
    Next("evidence", value.evidence);
    text_ += '}';
  }
  void Value(const agx_robot_task_runtime::StateUpdate& value) {
    First("status", value.status);
    Next("revision", value.revision);
    Next("halt", value.halt);
    Next("detail", value.detail);
    text_ += '}';
  }
  void Value(const agx_robot_task_runtime::PendingTransfer& value) {
    First("operation_id", value.operation_id);
    Next("object", value.object);
    Next("source", value.source);
    Next("destination", value.destination);
    text_ += '}';
  }
  void Value(const agx_robot_task_runtime::ObjectLocation& value) {
    First("object", value.object);
    Next("last_confirmed_holder", value.last_confirmed_holder);
    Next("last_confirmed_evidence", value.last_confirmed_evidence);
    Next("certainty", value.certainty);
    Next("current_evidence", value.current_evidence);
    Next("pending", value.pending);
    Next("revision", value.revision);
    Next("fresh", value.fresh);
    text_ += '}';
  }
  void Value(const agx_robot_task_runtime::GripperView& value) {
    First("occupancy", value.occupancy);
    Next("object", value.object);
    Next("evidence", value.evidence);
    Next("revision", value.revision);
    Next("fresh", value.fresh);
    text_ += '}';
  }
  void Value(const agx_robot_task_runtime::TransportView& value) {
    First("load", value.load);
    Next("eligibility", value.eligibility);
    Next("object", value.object);
    Next("custody_evidence", value.custody_evidence);
    Next("clearance_evidence", value.clearance_evidence);
    Next("pending", value.pending);
    Next("uncertain", value.uncertain);
    Next("ready", value.ready);
    Next("revision", value.revision);
    text_ += '}';
  }
  void Value(const agx_robot_task_runtime::PhysicalEffect& value) {
    First("id", value.id);
    Next("kind", value.kind);
    Next("object", value.object);
    Next("holder", value.holder);
    Next("eligibility", value.eligibility);
    text_ += '}';
  }
  void Value(const agx_robot_task_runtime::EffectRecord& value) {
    First("effect", value.effect);
    Next("evidence", value.evidence);
    Next("operation_id", value.operation_id);
    Next("expected_revision", value.expected_revision);
    Next("resulting_revision", value.resulting_revision);
    Next("status", value.status);
    text_ += '}';
  }
  std::string text_;
};
}  // namespace

std::string EncodeGraspEffectObservation(const GraspEffectObservation& row) {
  ValidateString(row.run_id);
  ValidateString(row.task_id);
  ValidateString(row.operation_id);
  if (row.run_id.empty() || row.task_id.empty())
    throw std::invalid_argument("grasp observation requires run and task identity");
  const std::string invocation = row.run_id + "/" + row.task_id;
  if (row.operation_id != invocation + "/grasp")
    throw std::invalid_argument("grasp observation operation does not match run/task identity");
  return RowJson().Encode(row, invocation);
}
std::string EncodeGraspJournalStatus(const GraspJournalStatus& status) {
  ValidateString(status.run_id);ValidateString(status.task_id);ValidateString(status.operation_id);
  if(status.run_id.empty() || status.task_id.empty())
    throw std::invalid_argument("grasp journal requires run and task identity");
  const std::string invocation=status.run_id+"/"+status.task_id;
  if(status.operation_id!=invocation+"/grasp")
    throw std::invalid_argument("grasp journal operation does not match run/task identity");
  return RowJson().Encode(status,invocation);
}
}  // namespace detail
}  // namespace agx_tube_tasks
