#pragma once

#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>
#include "agx_xarm_pick/tube_grasp_contract.hpp"

namespace agxcore {
enum class TubeEffectKind {
  TransferBegun, KnownEmpty, Captured, Retained, RetentionUncertain,
  ExtractedSafeClear, MeasuredStop, EmptySafeClear
};
enum class TubeClearance { Unknown, CapturedInFixture, SafeClear };
struct TubeEvidence {
  std::string source;
  double source_stamp_s = 0.0;
  double receipt_steady_s = 0.0;
};
struct TubeEffect {
  std::uint64_t sequence = 0;
  TubeEffectKind kind = TubeEffectKind::RetentionUncertain;
  TubeEvidence evidence;
};
using TubeEffectSink = std::function<bool(const TubeEffect&)>;
struct TubeMissionOutcome {
  TubeMissionStatus status = TubeMissionStatus::kInterrupted;
  PayloadKnowledge payload = PayloadKnowledge::kUnknown;
  TubeClearance clearance = TubeClearance::Unknown;
  std::vector<TubeEffect> effects;
  bool effects_accepted = true;
  bool worker_joined = false;
  bool measured_stop_confirmed = false;
};

// Serializes observation delivery. A sink rejection is permanent: callers must
// stop before issuing another command. Outcome reporting never replays effects.
class TubeEffectJournal {
 public:
  explicit TubeEffectJournal(TubeEffectSink sink = {}) : sink_(std::move(sink)) {}
  bool Record(TubeEffectKind kind, const TubeEvidence& evidence);
  TubeMissionOutcome Outcome(TubeMissionStatus status) const;
 private:
  mutable std::mutex mutex_;
  TubeEffectSink sink_;
  TubeMissionOutcome outcome_;
};
}  // namespace agxcore
