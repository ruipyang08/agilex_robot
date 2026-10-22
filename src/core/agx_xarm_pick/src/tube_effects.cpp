#include "agx_xarm_pick/tube_effects.hpp"
namespace agxcore {
bool TubeEffectJournal::Record(TubeEffectKind kind, const TubeEvidence& evidence) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!outcome_.effects_accepted) return false;
  TubeEffect effect{outcome_.effects.size() + 1, kind, evidence};
  switch (kind) {
    case TubeEffectKind::KnownEmpty:
      outcome_.payload = PayloadKnowledge::kKnownEmpty;
      outcome_.clearance = TubeClearance::Unknown;
      break;
    case TubeEffectKind::TransferBegun:
      outcome_.payload = PayloadKnowledge::kCloseIssuedUnknown;
      outcome_.clearance = TubeClearance::Unknown;
      break;
    case TubeEffectKind::Captured:
      outcome_.payload = PayloadKnowledge::kHolding;
      outcome_.clearance = TubeClearance::CapturedInFixture;
      break;
    case TubeEffectKind::Retained:
      outcome_.payload = PayloadKnowledge::kHolding;
      break;
    case TubeEffectKind::RetentionUncertain:
      outcome_.payload = PayloadKnowledge::kUnknown;
      outcome_.clearance = TubeClearance::Unknown;
      break;
    case TubeEffectKind::ExtractedSafeClear:
      outcome_.payload = PayloadKnowledge::kHolding;
      outcome_.clearance = TubeClearance::SafeClear;
      break;
    case TubeEffectKind::MeasuredStop:
      outcome_.measured_stop_confirmed = true;
      break;
    case TubeEffectKind::EmptySafeClear:
      outcome_.payload = PayloadKnowledge::kKnownEmpty;
      outcome_.clearance = TubeClearance::SafeClear;
      break;
  }
  outcome_.effects.push_back(effect);
  try { outcome_.effects_accepted = !sink_ || sink_(effect); }
  catch (...) { outcome_.effects_accepted = false; }
  return outcome_.effects_accepted;
}
TubeMissionOutcome TubeEffectJournal::Outcome(TubeMissionStatus status) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = outcome_; result.status = status; return result;
}
}
