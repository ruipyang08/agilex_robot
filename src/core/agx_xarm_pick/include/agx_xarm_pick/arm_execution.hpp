#ifndef AGX_XARM_PICK_ARM_EXECUTION_HPP
#define AGX_XARM_PICK_ARM_EXECUTION_HPP

#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace agx_xarm_pick {

enum class ArmMotion {
  Phase1Preparation, Phase2Preparation, Search, VisualAlignment,
  PregraspAdjustment, Insertion, Extraction, ReturnWithTube, Recovery,
};

inline constexpr std::array<const char*, 9> kArmMotionNames{{
    "phase1_preparation", "phase2_preparation", "search", "visual_alignment",
    "pregrasp_adjustment", "insertion", "extraction", "return_with_tube", "recovery"}};

struct ArmExecutionScales { double velocity; double acceleration; };
using ArmExecutionProfile = std::array<ArmExecutionScales, kArmMotionNames.size()>;

inline void require_unit_scale(double value) {
  if (!std::isfinite(value) || value <= 0.0 || value > 1.0)
    throw std::invalid_argument("arm scale must be finite and in (0, 1]");
}

inline ArmExecutionScales resolve_arm_execution(
    ArmExecutionScales global, double task_default,
    std::optional<double> motion_override) {
  require_unit_scale(global.velocity);
  require_unit_scale(global.acceleration);
  require_unit_scale(task_default);
  const double coefficient = motion_override.value_or(task_default);
  require_unit_scale(coefficient);
  ArmExecutionScales result{
      global.velocity * coefficient, global.acceleration * coefficient};
  require_unit_scale(result.velocity);
  require_unit_scale(result.acceleration);
  return result;
}

}  // namespace agx_xarm_pick
#endif
