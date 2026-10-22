#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

namespace agx_xarm_pick {

struct LiftTarget {
  std::int64_t millimetres;
  double metres;
};

inline std::optional<LiftTarget> compute_lift_target(
    const double table_surface_m,
    const double zero_lift_arm_base_m,
    const double minimum_m,
    const double maximum_m) {
  if (!std::isfinite(table_surface_m) ||
      !std::isfinite(zero_lift_arm_base_m) ||
      !std::isfinite(minimum_m) || !std::isfinite(maximum_m) ||
      minimum_m > maximum_m) {
    return std::nullopt;
  }

  const double target_m = table_surface_m - zero_lift_arm_base_m;
  if (!std::isfinite(target_m) || target_m < minimum_m ||
      target_m > maximum_m) {
    return std::nullopt;
  }

  const double target_mm = target_m * 1000.0;
  if (!std::isfinite(target_mm) ||
      static_cast<long double>(target_mm) <
          static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
      static_cast<long double>(target_mm) >
          static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    return std::nullopt;
  }

  const std::int64_t rounded_mm = std::llround(target_mm);
  return LiftTarget{rounded_mm, static_cast<double>(rounded_mm) / 1000.0};
}

inline bool accepted_lift_service_state(const std::int64_t state) {
  return state == 1 || state == -1;
}

inline bool lift_fault_belongs_to_operation(
    const std::uint64_t fault_feedback_epoch,
    const std::uint64_t operation_start_feedback_epoch) {
  return fault_feedback_epoch > operation_start_feedback_epoch;
}

inline bool lift_fault_evidence_belongs_to_operation(
    const std::uint64_t fault_feedback_epoch,
    const std::uint64_t operation_start_feedback_epoch,
    const double fault_receipt_time_s,
    const double command_time_s) {
  return lift_fault_belongs_to_operation(
             fault_feedback_epoch,
             operation_start_feedback_epoch) &&
      std::isfinite(fault_receipt_time_s) &&
      std::isfinite(command_time_s) &&
      fault_receipt_time_s > command_time_s;
}

inline bool lift_feedback_is_fault_free(
    const bool overflow_i,
    const bool overflow_v,
    const bool encoder_error,
    const bool position_bias_over,
    const bool under_voltage,
    const bool overloaded,
    const bool external_control_mode,
    const bool stop_flag,
    const std::int64_t fps_error) {
  return !overflow_i &&
      !overflow_v &&
      !encoder_error &&
      !position_bias_over &&
      !under_voltage &&
      !overloaded &&
      !external_control_mode &&
      !stop_flag &&
      fps_error == 0;
}

inline bool lift_feedback_reached(
    const bool has_feedback,
    const bool initialized,
    const std::int64_t back_height_mm,
    const std::int64_t target_height_mm,
    const double wall_receipt_age_sec,
    const double stamp_age_sec,
    const double maximum_age_sec,
    const std::int64_t tolerance_mm) {
  if (!has_feedback || !initialized ||
      !std::isfinite(wall_receipt_age_sec) ||
      !std::isfinite(stamp_age_sec) || !std::isfinite(maximum_age_sec) ||
      wall_receipt_age_sec < 0.0 || stamp_age_sec < 0.0 ||
      maximum_age_sec < 0.0 || wall_receipt_age_sec > maximum_age_sec ||
      stamp_age_sec > maximum_age_sec || tolerance_mm < 0) {
    return false;
  }

  const std::uint64_t absolute_error_mm =
      back_height_mm >= target_height_mm
          ? static_cast<std::uint64_t>(back_height_mm) -
                static_cast<std::uint64_t>(target_height_mm)
          : static_cast<std::uint64_t>(target_height_mm) -
                static_cast<std::uint64_t>(back_height_mm);
  return absolute_error_mm <= static_cast<std::uint64_t>(tolerance_mm);
}

inline bool current_lift_feedback_is_usable(
    const bool has_feedback,
    const bool initialized,
    const bool fault_free,
    const std::int64_t back_height_mm,
    const std::int64_t target_height_mm,
    const double wall_receipt_age_sec,
    const double stamp_age_sec,
    const double maximum_age_sec,
    const std::int64_t tolerance_mm,
    const std::int64_t minimum_height_mm,
    const std::int64_t maximum_height_mm) {
  if (!fault_free || minimum_height_mm > maximum_height_mm ||
      back_height_mm < minimum_height_mm ||
      back_height_mm > maximum_height_mm ||
      target_height_mm < minimum_height_mm ||
      target_height_mm > maximum_height_mm) {
    return false;
  }
  return lift_feedback_reached(
      has_feedback,
      initialized,
      back_height_mm,
      target_height_mm,
      wall_receipt_age_sec,
      stamp_age_sec,
      maximum_age_sec,
      tolerance_mm);
}

inline double resolve_zero_lift_z(
    const double zero_lift_z_m, const double measured_lift_m) {
  if (!std::isfinite(zero_lift_z_m) || !std::isfinite(measured_lift_m)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return zero_lift_z_m - measured_lift_m;
}

}  // namespace agx_xarm_pick
