#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "agx_xarm_pick/tube_lift_contract.hpp"
#include "agx_xarm_pick/tube_preparation_contract.hpp"

int main() {
  using agx_xarm_pick::accepted_lift_service_state;
  using agx_xarm_pick::accepted_lift_speed_service_state;
  using agx_xarm_pick::compute_lift_target;
  using agx_xarm_pick::current_lift_feedback_is_usable;
  using agx_xarm_pick::lift_fault_evidence_belongs_to_operation;
  using agx_xarm_pick::lift_fault_belongs_to_operation;
  using agx_xarm_pick::lift_feedback_is_fault_free;
  using agx_xarm_pick::lift_feedback_reached;
  using agx_xarm_pick::resolve_zero_lift_z;

  const double infinity = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const auto target = compute_lift_target(1.185, 0.885, 0.0, 0.600);
  assert(target.has_value());
  assert(target->millimetres == 300);
  assert(std::abs(target->metres - 0.300) < 1e-12);
  assert(!compute_lift_target(0.8, 0.9, 0.0, 0.6).has_value());
  assert(!compute_lift_target(1.6, 0.8, 0.0, 0.6).has_value());
  assert(accepted_lift_service_state(1));
  assert(accepted_lift_service_state(-1));
  assert(!accepted_lift_service_state(-9));
  assert(accepted_lift_speed_service_state(1));
  assert(!accepted_lift_speed_service_state(-1));
  assert(!accepted_lift_speed_service_state(0));
  assert(!lift_fault_belongs_to_operation(10, 10));
  assert(lift_fault_belongs_to_operation(11, 10));
  assert(lift_fault_belongs_to_operation(20, 19));
  assert(!lift_fault_belongs_to_operation(19, 20));
  assert(lift_fault_evidence_belongs_to_operation(
      11, 10, 101.0, 100.0));
  assert(!lift_fault_evidence_belongs_to_operation(
      11, 10, 99.0, 100.0));
  assert(!lift_fault_evidence_belongs_to_operation(
      11, 10, 100.0, 100.0));
  assert(!lift_fault_evidence_belongs_to_operation(
      10, 10, 101.0, 100.0));
  assert(!lift_fault_evidence_belongs_to_operation(
      9, 10, 101.0, 100.0));
  assert(!lift_fault_evidence_belongs_to_operation(
      11, 10, infinity, 100.0));
  assert(!lift_fault_evidence_belongs_to_operation(
      11, 10, 101.0, nan));
  assert(lift_feedback_reached(
      true, true, 300, 300, 0.0, 0.1, 0.25, 2));
  assert(!lift_feedback_reached(
      true, true, 296, 300, 0.0, 0.1, 0.25, 2));
  assert(std::abs(resolve_zero_lift_z(0.600, 0.300) - 0.300) < 1e-12);

  assert(current_lift_feedback_is_usable(
      true, true, true, 98, 100, 0.10, 0.10, 0.25, 2, 0, 600));
  assert(!current_lift_feedback_is_usable(
      false, true, true, 98, 100, 0.10, 0.10, 0.25, 2, 0, 600));
  assert(!current_lift_feedback_is_usable(
      true, false, true, 98, 100, 0.10, 0.10, 0.25, 2, 0, 600));
  assert(!current_lift_feedback_is_usable(
      true, true, false, 98, 100, 0.10, 0.10, 0.25, 2, 0, 600));
  assert(!current_lift_feedback_is_usable(
      true, true, true, 97, 100, 0.10, 0.10, 0.25, 2, 0, 600));
  assert(!current_lift_feedback_is_usable(
      true, true, true, 98, 100, 0.30, 0.10, 0.25, 2, 0, 600));
  assert(!current_lift_feedback_is_usable(
      true, true, true, 98, 100, 0.10, 0.30, 0.25, 2, 0, 600));
  assert(!current_lift_feedback_is_usable(
      true, true, true, -1, 0, 0.10, 0.10, 0.25, 2, 0, 600));
  assert(!current_lift_feedback_is_usable(
      true, true, true, 0, -1, 0.10, 0.10, 0.25, 2, 0, 600));
  assert(!current_lift_feedback_is_usable(
      true, true, true, 601, 601, 0.10, 0.10, 0.25, 2, 0, 600));
  assert(!current_lift_feedback_is_usable(
      true, true, true, 600, 601, 0.10, 0.10, 0.25, 2, 0, 600));
  assert(!current_lift_feedback_is_usable(
      true, true, true, 98, 100, 0.10, 0.10, 0.25, 2, 600, 0));

  assert(lift_feedback_is_fault_free(
      false, false, false, false, false, false, false, false, 0));
  assert(!lift_feedback_is_fault_free(
      true, false, false, false, false, false, false, false, 0));
  assert(!lift_feedback_is_fault_free(
      false, true, false, false, false, false, false, false, 0));
  assert(!lift_feedback_is_fault_free(
      false, false, true, false, false, false, false, false, 0));
  assert(!lift_feedback_is_fault_free(
      false, false, false, true, false, false, false, false, 0));
  assert(!lift_feedback_is_fault_free(
      false, false, false, false, true, false, false, false, 0));
  assert(!lift_feedback_is_fault_free(
      false, false, false, false, false, true, false, false, 0));
  assert(!lift_feedback_is_fault_free(
      false, false, false, false, false, false, true, false, 0));
  assert(!lift_feedback_is_fault_free(
      false, false, false, false, false, false, false, true, 0));
  assert(!lift_feedback_is_fault_free(
      false, false, false, false, false, false, false, false, 1));
  assert(!lift_feedback_is_fault_free(
      false, false, false, false, false, false, false, false, -1));

  assert(!compute_lift_target(nan, 0.8, 0.0, 0.6).has_value());
  assert(!compute_lift_target(1.1, infinity, 0.0, 0.6).has_value());
  assert(!compute_lift_target(1.1, 0.8, nan, 0.6).has_value());
  assert(!compute_lift_target(1.1, 0.8, 0.0, infinity).has_value());
  assert(!compute_lift_target(1.1, 0.8, 0.6, 0.0).has_value());

  const auto rounded_target =
      compute_lift_target(1.1854, 0.885, 0.0, 0.600);
  assert(rounded_target.has_value());
  assert(rounded_target->millimetres == 300);
  assert(std::abs(rounded_target->metres - 0.300) < 1e-12);

  assert(!lift_feedback_reached(
      false, true, 300, 300, 0.0, 0.1, 0.25, 2));
  assert(!lift_feedback_reached(
      true, false, 300, 300, 0.0, 0.1, 0.25, 2));
  assert(!lift_feedback_reached(
      true, true, 300, 300, -0.1, 0.1, 0.25, 2));
  assert(!lift_feedback_reached(
      true, true, 300, 300, 0.1, nan, 0.25, 2));
  assert(!lift_feedback_reached(
      true, true, 300, 300, 0.1, 0.1, infinity, 2));
  assert(!lift_feedback_reached(
      true, true, 300, 300, 0.3, 0.1, 0.25, 2));
  assert(!lift_feedback_reached(
      true, true, 300, 300, 0.1, 0.3, 0.25, 2));
  assert(!lift_feedback_reached(
      true, true, 300, 300, 0.1, 0.1, 0.25, -1));
  assert(lift_feedback_reached(
      true, true, 298, 300, 0.1, 0.1, 0.25, 2));

  assert(std::isnan(resolve_zero_lift_z(nan, 0.300)));
  assert(std::isnan(resolve_zero_lift_z(0.600, infinity)));
  return 0;
}
