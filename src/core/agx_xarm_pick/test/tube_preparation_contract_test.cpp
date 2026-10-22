#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

#include "agx_xarm_pick/tube_preparation_contract.hpp"

namespace {

bool expect(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
  }
  return condition;
}

}  // namespace

int main() {
  using agx_xarm_pick::accepted_lift_speed_service_state;
  using agx_xarm_pick::phase_two_barrier_is_open;
  using agx_xarm_pick::preparation_phase_is_valid;
  using agx_xarm_pick::valid_phase_one_joint_positions;

  bool passed = true;
  const std::vector<double> commissioned_joint_positions = {
      -2.8479814646964625e-05,
      -0.8655031056295434,
      -1.4268700011286828,
      9.985777318568821e-06,
      2.292373139239332,
      -2.176791500270438e-05,
  };
  passed &= expect(
      valid_phase_one_joint_positions(commissioned_joint_positions),
      "commissioned phase-one joint positions were rejected");

  auto wrong_size = commissioned_joint_positions;
  wrong_size.pop_back();
  passed &= expect(
      !valid_phase_one_joint_positions(wrong_size),
      "five phase-one joint positions were accepted");

  auto too_many_positions = commissioned_joint_positions;
  too_many_positions.push_back(0.0);
  passed &= expect(
      !valid_phase_one_joint_positions(too_many_positions),
      "seven phase-one joint positions were accepted");

  auto non_finite = commissioned_joint_positions;
  non_finite[0] = std::numeric_limits<double>::quiet_NaN();
  passed &= expect(
      !valid_phase_one_joint_positions(non_finite),
      "NaN phase-one joint position was accepted");
  non_finite[0] = std::numeric_limits<double>::infinity();
  passed &= expect(
      !valid_phase_one_joint_positions(non_finite),
      "positive-infinity phase-one joint position was accepted");
  non_finite[0] = -std::numeric_limits<double>::infinity();
  passed &= expect(
      !valid_phase_one_joint_positions(non_finite),
      "negative-infinity phase-one joint position was accepted");

  passed &= expect(
      accepted_lift_speed_service_state(std::int64_t{1}),
      "accepted lift speed response state was rejected");
  passed &= expect(
      !accepted_lift_speed_service_state(std::int64_t{-1}),
      "negative lift speed response state was accepted");
  passed &= expect(
      !accepted_lift_speed_service_state(std::int64_t{0}),
      "zero lift speed response state was accepted");
  passed &= expect(
      !accepted_lift_speed_service_state(std::int64_t{2}),
      "unexpected positive lift speed response state was accepted");

  passed &= expect(
      preparation_phase_is_valid(1), "preparation phase one was rejected");
  passed &= expect(
      preparation_phase_is_valid(2), "preparation phase two was rejected");
  passed &= expect(
      !preparation_phase_is_valid(0), "preparation phase zero was accepted");
  passed &= expect(
      !preparation_phase_is_valid(3), "preparation phase three was accepted");

  for (const bool phase_one_succeeded : {false, true}) {
    for (const bool lift_succeeded : {false, true}) {
      for (const bool base_stopped : {false, true}) {
        passed &= expect(
            phase_two_barrier_is_open(
                phase_one_succeeded, lift_succeeded, base_stopped) ==
                (phase_one_succeeded && lift_succeeded && base_stopped),
            "phase-two barrier did not match its required conjunction");
      }
    }
  }

  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
