#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace agx_xarm_pick {

inline bool valid_phase_one_joint_positions(
    const std::vector<double>& joint_positions) {
  return joint_positions.size() == 6 &&
         std::all_of(
             joint_positions.begin(),
             joint_positions.end(),
             [](double position) { return std::isfinite(position); });
}

inline bool accepted_lift_speed_service_state(std::int64_t state) {
  return state == 1;
}

inline bool preparation_phase_is_valid(int phase) {
  return phase == 1 || phase == 2;
}

inline bool phase_two_barrier_is_open(
    bool phase_one_succeeded,
    bool lift_succeeded,
    bool base_stopped) {
  return phase_one_succeeded && lift_succeeded && base_stopped;
}

}  // namespace agx_xarm_pick
