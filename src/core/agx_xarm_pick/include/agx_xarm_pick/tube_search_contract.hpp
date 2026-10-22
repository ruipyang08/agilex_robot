#ifndef AGX_XARM_PICK_TUBE_SEARCH_CONTRACT_HPP
#define AGX_XARM_PICK_TUBE_SEARCH_CONTRACT_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include "agx_xarm_pick/tube_grasp_contract.hpp"

namespace agxcore {

inline bool finite_values(const std::vector<double>& values) {
  return std::all_of(
      values.begin(), values.end(),
      [](double value) { return std::isfinite(value); });
}

inline bool strictly_increasing(const std::vector<double>& values) {
  return values.size() >= 2 && finite_values(values) &&
      std::adjacent_find(
          values.begin(), values.end(),
          [](double left, double right) { return left >= right; }) ==
      values.end();
}

inline bool strictly_decreasing(const std::vector<double>& values) {
  return values.size() >= 2 && finite_values(values) &&
      std::adjacent_find(
          values.begin(), values.end(),
          [](double left, double right) { return left <= right; }) ==
      values.end();
}

inline bool search_status_advances(TubeMissionStatus status) {
  return status == TubeMissionStatus::kTargetAbsent ||
      status == TubeMissionStatus::kOutOfWorkspace ||
      status == TubeMissionStatus::kInconsistentTarget;
}

inline bool search_status_retries(TubeMissionStatus status) {
  return status == TubeMissionStatus::kStaleFrame ||
      status == TubeMissionStatus::kTfUnavailable;
}

inline bool search_height_envelope_in_workspace(
    double zero_lift_cap_top_m,
    double lift_target_m,
    double lift_tolerance_m,
    double lift_minimum_m,
    double lift_maximum_m,
    double clearance_minimum_m,
    double clearance_maximum_m,
    double workspace_minimum_z_m,
    double workspace_maximum_z_m) {
  if (!finite_values({
          zero_lift_cap_top_m,
          lift_target_m,
          lift_tolerance_m,
          lift_minimum_m,
          lift_maximum_m,
          clearance_minimum_m,
          clearance_maximum_m,
          workspace_minimum_z_m,
          workspace_maximum_z_m}) ||
      lift_tolerance_m < 0.0 ||
      lift_minimum_m < 0.0 ||
      lift_minimum_m > lift_target_m ||
      lift_target_m > lift_maximum_m ||
      clearance_minimum_m > clearance_maximum_m ||
      workspace_minimum_z_m >= workspace_maximum_z_m) {
    return false;
  }
  const double accepted_lift_minimum_m = std::max(
      lift_minimum_m, lift_target_m - lift_tolerance_m);
  const double accepted_lift_maximum_m = std::min(
      lift_maximum_m, lift_target_m + lift_tolerance_m);
  const double candidate_minimum_z_m =
      zero_lift_cap_top_m - accepted_lift_maximum_m +
      clearance_minimum_m;
  const double candidate_maximum_z_m =
      zero_lift_cap_top_m - accepted_lift_minimum_m +
      clearance_maximum_m;
  return candidate_minimum_z_m >= workspace_minimum_z_m &&
      candidate_maximum_z_m <= workspace_maximum_z_m;
}

}  // namespace agxcore

#endif  // AGX_XARM_PICK_TUBE_SEARCH_CONTRACT_HPP
