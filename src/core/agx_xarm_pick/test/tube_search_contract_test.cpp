#include "agx_xarm_pick/tube_search_contract.hpp"

#include <cstdlib>
#include <iostream>

int main() {
  using agxcore::TubeMissionStatus;
  using agxcore::search_height_envelope_in_workspace;
  using agxcore::search_status_advances;
  using agxcore::search_status_retries;
  using agxcore::strictly_decreasing;
  using agxcore::strictly_increasing;

  if (!strictly_increasing({0.30, 0.45, 0.60}) ||
      strictly_increasing({0.30, 0.30, 0.60}) ||
      !strictly_decreasing({0.15, 0.05, -0.05}) ||
      strictly_decreasing({0.15, 0.15, -0.05})) {
    std::cerr << "search waypoint ordering is unsafe\n";
    return EXIT_FAILURE;
  }
  for (const auto status : {
           TubeMissionStatus::kTargetAbsent,
           TubeMissionStatus::kOutOfWorkspace,
           TubeMissionStatus::kInconsistentTarget}) {
    if (!search_status_advances(status)) {
      std::cerr << "ordinary rejection did not advance search\n";
      return EXIT_FAILURE;
    }
  }
  for (const auto status : {
           TubeMissionStatus::kStaleFrame,
           TubeMissionStatus::kTfUnavailable}) {
    if (!search_status_retries(status)) {
      std::cerr << "transient perception status did not retry\n";
      return EXIT_FAILURE;
    }
  }
  if (search_status_advances(TubeMissionStatus::kAmbiguousTarget) ||
      search_status_retries(TubeMissionStatus::kInvalidRequest)) {
    std::cerr << "unsafe search policy classification\n";
    return EXIT_FAILURE;
  }
  if (!search_height_envelope_in_workspace(
          0.402564,
          0.100,
          0.002,
          0.0,
          0.600,
          0.030,
          0.250,
          0.050,
          0.650) ||
      search_height_envelope_in_workspace(
          0.402564,
          0.0,
          0.002,
          0.0,
          0.600,
          0.030,
          0.250,
          0.050,
          0.650)) {
    std::cerr << "search height envelope ignored the commissioned lift\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
