#include "agx_xarm_pick/mission_contract.hpp"

#include <array>
#include <cstdlib>
#include <iostream>

int main() {
  using agxcore::MissionStatus;
  using agxcore::mission_exit_code;

  if (mission_exit_code(MissionStatus::kSuccess) != EXIT_SUCCESS) {
    std::cerr << "success did not map to EXIT_SUCCESS\n";
    return EXIT_FAILURE;
  }

  constexpr std::array<MissionStatus, 10> failures = {
      MissionStatus::kSetupUnavailable,
      MissionStatus::kMoveItUnavailable,
      MissionStatus::kMarkerUnavailable,
      MissionStatus::kSearchExhausted,
      MissionStatus::kPlanFailed,
      MissionStatus::kExecutionFailed,
      MissionStatus::kGripperTimeout,
      MissionStatus::kCaughtStateRequired,
      MissionStatus::kOpenStateRequired,
      MissionStatus::kInterrupted,
  };
  for (const auto status : failures) {
    if (mission_exit_code(status) != EXIT_FAILURE) {
      std::cerr << "failure status mapped to success\n";
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}
