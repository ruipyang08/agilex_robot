#ifndef AGX_XARM_PICK_MISSION_CONTRACT_HPP
#define AGX_XARM_PICK_MISSION_CONTRACT_HPP

#include <cstdlib>

namespace agxcore {

enum class MissionStatus {
  kSuccess = 0,
  kSetupUnavailable,
  kMoveItUnavailable,
  kMarkerUnavailable,
  kSearchExhausted,
  kPlanFailed,
  kExecutionFailed,
  kGripperTimeout,
  kCaughtStateRequired,
  kOpenStateRequired,
  kInterrupted,
};

inline bool mission_succeeded(MissionStatus status) {
  return status == MissionStatus::kSuccess;
}

inline int mission_exit_code(MissionStatus status) {
  return mission_succeeded(status) ? EXIT_SUCCESS : EXIT_FAILURE;
}

inline const char* mission_status_name(MissionStatus status) {
  switch (status) {
    case MissionStatus::kSuccess:
      return "success";
    case MissionStatus::kSetupUnavailable:
      return "setup unavailable";
    case MissionStatus::kMoveItUnavailable:
      return "MoveIt unavailable";
    case MissionStatus::kMarkerUnavailable:
      return "marker unavailable";
    case MissionStatus::kSearchExhausted:
      return "marker search exhausted";
    case MissionStatus::kPlanFailed:
      return "planning failed";
    case MissionStatus::kExecutionFailed:
      return "execution failed";
    case MissionStatus::kGripperTimeout:
      return "gripper state timeout";
    case MissionStatus::kCaughtStateRequired:
      return "caught gripper state required";
    case MissionStatus::kOpenStateRequired:
      return "open gripper state required";
    case MissionStatus::kInterrupted:
      return "interrupted";
  }
  return "unknown";
}

}  // namespace agxcore

#endif  // AGX_XARM_PICK_MISSION_CONTRACT_HPP
