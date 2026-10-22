#pragma once
#include <optional>
#include <string>
namespace agx_robot_task_runtime {
inline constexpr const char* kRobotSequencePlan = "robot.sequence";
// Prospective state used by robot task factories; never measured custody.
struct RobotSequencePlan {
  std::optional<std::string> held_station;
  bool payload_may_be_retained = false;
};
}
