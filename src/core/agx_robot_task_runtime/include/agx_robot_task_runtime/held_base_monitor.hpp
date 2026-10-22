#pragma once
#include <optional>

namespace agx_robot_task_runtime {
// Stopped handoff still requires a full quiet dwell. After hold is established,
// distinguish an isolated measured velocity spike from sustained movement.
// Freshness, authority, zero output and pose displacement are checked separately
// and fail immediately; this dwell does not filter or replace measured state.
class MotionDwell {
 public:
  bool Update(double now, bool below_stopped_thresholds, double dwell) {
    if (below_stopped_thresholds) { moving_since_.reset(); return true; }
    if (!moving_since_) moving_since_=now;
    return now-*moving_since_<dwell;
  }
 private:
  std::optional<double> moving_since_;
};
}
