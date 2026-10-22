#ifndef AGX_WORKBENCH_NAVIGATION_RANGER_CONTROL_HPP_
#define AGX_WORKBENCH_NAVIGATION_RANGER_CONTROL_HPP_

#include <agx_workbench_navigation/canonical_motion_feedback.hpp>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace agx_workbench_navigation {

enum class PlannerState : uint8_t {
  FOLLOW_ARC = 0,
  SPIN_ALIGN = 1,
  GOAL_ALIGN = 2,
  BLOCKED = 3,
  GOAL_REACHED = 4,
};

struct RangerPlannerConfig {
  std::string state_topic;
  std::string local_plan_topic;
  double min_turn_radius_m;
  double max_linear_speed_mps;
  double max_linear_accel_mps2;
  double max_linear_decel_mps2;
  double max_lateral_accel_mps2;
  double rate_limit_max_dt_s;
  double max_spin_speed_radps;
  double xy_goal_tolerance_m;
  double yaw_goal_tolerance_rad;
  double max_curvature_rate_per_m_s = 0.5;
  double curvature_tracking_slowdown_per_m = 0.25;
  double max_angular_accel_radps2 = 0.40;
  double progress_timeout_s = 45.0;
  double progress_min_translation_m = 0.2;
  double progress_min_rotation_rad = 0.1;
  double stock_oscillation_timeout_requested_s = 0.0;
};

struct RangerControlDecision {
  PlannerState state;
  double linear_x_mps;
  double linear_y_mps;
  double angular_z_radps;
  double command_radius_m;
  std::string reason;
};

// Final source-command slew before collision projection. Arc interpolation
// stays in the supported curvature cone; spin/arc mode changes pass via zero.
class RangerCommandLimiter {
 public:
  explicit RangerCommandLimiter(const RangerPlannerConfig& config);
  RangerControlDecision limit(const RangerControlDecision& desired,double now_s,
                              const CanonicalMotionProgress& motion);
  bool commandStopped() const;
  // The same narrow path is used by the plugin and deterministic host tests.
  // Validate the exact interpolated command before it can be published.
  template<class ArcClear,class RotationClear>
  RangerControlDecision limitAndValidate(const RangerControlDecision& desired,double now_s,
      const CanonicalMotionProgress& motion,ArcClear arc_clear,RotationClear rotation_clear) {
    auto output=limit(desired,now_s,motion);
    const bool clear=output.linear_x_mps>0 ? arc_clear(output.linear_x_mps,output.angular_z_radps) :
        output.angular_z_radps!=0 ? rotation_clear() : true;
    if(!clear) {
      reset();
      return {PlannerState::BLOCKED,0,0,0,0,"limited_command_collision_or_stale"};
    }
    return output;
  }
  void reset();
 private:
  RangerPlannerConfig config_;
  RangerControlDecision previous_{PlannerState::GOAL_REACHED,0,0,0,0,"initial"};
  double previous_time_s_=0;
  bool initialized_=false;
};

}  // namespace agx_workbench_navigation

#endif  // AGX_WORKBENCH_NAVIGATION_RANGER_CONTROL_HPP_
