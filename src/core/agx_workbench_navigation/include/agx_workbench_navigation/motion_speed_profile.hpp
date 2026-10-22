#pragma once
#include <agx_workbench_navigation/motion_route.hpp>

namespace agx_workbench_navigation {
// Input samples carry geometry and optional speed ceilings (zero means no input
// ceiling). Output carries finite profile times and exact requested boundary speeds.
// Failure leaves the caller's segment unchanged.
bool ProfileForwardMotion(MotionSegment& segment, const MotionLimits& limits,
                          double start_speed, double end_speed, std::string* detail=nullptr);
MotionSegment MakeForwardPrimitive(const MotionPose& start, double start_curvature,
    double target_curvature, double hold_length_m, double start_speed, double end_speed,
    double speed_ceiling, const MotionLimits& limits, double step_m=.01);
// Starts stopped: slew steering curvature at zero propulsion, reserve measured
// settling time, then move. Runtime still requires measured steering convergence.
MotionSegment MakePreparedForwardPrimitive(const MotionPose& start, double start_curvature,
    double target_curvature, double length_m, double end_speed, double speed_ceiling,
    const MotionLimits& limits, double step_m=.01);
MotionSegment MakeSpinPrimitive(const MotionPose& start, double angle_rad,
    const MotionLimits& limits, double step_rad=.02);
}  // namespace agx_workbench_navigation
