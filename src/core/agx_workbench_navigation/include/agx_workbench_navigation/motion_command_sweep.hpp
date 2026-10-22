#pragma once
#include <agx_workbench_navigation/motion_geometry.hpp>
namespace agx_workbench_navigation {
std::vector<std::vector<MotionPose>> CommandStoppingSweeps(const MotionPose& pose,
    double measured_vx,double measured_vy,double measured_w,double candidate_vx,double candidate_w,
    const MotionLimits& limits,double latency_s);
}
