#include "agx_xarm_pick/tube_top_down_planner.hpp"

#include <cstdlib>
#include <iostream>

int main() {
  using agx_xarm_pick::TopDownMotionRequest;
  using agx_xarm_pick::TopDownViewRegion;
  using agx_xarm_pick::trajectory_sample_has_minimum_clearance;
  using agx_xarm_pick::valid_top_down_motion_request;

  TopDownMotionRequest request;
  request.region = TopDownViewRegion{
      0.45, 0.65, 0.55, 0.03, 0.25, 0.06, 0.025, 0.005, 8};
  request.desired_y_m = 0.0;
  request.runtime_cap_top_z_m = 0.302564;
  request.minimum_clearance_m = 0.03;
  request.downward_axis_tolerance_rad = 0.035;
  request.planning_time_per_candidate_sec = 2.0;
  if (!valid_top_down_motion_request(request)) {
    std::cerr << "commissioned top-down request is invalid\n";
    return EXIT_FAILURE;
  }
  if (!trajectory_sample_has_minimum_clearance(
          0.332564, request.runtime_cap_top_z_m,
          request.minimum_clearance_m) ||
      trajectory_sample_has_minimum_clearance(
          0.332563, request.runtime_cap_top_z_m,
          request.minimum_clearance_m)) {
    std::cerr << "minimum-clearance contract changed\n";
    return EXIT_FAILURE;
  }
  request.downward_axis_tolerance_rad = 3.2;
  if (valid_top_down_motion_request(request)) {
    std::cerr << "unsafe orientation tolerance was accepted\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
