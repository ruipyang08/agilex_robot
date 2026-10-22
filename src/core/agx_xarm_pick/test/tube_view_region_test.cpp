#include "agx_xarm_pick/tube_view_region.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

int main() {
  using agx_xarm_pick::TopDownViewRegion;
  using agx_xarm_pick::resolve_runtime_cap_top_z;
  using agx_xarm_pick::sample_top_down_view_candidates;
  using agx_xarm_pick::valid_top_down_view_region;

  const TopDownViewRegion region{
      0.45, 0.65, 0.55, 0.03, 0.25, 0.06, 0.025, 0.005, 8};
  if (!valid_top_down_view_region(region)) {
    std::cerr << "commissioned view region is invalid\n";
    return EXIT_FAILURE;
  }
  const auto candidates = sample_top_down_view_candidates(
      region, 0.0, 0.194);
  if (candidates.size() != 8 ||
      std::abs(candidates.front().tcp_x_m - 0.55) > 1e-12 ||
      std::abs(candidates.front().tcp_y_m) > 1e-12 ||
      std::abs(candidates.front().tcp_z_m - 0.254) > 1e-12 ||
      std::abs(candidates.front().preference_cost) > 1e-12) {
    std::cerr << "preference-first view samples changed\n";
    return EXIT_FAILURE;
  }
  for (const auto& candidate : candidates) {
    if (candidate.tcp_x_m < region.x_min_m ||
        candidate.tcp_x_m > region.x_max_m ||
        candidate.tcp_z_m < 0.194 + region.clearance_min_m ||
        candidate.tcp_z_m > 0.194 + region.clearance_max_m) {
      std::cerr << "view sample escaped configured bounds\n";
      return EXIT_FAILURE;
    }
  }
  if (std::abs(resolve_runtime_cap_top_z(0.194, 0.100) - 0.094) >
      1e-12) {
    std::cerr << "measured lift was not applied exactly once\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
