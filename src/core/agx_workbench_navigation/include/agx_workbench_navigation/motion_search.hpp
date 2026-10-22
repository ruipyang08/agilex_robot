#pragma once
#include <agx_workbench_navigation/motion_geometry.hpp>
#include <functional>

namespace agx_workbench_navigation {
struct MotionSearchOptions {
  std::size_t max_expansions=200000, max_nodes=20000, max_labels_per_bin=2;
  double timeout_s=5., position_bin_m=.05, heading_bin_rad=.08726646259971647;
  double soft_penalty_s_per_m=1.;
  // Prioritize goal progress within a bounded search. This is a queue-order
  // weight, not a geometry relaxation or a claim of globally optimal routing.
  double heuristic_weight=4.;
  std::function<bool()> cancelled;
};
struct MotionSearchResult {
  RouteResult result;
  std::size_t expansions=0, generated=0;
  std::size_t label_pruned=0, collision_rejected=0, duplicate_rejected=0;
  std::size_t forward_generated=0, spin_generated=0, connector_attempts=0;
  std::string budget_reason;
  double elapsed_s=0;
  bool budget_exhausted=false;
  bool collision_reported=false;
  MotionSweepResult collision;
};
// Pure onboard search over a coherent occupancy snapshot; no ROS or actuator API.
// Header supplies identities and exact start/goal. Departure is composed separately.
MotionSearchResult SearchMotionRoute(const MotionGrid& grid, const MotionGeometryPolicy& geometry,
    const MotionLimits& limits, const MotionRoute& header, const MotionSearchOptions& options={});
}  // namespace agx_workbench_navigation
