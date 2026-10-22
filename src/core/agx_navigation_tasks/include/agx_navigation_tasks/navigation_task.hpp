#pragma once
#include "agx_task_workflow/runner.hpp"

namespace agx_navigation_tasks {
// Bound to one admitted destination and invocation. The runner invokes bounded
// stop/cleanup on failure; no subsequent port is dispatched after a failed guard.
struct NavigationTaskPorts {
  std::function<agx_task_workflow::FailureKind()> runtime_guard;
  std::function<void()> invalidate_arrival;
  // Read-only route admission precedes arrival invalidation and all physical
  // preparation. Revalidation runs after readiness and before hold release.
  std::function<agx_task_workflow::StepResult()> plan_route, revalidate_route;
  std::function<agx_task_workflow::StepResult()> establish_departure_hold, prepare_travel,
      verify_travel_ready, release_hold, travel, verify_stop,
      acquire_arrival_hold, commit_arrival;
};
agx_task_workflow::StepResult RunNavigationTask(const NavigationTaskPorts&,const agx_task_workflow::PhaseSink&);
}  // namespace agx_navigation_tasks
