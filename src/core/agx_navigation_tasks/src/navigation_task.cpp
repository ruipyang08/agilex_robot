#include "agx_navigation_tasks/navigation_task.hpp"
#include <array>

namespace agx_navigation_tasks {
agx_task_workflow::StepResult RunNavigationTask(const NavigationTaskPorts& p,const agx_task_workflow::PhaseSink& phase) {
  using Stage=std::pair<const char*,const std::function<agx_task_workflow::StepResult()>*>;
  const std::array<Stage,9> stages{{
    {"PREPARE_DEPARTURE",&p.establish_departure_hold},
    {"PREPARE_TRAVEL",&p.prepare_travel},
    {"VERIFY_TRAVEL_READY",&p.verify_travel_ready},
    {"REVALIDATE_ROUTE",&p.revalidate_route},
    {"RELEASE_BASE",&p.release_hold},
    {"NAVIGATE",&p.travel},
    {"VERIFY_STOP",&p.verify_stop},
    {"HOLD_BASE",&p.acquire_arrival_hold},
    {"VERIFY_ARRIVAL",&p.commit_arrival}}};
  if (!p.runtime_guard || !p.invalidate_arrival || !p.plan_route)
    return {false,"navigation guard, route admission and arrival invalidation must be bound"};
  for (const auto& stage:stages) if (!*stage.second)
    return {false,"all navigation physical handoff ports must be bound"};
  agx_task_workflow::StepResult result{true,"navigation admitted"};
  auto check=[&] {
    const auto failure=p.runtime_guard();
    if (failure!=agx_task_workflow::FailureKind::None) {
      result.ok=false;result.failure=failure;
      if(!result.detail.empty())result.detail+="; ";
      result.detail+="navigation interrupted at a physical boundary";
    }
    return result.ok;
  };
  auto dispatch=[&](const Stage& stage) {
    if (phase) phase(stage.first);
    if (!check()) return false;
    auto next=(*stage.second)();
    // Physical effects reported before interruption remain part of the result.
    result.physical_effect_ids.insert(result.physical_effect_ids.end(),
        next.physical_effect_ids.begin(),next.physical_effect_ids.end());
    result.state_revision=next.state_revision?next.state_revision:result.state_revision;
    result.ok=next.ok;result.detail=std::move(next.detail);
    result.failure=next.failure;result.stop=next.stop;
    return check();
  };
  if (!check() || !dispatch({"PLAN_ROUTE",&p.plan_route})) return result;
  p.invalidate_arrival();
  if (!check()) return result;
  for (const auto& stage:stages) {
    if (!dispatch(stage)) return result;
  }
  return result;
}
}  // namespace agx_navigation_tasks
