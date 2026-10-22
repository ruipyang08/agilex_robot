#include "agx_tube_tasks/grasp_execution.hpp"

namespace agx_tube_tasks {
agx_task_workflow::StepResult ExecuteGrasp(const GraspParameters& request,const GraspExecutionPorts& ports) {
  if(!ports.state || ports.run_id.empty() || ports.task_id.empty() ||
      !ports.held_base || !ports.refresh_custody || !ports.run ||
      !ports.runtime_failure || !ports.stop_reason || !ports.mission_returned)
    return {false,"grasp execution bindings are incomplete"};
  if(!ports.held_base())
    return {false,"grasp requires fresh held/stopped base evidence"};
  ports.refresh_custody();
  const auto initial=ports.state->Snapshot();
  if(initial.halted || initial.transport.pending || !initial.gripper.fresh ||
      initial.gripper.occupancy!=agx_robot_task_runtime::GripperOccupancy::Empty)
    return {false,"grasp requires current observed empty custody and no pending transfer"};
  if(!request.config)return {false,"grasp requires its admitted immutable configuration"};
  auto effects=std::make_shared<GraspEffectAdapter>(ports.state,ports.run_id,ports.task_id,
      ports.run_id+"/"+ports.task_id+"/grasp",request.config->container.id,initial.revision);
  if(ports.effects_bound)ports.effects_bound(effects);
  const auto result=ports.run(request,effects->Sink());
  ports.mission_returned(result);
  auto outcome=effects->Finish(result.outcome);
  if(ports.effects_finished)ports.effects_finished();
  const auto reason=ports.stop_reason();
  outcome.detail=agxcore::tube_mission_status_name(result.status)+
      (reason.empty()?std::string{}:"; "+reason);
  const auto interrupted=ports.runtime_failure();
  if(interrupted!=agx_task_workflow::FailureKind::None) {outcome.ok=false;outcome.failure=interrupted;}
  if(result.exit_required) {outcome.ok=false;outcome.failure=agx_task_workflow::FailureKind::UnconfirmedStop;}
  return outcome;
}
}  // namespace agx_tube_tasks
