#include <agx_tube_tasks/grasp_test_tube_task.hpp>
#include <agx_tube_tasks/grasp_execution.hpp>
#include <stdexcept>
int main() {
  if(agx_tube_tasks::ExecuteGrasp({},{}).ok)
    throw std::runtime_error("unbound independent execution must not dispatch");
  agx_task_workflow::TaskRegistry registry;
  agx_tube_tasks::RegisterTubeTasks(registry,{},{});
  const auto task=agx_task_workflow::DescribeTask(registry,"grasp_test_tube");
  if(!task.metadata_complete || task.resources.empty() || task.interfaces.empty() ||
      !task.prepare || registry.size()!=1)
    throw std::runtime_error("independent module consumer contract failed");
}
