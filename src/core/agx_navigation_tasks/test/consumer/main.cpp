#include <agx_navigation_tasks/navigate_task.hpp>
#include <stdexcept>
int main() {
  agx_task_workflow::TaskRegistry registry;
  agx_navigation_tasks::RegisterNavigationTasks(registry,{},{});
  const auto task=agx_task_workflow::DescribeTask(registry,"navigate");
  if(!task.metadata_complete || task.resources.empty() || task.interfaces.empty() ||
      !task.prepare || registry.size()!=1)
    throw std::runtime_error("independent module consumer contract failed");
}
