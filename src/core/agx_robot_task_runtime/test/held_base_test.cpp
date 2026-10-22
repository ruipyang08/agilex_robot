#include <cassert>
#include "agx_robot_task_runtime/held_base_monitor.hpp"
int main() {
  agx_robot_task_runtime::MotionDwell monitor;
  assert(monitor.Update(1,true,.5));
  assert(monitor.Update(2,false,.5)); // One measured spike.
  assert(monitor.Update(2.07,true,.5));
  assert(monitor.Update(3,false,.5));
  assert(monitor.Update(3.49,false,.5));
  assert(!monitor.Update(3.5,false,.5)); // Sustained motion fails at the configured dwell.
}
