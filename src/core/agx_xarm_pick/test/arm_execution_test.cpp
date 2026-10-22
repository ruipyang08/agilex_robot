#include "agx_xarm_pick/arm_execution.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>
#include <iostream>

using agx_xarm_pick::ArmExecutionScales;
using agx_xarm_pick::resolve_arm_execution;

static void check(bool condition) {
  if (!condition) throw std::runtime_error("arm execution assertion failed");
}

int main() {
  const auto prep = resolve_arm_execution({.20, .50}, .60, 1.0);
  check(std::abs(prep.velocity-.20) < 1e-12);
  check(std::abs(prep.acceleration-.50) < 1e-12);
  const auto task = resolve_arm_execution({.20, .50}, .60, std::nullopt);
  check(std::abs(task.velocity-.12) < 1e-12);
  check(std::abs(task.acceleration-.30) < 1e-12);
  const auto alignment = resolve_arm_execution({.20, .50}, .60, .40);
  check(std::abs(alignment.velocity-.08) < 1e-12);
  check(std::abs(alignment.acceleration-.20) < 1e-12);
  for (double bad : {0., -1., 1.01, std::numeric_limits<double>::infinity(),
                     std::numeric_limits<double>::quiet_NaN()}) {
    for (int field=0; field<4; ++field) {
      bool rejected=false;
      try {
        resolve_arm_execution({field==0 ? bad : .20, field==1 ? bad : .50},
                              field==2 ? bad : .60, field==3 ? bad : .40);
      } catch (const std::invalid_argument&) { rejected=true; }
      check(rejected);
    }
  }
  bool underflow=false;
  try { resolve_arm_execution({std::numeric_limits<double>::denorm_min(), .5}, .5, .1); }
  catch (const std::invalid_argument&) { underflow=true; }
  check(underflow);
  std::cout << "arm execution policy: PASS\n";
}
