#ifndef AGX_WORKBENCH_NAVIGATION_ROVER_EXECUTION_HPP_
#define AGX_WORKBENCH_NAVIGATION_ROVER_EXECUTION_HPP_

#include <XmlRpcValue.h>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <agx_workbench_navigation/ranger_control.hpp>
#include <agx_workbench_navigation/effective_planner_config.hpp>

namespace agx_workbench_navigation {

inline double roverExecutionScale(const XmlRpc::XmlRpcValue& value) {
  double result;
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    result = static_cast<double>(value);
  } else if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    result = static_cast<int>(value);
  } else {
    throw std::invalid_argument("rover execution scale must be numeric");
  }
  if (!std::isfinite(result) || result <= 0.0 || result > 1.0) {
    throw std::invalid_argument("rover execution scale must be finite and in (0, 1]");
  }
  return result;
}

inline std::pair<double, double> resolveRoverScales(
    XmlRpc::XmlRpcValue policy, const std::string& task) {
  const std::set<std::string> fields{
      "velocity_scale", "acceleration_scale", "default_coefficient", "task_coefficients"};
  const std::set<std::string> tasks{
      "coarse_navigation", "visual_docking", "final_approach", "rollback"};
  if (tasks.count(task) == 0 || policy.getType() != XmlRpc::XmlRpcValue::TypeStruct ||
      policy.size() != static_cast<int>(fields.size())) {
    throw std::invalid_argument("invalid rover execution policy or task");
  }
  for (const auto& name : fields) {
    if (!policy.hasMember(name)) throw std::invalid_argument("missing rover execution field: " + name);
  }
  const double velocity = roverExecutionScale(policy["velocity_scale"]);
  const double acceleration = roverExecutionScale(policy["acceleration_scale"]);
  const double fallback = roverExecutionScale(policy["default_coefficient"]);
  auto& coefficients = policy["task_coefficients"];
  if (coefficients.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    throw std::invalid_argument("rover execution task coefficients must be a map");
  }
  const auto validate_product = [velocity, acceleration](double coefficient) {
    if (velocity * coefficient <= 0.0 || acceleration * coefficient <= 0.0) {
      throw std::invalid_argument("rover execution scale underflow");
    }
  };
  validate_product(fallback);
  for (auto it = coefficients.begin(); it != coefficients.end(); ++it) {
    if (tasks.count(it->first) == 0) throw std::invalid_argument("unknown rover execution task: " + it->first);
    validate_product(roverExecutionScale(it->second));
  }
  const double coefficient = coefficients.hasMember(task)
      ? roverExecutionScale(coefficients[task]) : fallback;
  return {velocity * coefficient, acceleration * coefficient};
}

inline RoverExecutionSnapshot snapshotRoverExecution(XmlRpc::XmlRpcValue policy) {
  // Validate the exact same loaded object used to derive controller scaling.
  resolveRoverScales(policy,"coarse_navigation");
  RoverExecutionSnapshot snapshot;
  snapshot.velocity_scale=roverExecutionScale(policy["velocity_scale"]);
  snapshot.acceleration_scale=roverExecutionScale(policy["acceleration_scale"]);
  snapshot.default_coefficient=roverExecutionScale(policy["default_coefficient"]);
  auto& coefficients=policy["task_coefficients"];
  for(auto it=coefficients.begin();it!=coefficients.end();++it)
    snapshot.task_coefficients.emplace(it->first,roverExecutionScale(it->second));
  return snapshot;
}

}  // namespace agx_workbench_navigation
#endif
