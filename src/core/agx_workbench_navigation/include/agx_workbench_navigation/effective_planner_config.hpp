#ifndef AGX_WORKBENCH_NAVIGATION_EFFECTIVE_PLANNER_CONFIG_HPP_
#define AGX_WORKBENCH_NAVIGATION_EFFECTIVE_PLANNER_CONFIG_HPP_

#include <agx_workbench_navigation/ranger_control.hpp>
#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace agx_workbench_navigation {

using EffectiveFootprint = std::vector<std::array<double, 2>>;
template <typename Points>
EffectiveFootprint EffectivePlannerFootprint(const Points& points) {
  EffectiveFootprint result;result.reserve(points.size());
  for(const auto& point:points) result.push_back({{point.x,point.y}});
  return result;
}
struct RoverExecutionSnapshot {
  double velocity_scale=1.0, acceleration_scale=1.0, default_coefficient=1.0;
  std::map<std::string,double> task_coefficients;
};

inline std::string NewEffectivePlannerSession() {
  return boost::uuids::to_string(boost::uuids::random_generator()());
}

namespace effective_config_detail {
inline void String(std::ostream& out, const std::string& value) {
  // std::quoted escapes JSON quotes and backslashes. Control bytes are not
  // legal in component, session or policy identifiers and fail closed.
  for (unsigned char c:value) if(c<0x20) throw std::invalid_argument("invalid report identifier");
  out << std::quoted(value);
}
inline void Number(std::ostream& out, double value) {
  if(!std::isfinite(value)) throw std::invalid_argument("nonfinite effective configuration");
  out << value;
}
inline void Footprint(std::ostream& out, const EffectiveFootprint& points) {
  if(points.size()<3) throw std::invalid_argument("effective footprint needs at least three points");
  out << '[';
  for(std::size_t i=0;i<points.size();++i) {
    if(i) out << ',';
    out << '[';Number(out,points[i][0]);out << ',';Number(out,points[i][1]);out << ']';
  }
  out << ']';
}
inline void Begin(std::ostringstream& out,const std::string& component,const std::string& session) {
  if(session.empty()) throw std::invalid_argument("empty effective configuration session");
  out.imbue(std::locale::classic());
  out << std::setprecision(std::numeric_limits<double>::max_digits10);
  out << "{\"schema_version\":1,\"component\":";String(out,component);
  out << ",\"session_id\":";String(out,session);out << ",\"configuration\":{";
}
inline void Policy(std::ostream& out,const RoverExecutionSnapshot& policy) {
  out << "{\"velocity_scale\":";Number(out,policy.velocity_scale);
  out << ",\"acceleration_scale\":";Number(out,policy.acceleration_scale);
  out << ",\"default_coefficient\":";Number(out,policy.default_coefficient);
  out << ",\"task_coefficients\":{";
  bool first=true;
  for(const auto& coefficient:policy.task_coefficients) {
    if(!first) out << ',';
    first=false;String(out,coefficient.first);out << ':';Number(out,coefficient.second);
  }
  out << "}}";
}
}  // namespace effective_config_detail

inline std::string GlobalPlannerEffectiveReport(const std::string& session,
                                                const EffectiveFootprint& live_footprint) {
  std::ostringstream out;
  effective_config_detail::Begin(out,"global_planner",session);
  out << "\"footprint\":";effective_config_detail::Footprint(out,live_footprint);
  out << "}}";return out.str();
}

inline std::string LocalPlannerEffectiveReport(const std::string& session,
    const RangerPlannerConfig& loaded_limits,const RoverExecutionSnapshot& loaded_policy,
    const EffectiveFootprint& live_footprint,const EffectiveFootprint& collision_footprint) {
  using namespace effective_config_detail;
  std::ostringstream out;Begin(out,"local_planner",session);
  out << "\"footprint\":";Footprint(out,live_footprint);
  out << ",\"collision_footprint\":";Footprint(out,collision_footprint);
  out << ",\"max_linear_speed_mps\":";Number(out,loaded_limits.max_linear_speed_mps);
  out << ",\"max_spin_speed_radps\":";Number(out,loaded_limits.max_spin_speed_radps);
  out << ",\"min_turn_radius_m\":";Number(out,loaded_limits.min_turn_radius_m);
  out << ",\"max_linear_accel_mps2\":";Number(out,loaded_limits.max_linear_accel_mps2);
  out << ",\"max_linear_decel_mps2\":";Number(out,loaded_limits.max_linear_decel_mps2);
  out << ",\"max_lateral_accel_mps2\":";Number(out,loaded_limits.max_lateral_accel_mps2);
  out << ",\"max_angular_accel_radps2\":";Number(out,loaded_limits.max_angular_accel_radps2);
  out << ",\"navigation_progress\":{\"policy\":\"operation_pose_displacement\",\"clock\":\"ros_source\"";
  out << ",\"timeout_s\":";Number(out,loaded_limits.progress_timeout_s);
  out << ",\"translation_m\":";Number(out,loaded_limits.progress_min_translation_m);
  out << ",\"rotation_rad\":";Number(out,loaded_limits.progress_min_rotation_rad);
  out << ",\"parent_frame\":\"odom\",\"body_frame\":\"base_link\"";
  out << ",\"stock_oscillation_timeout_requested_s\":";Number(out,loaded_limits.stock_oscillation_timeout_requested_s);
  out << '}';
  out << ",\"rover_execution\":";Policy(out,loaded_policy);
  out << "}}";return out.str();
}

}  // namespace agx_workbench_navigation
#endif
