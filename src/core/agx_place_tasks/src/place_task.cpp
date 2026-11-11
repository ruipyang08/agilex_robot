#include "agx_place_tasks/place_task.hpp"

#include <set>
#include <stdexcept>

#include "agx_robot_task_runtime/robot_sequence_plan.hpp"
#include "agx_robot_task_runtime/task_metadata.hpp"
#include "agx_task_workflow/config.hpp"

namespace agx_place_tasks {
namespace {

using agx_task_workflow::InterfaceDirection;
using agx_task_workflow::InterfaceKind;
using agx_task_workflow::InterfaceRequirement;

bool Possible(const agx_task_workflow::PreparationState& state,
              agx_task_workflow::ExpectedEffectKind kind) {
  for (const auto& effect : state.conditional_effects)
    if (effect.effect.kind == kind) return true;
  return false;
}

void BindLifecycle(agx_task_workflow::PreparedTask& task, const PlaceTaskPorts& ports) {
  task.entry_check = [ports] {
    if (ports.entry_check) return ports.entry_check();
    return agx_task_workflow::StepResult{false, "measured entry guard adapter is not connected"};
  };
  task.stop = [ports](double timeout) {
    if (ports.stop) return ports.stop(timeout);
    agx_task_workflow::StepResult result{false, "bounded stop adapter is not connected"};
    result.failure = agx_task_workflow::FailureKind::UnconfirmedStop;
    return result;
  };
  task.cleanup = [ports](double timeout) {
    if (ports.cleanup) return ports.cleanup(timeout);
    return agx_task_workflow::StepResult{false, "bounded cleanup adapter is not connected"};
  };
}

std::vector<InterfaceRequirement> BaseInterfaces() {
  return {{InterfaceKind::Topic, "base_authority", "/workbench_nav/authority_state",
           "agx_workbench_navigation/BaseAuthorityState"},
          {InterfaceKind::Topic, "base_odometry", "/odom", "nav_msgs/Odometry"},
          {InterfaceKind::Topic, "base_command", "/cmd_vel", "geometry_msgs/Twist"},
          {InterfaceKind::Service, "base_hold", "/workbench_nav/hold_for_manipulation",
           "std_srvs/Trigger"},
          {InterfaceKind::Topic, "workflow_watchdog", "/workbench_workflow/watchdog_ready",
           "std_msgs/Bool"}};
}

std::size_t RequireSlot(const YAML::Node& node) {
  if (!node.IsScalar() || node.Tag() == "tag:yaml.org,2002:str")
    throw std::invalid_argument("slot must be an integer");
  long long slot = 0;
  try {
    slot = node.as<long long>();
  } catch (const YAML::Exception&) {
    throw std::invalid_argument("slot must be an integer");
  }
  if (slot < 1) throw std::invalid_argument("slot must be 1 or greater");
  return static_cast<std::size_t>(slot);
}

}  // namespace

void RegisterPlaceTasks(agx_task_workflow::TaskRegistry& registry, const PlaceInstallation& installed,
                        const PlaceTaskPorts& ports) {
  agx_task_workflow::TaskDefinition place;
  place.type = "place_test_tube";
  place.description =
      "Place one held tube into a configured slot of the rack mounted on this robot; no vision";
  place.inputs = {{"slot", agx_task_workflow::InputType::Integer, false, "",
                   "1-based onboard rack slot number; alternative to position"},
                  {"position", agx_task_workflow::InputType::Integer, false, "",
                   "1-based index in the measured position table; alternative to slot"},
                  {"rack_profile", agx_task_workflow::InputType::String, false, "",
                   "Optional assertion; must equal the installed onboard rack profile"}};
  place.interfaces = BaseInterfaces();
  place.interfaces.push_back({InterfaceKind::Topic, "arm_joint_state", "/joint_states",
                              "sensor_msgs/JointState"});
  place.interfaces.push_back({InterfaceKind::Action, "arm_execution", "/execute_trajectory",
                              "moveit_msgs/ExecuteTrajectoryAction"});
  place.interfaces.push_back({InterfaceKind::Action, "arm_planning", "/move_group",
                              "moveit_msgs/MoveGroupAction"});
  place.interfaces.push_back({InterfaceKind::Service, "arm_cartesian_path", "/compute_cartesian_path",
                              "moveit_msgs/GetCartesianPath"});
  place.interfaces.push_back({InterfaceKind::Topic, "arm_stop", "/trajectory_execution_event",
                              "std_msgs/String", InterfaceDirection::Command});
  place.interfaces.push_back({InterfaceKind::Topic, "gripper_control", "/gripper/ctrl",
                              "dh_gripper_msgs/GripperCtrl", InterfaceDirection::Command});
  place.interfaces.push_back({InterfaceKind::Topic, "gripper_state", "/gripper/state",
                              "dh_gripper_msgs/GripperState"});
  place.resources = {agx_robot_task_runtime::resources::Base,
                     agx_robot_task_runtime::resources::Arm,
                     agx_robot_task_runtime::resources::Gripper};
  place.requirements = {
      {agx_robot_task_runtime::requirements::HeldArrival,
       agx_task_workflow::RequirementTiming::Continuous,
       "fresh held arrival and measured stopped base"},
      {agx_robot_task_runtime::requirements::GripperHolding,
       agx_task_workflow::RequirementTiming::Entry, "fresh confirmed held tube custody"},
      {agx_robot_task_runtime::requirements::FixtureCapability,
       agx_task_workflow::RequirementTiming::Entry, "installed qualified onboard rack profile"},
      {agx_task_workflow::requirements::CommonSupervision,
       agx_task_workflow::RequirementTiming::Continuous,
       "fresh actuator feedback and watchdog"}};
  place.effects = {{agx_robot_task_runtime::effects::ObjectAtHolder,
                    agx_task_workflow::EffectCondition::VerifiedPhysicalEvent,
                    "measured slot occupancy together with a measured gripper release; never a "
                    "commanded or assumed placement"}};

  place.prepare = [installed, ports](const agx_task_workflow::TaskSpec& task,
                                     agx_task_workflow::PreparationState& state) {
    auto plan = state.planning.Get<agx_robot_task_runtime::RobotSequencePlan>(
        agx_robot_task_runtime::kRobotSequencePlan);
    if (!installed.rack) throw std::invalid_argument("no onboard rack profile is installed");
    std::string detail;
    if (!Validate(*installed.rack, &detail))
      throw std::invalid_argument("installed onboard rack profile is invalid: " + detail);
    if (!installed.rack->qualified)
      throw std::invalid_argument("onboard rack profile '" + installed.rack->profile_id +
                                  "' has no qualification evidence");
    if (!plan.held_station)
      throw std::invalid_argument("placement requires a preceding verified navigation/hold task");
    if (!plan.payload_may_be_retained)
      throw std::invalid_argument(
          "placement requires a preceding task that may retain a held tube");
    if (Possible(state, agx_robot_task_runtime::effects::ObjectAtHolder))
      throw std::invalid_argument("placement requires a tube that is not already in a fixture slot");
    const bool has_position = static_cast<bool>(task.parameters["position"]);
    const bool has_slot = static_cast<bool>(task.parameters["slot"]);
    if (has_position == has_slot)
      throw std::invalid_argument("place_test_tube takes exactly one of slot or position");
    std::set<std::string> keys{has_position ? "position" : "slot"};
    if (task.parameters["rack_profile"]) keys.insert("rack_profile");
    agx_task_workflow::RequireKeys(task.parameters, keys, "place_test_tube parameters");
    if (task.parameters["rack_profile"]) {
      const auto selected =
          agx_task_workflow::RequireString(task.parameters["rack_profile"], "rack_profile");
      if (selected != installed.rack->profile_id)
        throw std::invalid_argument("rack_profile is not the installed onboard rack");
    }
    PlaceParameters request;
    request.rack = installed.rack;
    if (has_position) {
      if (!ports.positions)
        throw std::invalid_argument("a measured position table is not installed");
      const auto index = RequireSlot(task.parameters["position"]);
      const auto* target = ports.positions->Find(std::to_string(index));
      if (!target)
        throw std::invalid_argument("position " + std::to_string(index) +
                                    " is not in the measured position table");
      request.uses_measured_position = true;
      request.measured_x = target->x;
      request.measured_y = target->y;
      request.measured_approach_z = target->approach_z;
      // Releasing at the grasp height is what the standalone cycle validated:
      // the tube drops the last few millimetres into its own hole.
      request.measured_release_z = target->place_z;
      request.measured_frame = ports.positions->frame;
      request.measured_rpy_deg = ports.positions->tool_rpy_deg;
      request.measured_yaw_offset_deg = ports.positions->tool_yaw_offset_deg;
    } else {
      request.slot = RequireSlot(task.parameters["slot"]);
      if (!installed.rack->Find(request.slot))
        throw std::invalid_argument("slot " + std::to_string(request.slot) +
                                    " was never taught for the onboard rack '" +
                                    installed.rack->profile_id + "'");
    }
    if (!ports.place || !ports.cancelled)
      throw std::invalid_argument("place execution ports are not bound");
    // The tube stays on the robot, but it is no longer carried by the gripper.
    // Onboard custody is not modelled yet; see docs/configuration.md.
    plan.payload_may_be_retained = false;
    state.planning.Set(agx_robot_task_runtime::kRobotSequencePlan, plan);
    agx_task_workflow::PreparedTask bound{
        task.id, task.type, "PLACE", [ports, request](const agx_task_workflow::PhaseSink&) {
          if (ports.cancelled()) return agx_task_workflow::StepResult{false, "cancelled"};
          return ports.place(request);
        }};
    BindLifecycle(bound, ports);
    if (ports.admit) bound.admission_check = [ports, request, id = task.id] { return ports.admit(id, request); };
    if (ports.begin) bound.begin = [ports, request] { ports.begin(request); };
    bound.continuation_check = ports.continuation_check;
    return bound;
  };
  agx_task_workflow::RegisterTask(registry, std::move(place));
}

}  // namespace agx_place_tasks
