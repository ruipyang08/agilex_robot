#pragma once
#include "agx_task_workflow/task_contract.hpp"
namespace agx_robot_task_runtime {
namespace resources {
inline const agx_task_workflow::TaskResource Base = "robot.base";
inline const agx_task_workflow::TaskResource Arm = "robot.arm";
inline const agx_task_workflow::TaskResource Lift = "robot.lift";
inline const agx_task_workflow::TaskResource Gripper = "robot.gripper";
inline const agx_task_workflow::TaskResource Camera = "robot.camera";
}
namespace requirements {
inline const agx_task_workflow::StateRequirementKind HeldArrival = "robot.held_arrival";
inline const agx_task_workflow::StateRequirementKind GripperEmpty = "robot.gripper_empty";
inline const agx_task_workflow::StateRequirementKind SupportedTransport = "robot.supported_transport";
inline const agx_task_workflow::StateRequirementKind SafeClearance = "robot.safe_clearance";
inline const agx_task_workflow::StateRequirementKind StationCapability = "robot.station_capability";
inline const agx_task_workflow::StateRequirementKind FixtureCapability = "robot.fixture_capability";
}
namespace effects {
inline const agx_task_workflow::ExpectedEffectKind HeldArrival = "robot.held_arrival";
inline const agx_task_workflow::ExpectedEffectKind GripperHolding = "robot.gripper_holding";
inline const agx_task_workflow::ExpectedEffectKind GripperEmpty = "robot.gripper_empty";
inline const agx_task_workflow::ExpectedEffectKind TransportReady = "robot.transport_ready";
}
}
