#include "agx_xarm_pick/tube_grasp_mission.hpp"
#include "agx_xarm_pick/tube_grasp_config.hpp"
#include "agx_xarm_pick/visual_alignment_motion.hpp"
#include "agx_xarm_pick/tube_grasp_geometry.hpp"
#include "agx_xarm_pick/tube_search_contract.hpp"
#include "agx_xarm_pick/tube_top_down_planner.hpp"
#include "agx_xarm_pick/visual_alignment_contract.hpp"
#include "agx_xarm_pick/tube_target_provider.hpp"
#include "agx_xarm_pick/tube_perception_projection.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/Transform.h>
#include <std_msgs/String.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <ros/master.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace agxcore {
using agx_xarm_pick::ArmMotion;
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kJointStateMatchToleranceRad = 1e-6;
constexpr double kMaximumVerticalXyDriftM = 0.002;
constexpr std::size_t kMaximumTcpRecenteringCorrections = 3;
constexpr double kMaximumInPlaceYawDriftM = 0.001;
constexpr double kPhaseOneTcpPositionToleranceM = 0.005;
constexpr auto kPollPeriod = std::chrono::milliseconds(20);

std::chrono::steady_clock::time_point PlanningDeadline(
    double timeout_sec) {
  return std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(timeout_sec));
}

double SteadySeconds(
    std::chrono::steady_clock::time_point time) {
  return std::chrono::duration<double>(
      time.time_since_epoch()).count();
}

bool GripperSourceFresh(const ros::Time& stamp, double maximum_age_s) {
  const double age = (ros::Time::now() - stamp).toSec();
  return !stamp.isZero() && std::isfinite(age) && age >= 0.0 && age <= maximum_age_s;
}

std::vector<double> OrderedMoveItArmPositions(
    const moveit::core::RobotState& state) {
  static const std::array<std::string, 6> kJointNames = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  std::vector<double> positions;
  positions.reserve(kJointNames.size());
  for (const auto& joint_name : kJointNames) {
    positions.push_back(state.getVariablePosition(joint_name));
  }
  return positions;
}

bool FinitePoint(const geometry_msgs::Point& point) {
  return std::isfinite(point.x) &&
      std::isfinite(point.y) &&
      std::isfinite(point.z);
}

bool FiniteVector(const geometry_msgs::Vector3& vector) {
  return std::isfinite(vector.x) &&
      std::isfinite(vector.y) &&
      std::isfinite(vector.z);
}

TubeMissionStatus AlignmentMissionStatus(
    agx_xarm_pick::VisualAlignmentStatus status) {
  using agx_xarm_pick::VisualAlignmentStatus;
  switch (status) {
    case VisualAlignmentStatus::kSuccess:
      return TubeMissionStatus::kSuccess;
    case VisualAlignmentStatus::kInvalidConfiguration:
      return TubeMissionStatus::kInvalidRequest;
    case VisualAlignmentStatus::kTargetUnavailable:
      return TubeMissionStatus::kDetectorUnavailable;
    case VisualAlignmentStatus::kInvalidTarget:
      return TubeMissionStatus::kInconsistentTarget;
    case VisualAlignmentStatus::kInvalidPose:
      return TubeMissionStatus::kSetupUnavailable;
    case VisualAlignmentStatus::kOrientationRejected:
      return TubeMissionStatus::kObservationExecutionFailed;
    case VisualAlignmentStatus::kWorkspaceRejected:
      return TubeMissionStatus::kOutOfWorkspace;
    case VisualAlignmentStatus::kPlanningFailed:
      return TubeMissionStatus::kObservationPlanFailed;
    case VisualAlignmentStatus::kExecutionFailed:
    case VisualAlignmentStatus::kInvalidEndpoint:
      return TubeMissionStatus::kObservationExecutionFailed;
    case VisualAlignmentStatus::kNoImprovement:
    case VisualAlignmentStatus::kIterationLimit:
    case VisualAlignmentStatus::kDisplacementLimit:
      return TubeMissionStatus::kRecenteringExhausted;
    case VisualAlignmentStatus::kCancelled:
      return TubeMissionStatus::kInterrupted;
    case VisualAlignmentStatus::kInternalError:
      return TubeMissionStatus::kSetupUnavailable;
  }
  return TubeMissionStatus::kSetupUnavailable;
}

geometry_msgs::Transform RosTransform(const agx_xarm_pick::ConfigTransform& transform) {
  geometry_msgs::Transform result;
  result.translation.x=transform.translation.x;
  result.translation.y=transform.translation.y;
  result.translation.z=transform.translation.z;
  result.rotation.x=transform.rotation.x;
  result.rotation.y=transform.rotation.y;
  result.rotation.z=transform.rotation.z;
  result.rotation.w=transform.rotation.w;
  return result;
}

Eigen::Isometry3d TransformToEigen(
    const agx_xarm_pick::ConfigTransform& transform) {
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  const Eigen::Quaterniond rotation(
      transform.rotation.w,
      transform.rotation.x,
      transform.rotation.y,
      transform.rotation.z);
  result.linear() = rotation.normalized().toRotationMatrix();
  result.translation() = Eigen::Vector3d(
      transform.translation.x,
      transform.translation.y,
      transform.translation.z);
  return result;
}

}  // namespace

TubeGraspMission::TubeGraspMission(ros::NodeHandle* node)
    : TubeGraspMission(node, node ? SnapshotLegacyTubeTaskConfig(*node) : nullptr, "") {}

TubeGraspMission::TubeGraspMission(ros::NodeHandle* node,
    std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig> resolved,
    const std::string& color, TubeEffectSink effect_sink, std::function<bool()> held_base)
    : node_(node), resolved_config_(std::move(resolved)), effects_(std::move(effect_sink)),
      held_base_(std::move(held_base)) {
  if (node_ == nullptr || !resolved_config_) {
    throw std::invalid_argument("tube mission requires a node and admitted configuration");
  }
  config_ = std::make_shared<const agx_xarm_pick::TubeTaskMotionConfig>(
      agx_xarm_pick::BindTubeTaskMotion(*resolved_config_,
          color.empty() ? resolved_config_->motion.target_cap_color : color));
  if (!ValidateTubeMotionConfig(*config_))
    throw std::runtime_error("invalid /tube_grasp configuration");
  observer_ = MakeTubeLiveObserver(*node_, *resolved_config_, *config_);
  lift_target_ = agx_xarm_pick::compute_lift_target(
      config_->table_surface_height_above_floor_m,
      config_->arm_base_height_at_zero_lift_above_floor_m,
      config_->lift_minimum_position_m, config_->lift_maximum_position_m);
  detector_client_ =
      node_->serviceClient<agx_pick_msg::DetectTubeCapConfigured>(resolved_config_->robot.detector_service + "_configured");
  lift_client_ =
      node_->serviceClient<bt_task_msgs::LiftMotorSrv>(
          config_->lift_service_name);
  lift_stop_client_ = node_->serviceClient<bt_task_msgs::LiftMotorSrv>(config_->lift_service_name);
  arm_stop_publisher_ = node_->advertise<std_msgs::String>("/trajectory_execution_event", 1);
  gripper_publisher_ =
      node_->advertise<dh_gripper_msgs::GripperCtrl>(resolved_config_->robot.gripper_command_topic, 1);
  gripper_subscriber_ =
      node_->subscribe<dh_gripper_msgs::GripperState>(
          resolved_config_->robot.gripper_state_topic, 5,
          &TubeGraspMission::GripperStateCallback, this);
  joint_state_subscriber_ =
      node_->subscribe<sensor_msgs::JointState>(
          resolved_config_->robot.joint_state_topic, 5,
          &TubeGraspMission::JointStateCallback, this);
  lift_state_subscriber_ =
      node_->subscribe<bt_task_msgs::LiftMotorMsg>(
          config_->lift_state_topic, 5,
          &TubeGraspMission::LiftStateCallback, this);
  move_group_ =
      std::make_unique<
          moveit::planning_interface::MoveGroupInterface>("xarm6");
  move_group_->setPoseReferenceFrame(config_->planning_frame);
  move_group_->setEndEffectorLink("link6");
  move_group_->setPlanningTime(config_->planning_timeout_sec);
  ApplyArmExecution(ArmMotion::Phase1Preparation);
}

bool TubeGraspMission::RecordEffect(TubeEffectKind kind, const TubeEvidence& evidence) {
  if (effects_.Record(kind, evidence)) return true;
  RequestStop();
  last_gripper_wait_status_ = TubeMissionStatus::kPayloadStateUnsafe;
  return false;
}

bool TubeGraspMission::EmitGripperEffect(TubeEffectKind kind) {
  std::lock_guard<std::mutex> effect_lock(effects_state_mutex_);
  TubeEvidence evidence;
  bool fresh = false, caught = false, empty = false;
  {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    evidence = {resolved_config_->robot.gripper_state_topic,
        latest_gripper_state_.header.stamp.toSec(), SteadySeconds(latest_gripper_wall_time_)};
    const double receipt_age = SteadySeconds(std::chrono::steady_clock::now()) - evidence.receipt_steady_s;
    fresh = has_gripper_state_ && receipt_age >= 0 && receipt_age <= config_->gripper_state_max_age_sec &&
        GripperSourceFresh(latest_gripper_state_.header.stamp, config_->gripper_state_max_age_sec);
    caught = fresh && known_caught_gripper_state(latest_gripper_state_.is_initialized,
        latest_gripper_state_.grip_state, latest_gripper_state_.target_position,
        config_->close_command, config_->gripper_command_tolerance);
    empty = fresh && (known_open_gripper_state(latest_gripper_state_.is_initialized,
        latest_gripper_state_.grip_state, latest_gripper_state_.position,
        latest_gripper_state_.target_position, config_->fully_open_command,
        config_->gripper_command_tolerance) ||
        (payload_ == PayloadKnowledge::kInsertionAperture && current_aperture_evidence_is_valid(
            has_gripper_state_, latest_gripper_state_.is_initialized, latest_gripper_state_.grip_state,
            latest_gripper_state_.position, latest_gripper_state_.target_position,
            latest_gripper_state_.target_position, config_->gripper_command_tolerance,
            evidence.receipt_steady_s, SteadySeconds(gripper_command_wall_time_),
            SteadySeconds(std::chrono::steady_clock::now()), config_->gripper_state_max_age_sec)));
  }
  if (((kind == TubeEffectKind::Captured || kind == TubeEffectKind::Retained) && !caught) ||
      (kind == TubeEffectKind::KnownEmpty && !empty)) {
    payload_ = PayloadKnowledge::kUnknown;
    RecordEffect(TubeEffectKind::RetentionUncertain, evidence);
    RequestStop();
    last_gripper_wait_status_ = TubeMissionStatus::kRetentionLost;
    return false;
  }
  if (kind == TubeEffectKind::Captured || kind == TubeEffectKind::Retained) payload_ = PayloadKnowledge::kHolding;
  if (kind == TubeEffectKind::RetentionUncertain) payload_ = PayloadKnowledge::kUnknown;
  return RecordEffect(kind, evidence);
}

TubeEvidence TubeGraspMission::ArmEvidence() const {
  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  return {resolved_config_->robot.joint_state_topic, latest_robot_state_snapshot_.source_stamp.toSec(),
      SteadySeconds(latest_robot_state_snapshot_.receipt_time)};
}

void TubeGraspMission::RefreshPayloadEvidence() {
  std::lock_guard<std::mutex> effect_lock(effects_state_mutex_);
  if (payload_ != PayloadKnowledge::kHolding) return;
  TubeEvidence evidence;
  bool retained = false;
  {
    std::lock_guard<std::mutex> lock(gripper_mutex_);
    evidence = {resolved_config_->robot.gripper_state_topic,
        latest_gripper_state_.header.stamp.toSec(), SteadySeconds(latest_gripper_wall_time_)};
    const double age = SteadySeconds(std::chrono::steady_clock::now()) - evidence.receipt_steady_s;
    retained = has_gripper_state_ && age >= 0 && age <= config_->gripper_state_max_age_sec &&
        GripperSourceFresh(latest_gripper_state_.header.stamp, config_->gripper_state_max_age_sec) &&
        known_caught_gripper_state(latest_gripper_state_.is_initialized, latest_gripper_state_.grip_state,
            latest_gripper_state_.target_position, config_->close_command, config_->gripper_command_tolerance);
  }
  if (!retained) {
    payload_ = PayloadKnowledge::kUnknown;
    // Preserve the actual stale sample timestamps. The workflow store will
    // invalidate custody; dispatch time must not freshen old device evidence.
    RecordEffect(TubeEffectKind::RetentionUncertain, evidence);
    RequestStop();
  }
}

TubeMissionOutcome TubeGraspMission::Outcome(TubeMissionStatus status) {
  RefreshPayloadEvidence();
  return effects_.Outcome(status);
}

bool TubeGraspMission::ObserveStopped(const TubeEvidence& stop_request) {
  if (!observer_ || !observer_->Snapshot().ArmLiftStoppedSince(stop_request)) return false;
  if (!effects_.Outcome(TubeMissionStatus::kInterrupted).measured_stop_confirmed)
    RecordEffect(TubeEffectKind::MeasuredStop, ArmEvidence());
  return true;
}

agx_xarm_pick::ArmExecutionScales TubeGraspMission::ArmScales(ArmMotion motion) const {
  return config_->arm_execution.at(static_cast<std::size_t>(motion));
}

void TubeGraspMission::ApplyArmExecution(ArmMotion motion) {
  const auto scales = ArmScales(motion);
  move_group_->setMaxVelocityScalingFactor(scales.velocity);
  move_group_->setMaxAccelerationScalingFactor(scales.acceleration);
  ROS_INFO("tube arm execution: %s velocity=%.3f acceleration=%.3f",
           agx_xarm_pick::kArmMotionNames.at(static_cast<std::size_t>(motion)),
           scales.velocity, scales.acceleration);
}

TubeGraspMission::~TubeGraspMission() {
  // The monitored runner can finish while its caller's spinner is still
  // running. Unregister and quiesce callbacks before their buffers, mutexes
  // and configuration are destroyed by reverse member destruction order.
  gripper_subscriber_.shutdown();
  joint_state_subscriber_.shutdown();
  lift_state_subscriber_.shutdown();
  std::lock_guard<std::mutex> lock(stop_rpc_mutex_);
  lift_stop_retry_=false;
  ReapLiftStopLocked();
  // A false FinishStopRequests result is terminal. Its independent client and
  // completion flag may outlive destruction, never a reference to this mission.
  if(lift_stop_worker_.joinable())lift_stop_worker_.detach();
}

void TubeGraspMission::RequestStop() {
  RequestStopInternal(false);
}

void TubeGraspMission::RequestStopInternal(bool final) {
  stop_requested_.store(true);
  std_msgs::String event;
  event.data = "stop";
  arm_stop_publisher_.publish(event);
  std::lock_guard<std::mutex> lock(stop_rpc_mutex_);
  ReapLiftStopLocked();
  if (stop_rpcs_sealed_ || stop_rpc_failed_) return;
  if (final) stop_rpcs_sealed_ = true;
  if (!lift_stop_worker_.joinable()) LaunchLiftStopLocked();
  else if (final) lift_stop_retry_ = true;
}

void TubeGraspMission::LaunchLiftStopLocked() {
  if (std::chrono::steady_clock::now() >= stop_rpc_deadline_) {
    stop_rpc_failed_ = true;
    return;
  }
  auto client = lift_stop_client_;
  auto active = lift_stop_sent_;
  active->store(true);
  try {
    lift_stop_worker_ = std::thread([client, active]() mutable {
      try {
        bt_task_msgs::LiftMotorSrv stop;
        stop.request.mode = -2;
        stop.request.val = 0;
        client.call(stop);
      } catch (...) {}
      active->store(false);
    });
  } catch (...) {
    active->store(false);
    // Keep RequestStop nonthrowing when the independent worker cannot start.
    // The monitor will report terminal cleanup through FinishStopRequests.
    stop_rpc_failed_ = true;
  }
}

void TubeGraspMission::ReapLiftStopLocked() {
  if (lift_stop_worker_.joinable() && !lift_stop_sent_->load()) lift_stop_worker_.join();
  if (lift_stop_retry_ && !lift_stop_worker_.joinable()) {
    lift_stop_retry_ = false;
    LaunchLiftStopLocked();
  }
}

bool TubeGraspMission::FinishStopRequests(double timeout_s) {
  const auto budget = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::isfinite(timeout_s) ?
          std::max(0.0, std::min(timeout_s, 60.0)) : 0.0));
  std::chrono::steady_clock::time_point deadline;
  {
    std::lock_guard<std::mutex> lock(stop_rpc_mutex_);
    // Repeated cleanup cannot extend the stop budget, including callbacks that
    // reap an earlier RPC and would otherwise dispatch its retry after expiry.
    stop_rpc_deadline_ = std::min(stop_rpc_deadline_, std::chrono::steady_clock::now() + budget);
    deadline = stop_rpc_deadline_;
  }
  if (budget.count() > 0) RequestStopInternal(true);
  while (ros::ok() && std::chrono::steady_clock::now() < deadline) {
    RefreshPayloadEvidence();
    {
      std::lock_guard<std::mutex> lock(stop_rpc_mutex_);
      ReapLiftStopLocked();
      if (stop_rpc_failed_) return false;
      if (!lift_stop_worker_.joinable() && !lift_stop_retry_) return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::lock_guard<std::mutex> lock(stop_rpc_mutex_);
  stop_rpcs_sealed_ = true;
  lift_stop_retry_ = false;
  stop_rpc_failed_ = true;
  ReapLiftStopLocked();
  return false;
}


void TubeGraspMission::GripperStateCallback(
    const dh_gripper_msgs::GripperState::ConstPtr& message) {
  std::lock_guard<std::mutex> lock(gripper_mutex_);
  latest_gripper_state_ = *message;
  latest_gripper_wall_time_ = std::chrono::steady_clock::now();
  has_gripper_state_ = true;
}

void TubeGraspMission::JointStateCallback(
    const sensor_msgs::JointState::ConstPtr& message) {
  std::vector<double> ordered_positions;
  const bool valid = !message->header.stamp.isZero() &&
      extract_required_arm_joint_positions(
          message->name, message->position, &ordered_positions);

  std::lock_guard<std::mutex> lock(joint_state_mutex_);
  latest_robot_state_snapshot_.valid = valid;
  if (valid) {
    latest_robot_state_snapshot_.source_stamp = message->header.stamp;
    latest_robot_state_snapshot_.receipt_time =
        std::chrono::steady_clock::now();
    latest_robot_state_snapshot_.message = *message;
    latest_robot_state_snapshot_.ordered_positions =
        std::move(ordered_positions);
  } else {
    latest_robot_state_snapshot_.message = sensor_msgs::JointState();
    latest_robot_state_snapshot_.ordered_positions.clear();
  }
}

void TubeGraspMission::LiftStateCallback(
    const bt_task_msgs::LiftMotorMsg::ConstPtr& message) {
  const bool fault_free =
      agx_xarm_pick::lift_feedback_is_fault_free(
          message->overflowI,
          message->overflowV,
          message->encodeErr,
          message->posBiasOver,
          message->underVol,
          message->overLoad,
          message->externalCtrlMode,
          message->stopFlag,
          message->fpsError);

  std::lock_guard<std::mutex> lock(lift_mutex_);
  const auto receipt_time = std::chrono::steady_clock::now();
  const ros::Time ros_receipt_time = ros::Time::now();
  const bool stamp_valid =
      !message->header.stamp.isZero() &&
      message->header.stamp <= ros_receipt_time;
  const double stamp_age_sec = stamp_valid
      ? (ros_receipt_time - message->header.stamp).toSec()
      : std::numeric_limits<double>::infinity();
  const bool fresh_at_receipt =
      stamp_valid &&
      stamp_age_sec >= 0.0 &&
      stamp_age_sec <= config_->lift_state_max_age_sec;
  ++lift_feedback_epoch_;
  latest_lift_state_ = *message;
  latest_lift_wall_time_ = receipt_time;
  has_lift_state_ = true;
  if (fresh_at_receipt && !fault_free) {
    latest_lift_fault_epoch_ = lift_feedback_epoch_;
    latest_lift_fault_wall_time_ = receipt_time;
  }
}

bool TubeGraspMission::WaitForSynchronizedRobotState(
    std::chrono::steady_clock::time_point minimum_receipt_time,
    std::chrono::steady_clock::time_point deadline,
    moveit::core::RobotStatePtr* synchronized_state) const {
  if (synchronized_state == nullptr) {
    return false;
  }
  synchronized_state->reset();
  const auto robot_model = move_group_->getRobotModel();
  const auto* joint_model_group = robot_model
      ? robot_model->getJointModelGroup("xarm6")
      : nullptr;
  if (!robot_model || joint_model_group == nullptr) {
    return false;
  }
  while (ContinueRunning() && std::chrono::steady_clock::now() < deadline) {
    RobotStateSnapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      snapshot = latest_robot_state_snapshot_;
    }

    if (snapshot.valid &&
        snapshot.receipt_time >= minimum_receipt_time) {
      moveit::core::RobotStatePtr moveit_state(
          new moveit::core::RobotState(robot_model));
      moveit_state->setToDefaultValues();
      const bool converted = moveit::core::jointStateToRobotState(
          snapshot.message, *moveit_state);
      moveit_state->updateLinkTransforms();
      const auto wall_now = std::chrono::steady_clock::now();
      if (converted &&
          moveit_state->satisfiesBounds(joint_model_group) &&
          synchronized_robot_state_evidence_is_valid(
              snapshot.valid,
              snapshot.source_stamp.toSec(),
              ros::Time::now().toSec(),
              SteadySeconds(snapshot.receipt_time),
              SteadySeconds(wall_now),
              SteadySeconds(minimum_receipt_time),
              config_->robot_state_max_age_sec,
              snapshot.ordered_positions,
              OrderedMoveItArmPositions(*moveit_state),
              kJointStateMatchToleranceRad)) {
        *synchronized_state = moveit_state;
        return true;
      }
    }
    std::this_thread::sleep_for(kPollPeriod);
  }
  return false;
}

bool TubeGraspMission::WaitForKnownOpenState() {
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(config_->gripper_timeout_sec));
  while (ContinueRunning() && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(gripper_mutex_);
      const auto now = std::chrono::steady_clock::now();
      const bool fresh = has_gripper_state_ &&
          GripperSourceFresh(latest_gripper_state_.header.stamp, config_->gripper_state_max_age_sec) &&
          std::chrono::duration<double>(
              now - latest_gripper_wall_time_).count() <=
              config_->gripper_state_max_age_sec;
      if (fresh && known_open_gripper_state(
          latest_gripper_state_.is_initialized,
          latest_gripper_state_.grip_state,
          latest_gripper_state_.position,
          latest_gripper_state_.target_position,
          config_->fully_open_command,
          config_->gripper_command_tolerance)) {
        last_gripper_wait_status_ = TubeMissionStatus::kSuccess;
        return true;
      }
    }
    std::this_thread::sleep_for(kPollPeriod);
  }
  last_gripper_wait_status_ = ContinueRunning()
      ? TubeMissionStatus::kGripperStateUnavailable
      : TubeMissionStatus::kInterrupted;
  return false;
}

bool TubeGraspMission::WaitForGripperPosition(double target_position) {
  const auto deadline = gripper_command_wall_time_ +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(config_->gripper_timeout_sec));
  bool received_fresh_state = false;
  while (ContinueRunning() && std::chrono::steady_clock::now() < deadline) {
    bool position_verified = false;
    {
      std::lock_guard<std::mutex> lock(gripper_mutex_);
      const auto now = std::chrono::steady_clock::now();
      const bool newer_than_command = has_gripper_state_ &&
          latest_gripper_wall_time_ > gripper_command_wall_time_;
      received_fresh_state = received_fresh_state || newer_than_command;
      position_verified = current_aperture_evidence_is_valid(
          has_gripper_state_,
          latest_gripper_state_.is_initialized,
          latest_gripper_state_.grip_state,
          latest_gripper_state_.position,
          latest_gripper_state_.target_position,
          target_position,
          config_->gripper_command_tolerance,
          SteadySeconds(latest_gripper_wall_time_),
          SteadySeconds(gripper_command_wall_time_),
          SteadySeconds(now),
          config_->gripper_state_max_age_sec);
    }
    if (position_verified) {
      last_gripper_wait_status_ = TubeMissionStatus::kSuccess;
      return true;
    }
    std::this_thread::sleep_for(kPollPeriod);
  }
  if (!ContinueRunning()) {
    last_gripper_wait_status_ = TubeMissionStatus::kInterrupted;
  } else if (!received_fresh_state) {
    last_gripper_wait_status_ = TubeMissionStatus::kGripperTimeout;
  } else {
    last_gripper_wait_status_ =
        TubeMissionStatus::kInsertionApertureFailed;
  }
  return false;
}

bool TubeGraspMission::ApertureEvidenceIsCurrent(
    double target_position) const {
  std::lock_guard<std::mutex> lock(gripper_mutex_);
  const auto now = std::chrono::steady_clock::now();
  return current_aperture_evidence_is_valid(
      has_gripper_state_,
      latest_gripper_state_.is_initialized,
      latest_gripper_state_.grip_state,
      latest_gripper_state_.position,
      latest_gripper_state_.target_position,
      target_position,
      config_->gripper_command_tolerance,
      SteadySeconds(latest_gripper_wall_time_),
      SteadySeconds(gripper_command_wall_time_),
      SteadySeconds(now),
      config_->gripper_state_max_age_sec);
}

bool TubeGraspMission::WaitForCaughtState() {
  const auto deadline = gripper_command_wall_time_ +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(config_->gripper_timeout_sec));
  bool received_fresh_state = false;
  bool tracking_caught_state = false;
  std::chrono::steady_clock::time_point caught_since;
  while (ContinueRunning() && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(gripper_mutex_);
      const auto now = std::chrono::steady_clock::now();
      const bool current = has_gripper_state_ &&
          latest_gripper_wall_time_ >= gripper_command_wall_time_;
      const bool fresh = current &&
          GripperSourceFresh(latest_gripper_state_.header.stamp, config_->gripper_state_max_age_sec) &&
          std::chrono::duration<double>(
              now - latest_gripper_wall_time_).count() <=
              config_->gripper_state_max_age_sec;
      received_fresh_state = received_fresh_state || fresh;
      const bool caught_state =
          latest_gripper_state_.grip_state == 2;
      if (fresh && caught_state && known_caught_gripper_state(
          latest_gripper_state_.is_initialized,
          latest_gripper_state_.grip_state,
          latest_gripper_state_.target_position,
          config_->close_command,
          config_->gripper_command_tolerance)) {
        if (!tracking_caught_state) {
          caught_since = now;
          tracking_caught_state = true;
        }
        const bool caught_state_stable =
            std::chrono::duration<double>(now - caught_since).count() >=
            config_->hold_duration_sec;
        if (caught_state_stable) {
          last_gripper_wait_status_ = TubeMissionStatus::kSuccess;
          return true;
        }
      } else {
        tracking_caught_state = false;
      }
    }
    std::this_thread::sleep_for(kPollPeriod);
  }
  if (!ContinueRunning()) {
    last_gripper_wait_status_ = TubeMissionStatus::kInterrupted;
  } else if (!received_fresh_state) {
    last_gripper_wait_status_ = TubeMissionStatus::kGripperTimeout;
  } else {
    last_gripper_wait_status_ = TubeMissionStatus::kCaughtStateRequired;
  }
  return false;
}

bool TubeGraspMission::HoldCaughtState() {
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(config_->hold_duration_sec));
  while (ContinueRunning() && std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(gripper_mutex_);
      const auto now = std::chrono::steady_clock::now();
      const bool fresh = has_gripper_state_ &&
          GripperSourceFresh(latest_gripper_state_.header.stamp, config_->gripper_state_max_age_sec) &&
          std::chrono::duration<double>(
              now - latest_gripper_wall_time_).count() <=
              config_->gripper_state_max_age_sec;
      if (!fresh || !known_caught_gripper_state(
          latest_gripper_state_.is_initialized,
          latest_gripper_state_.grip_state,
          latest_gripper_state_.target_position,
          config_->close_command,
          config_->gripper_command_tolerance)) {
        last_gripper_wait_status_ = TubeMissionStatus::kRetentionLost;
        payload_ = PayloadKnowledge::kUnknown;
        RecordEffect(TubeEffectKind::RetentionUncertain,
            {resolved_config_->robot.gripper_state_topic, latest_gripper_state_.header.stamp.toSec(),
             SteadySeconds(latest_gripper_wall_time_)});
        return false;
      }
    }
    std::this_thread::sleep_for(kPollPeriod);
  }
  last_gripper_wait_status_ = ContinueRunning()
      ? TubeMissionStatus::kSuccess
      : TubeMissionStatus::kInterrupted;
  return ContinueRunning();
}

TubeMissionStatus TubeGraspMission::MoveTopDown(
    ArmMotion motion,
    const agx_xarm_pick::TopDownViewRegion& region,
    double desired_y_m,
    double minimum_clearance_m,
    TubeMissionStatus plan_failure,
    TubeMissionStatus execution_failure,
    geometry_msgs::Pose* measured_tcp,
    const TubeObservation* required_fresh_observation) {
  if (!ContinueRunning()) {
    return TubeMissionStatus::kInterrupted;
  }
  if (!runtime_geometry_resolved_) {
    return plan_failure;
  }
  agx_xarm_pick::TubeTopDownPlanner planner(
      move_group_.get(),
      config_->planning_frame,
      "link6",
      TransformToEigen(config_->link6_to_grasp_tcp));
  agx_xarm_pick::TopDownMotionRequest request;
  request.region = region;
  request.desired_y_m = desired_y_m;
  request.runtime_cap_top_z_m = runtime_cap_top_z_m_;
  request.minimum_clearance_m = minimum_clearance_m;
  request.downward_axis_tolerance_rad =
      config_->downward_axis_tolerance_deg * kPi / 180.0;
  request.planning_time_per_candidate_sec =
      config_->view_planning_time_per_candidate_sec;
  const auto planning_deadline =
      PlanningDeadline(config_->planning_timeout_sec);
  moveit::core::RobotStatePtr planning_state;
  if (!WaitForSynchronizedRobotState(
          std::chrono::steady_clock::time_point{},
          planning_deadline,
          &planning_state)) {
    return plan_failure;
  }
  if (required_fresh_observation != nullptr &&
      !ObservationIsFresh(*required_fresh_observation)) {
    return TubeMissionStatus::kStaleFrame;
  }

  ApplyArmExecution(motion);
  auto result = planner.Plan(request, *planning_state);
  move_group_->setPlanningTime(config_->planning_timeout_sec);
  if (!result.has_value()) {
    ROS_ERROR(
        "tube top-down planner exhausted %zu candidates at y=%.6f",
        region.maximum_candidate_attempts,
        desired_y_m);
    return plan_failure;
  }
  if (!ContinueRunning()) {
    return TubeMissionStatus::kInterrupted;
  }
  if (required_fresh_observation != nullptr &&
      !ObservationIsFresh(*required_fresh_observation)) {
    return TubeMissionStatus::kStaleFrame;
  }
  if (move_group_->execute(result->plan) !=
      moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    return execution_failure;
  }
  const auto execution_complete = std::chrono::steady_clock::now();
  geometry_msgs::Pose measured;
  geometry_msgs::Point requested_tcp;
  requested_tcp.x = result->candidate.tcp_x_m;
  requested_tcp.y = result->candidate.tcp_y_m;
  requested_tcp.z = result->candidate.tcp_z_m;
  if (!CurrentTcpPose(
          execution_complete + std::chrono::steady_clock::duration(1),
          PlanningDeadline(config_->planning_timeout_sec),
          &measured) ||
      !ValidateExecutedTcp(requested_tcp, measured) ||
      !CameraAxisIsDownwardAtTcpPose(measured)) {
    return execution_failure;
  }
  if (measured_tcp != nullptr) {
    *measured_tcp = measured;
  }
  ROS_INFO(
      "tube top-down TCP reached candidate=(%.6f, %.6f, %.6f) "
      "measured=(%.6f, %.6f, %.6f)",
      requested_tcp.x,
      requested_tcp.y,
      requested_tcp.z,
      measured.position.x,
      measured.position.y,
      measured.position.z);
  return TubeMissionStatus::kSuccess;
}

bool TubeGraspMission::CurrentTcpPose(
    std::chrono::steady_clock::time_point minimum_receipt_time,
    std::chrono::steady_clock::time_point deadline,
    geometry_msgs::Pose* tcp_pose,
    moveit::core::RobotStatePtr* synchronized_state) const {
  if (tcp_pose == nullptr) {
    return false;
  }
  moveit::core::RobotStatePtr current_state;
  if (!WaitForSynchronizedRobotState(
          minimum_receipt_time, deadline, &current_state) ||
      !TcpPoseFromState(*current_state, tcp_pose)) {
    return false;
  }
  if (synchronized_state != nullptr) {
    *synchronized_state = current_state;
  }
  return true;
}

bool TubeGraspMission::TcpPoseFromState(
    const moveit::core::RobotState& state,
    geometry_msgs::Pose* tcp_pose) const {
  if (tcp_pose == nullptr) {
    return false;
  }
  const Eigen::Isometry3d model_to_planning =
      state.getGlobalLinkTransform(config_->planning_frame);
  const Eigen::Isometry3d model_to_link6 =
      state.getGlobalLinkTransform("link6");
  const Eigen::Isometry3d planning_to_link6_eigen =
      model_to_planning.inverse() * model_to_link6;
  const Eigen::Quaterniond orientation(
      planning_to_link6_eigen.rotation());
  tf2::Transform planning_to_link6;
  planning_to_link6.setOrigin(tf2::Vector3(
      planning_to_link6_eigen.translation().x(),
      planning_to_link6_eigen.translation().y(),
      planning_to_link6_eigen.translation().z()));
  planning_to_link6.setRotation(tf2::Quaternion(
      orientation.x(),
      orientation.y(),
      orientation.z(),
      orientation.w()));
  tf2::Transform link6_to_tcp;
  tf2::fromMsg(RosTransform(config_->link6_to_grasp_tcp), link6_to_tcp);
  const tf2::Transform planning_to_tcp =
      planning_to_link6 * link6_to_tcp;

  tcp_pose->position.x = planning_to_tcp.getOrigin().x();
  tcp_pose->position.y = planning_to_tcp.getOrigin().y();
  tcp_pose->position.z = planning_to_tcp.getOrigin().z();
  tcp_pose->orientation = tf2::toMsg(planning_to_tcp.getRotation());
  return true;
}

bool TubeGraspMission::ValidateExecutedTcp(
    const geometry_msgs::Point& requested_tcp,
    const geometry_msgs::Pose& measured_tcp) const {
  const double position_tolerance_m =
      execution_position_tolerance_m(
          config_->view_region.y_tolerance_m);
  const bool position_valid = tcp_pose_within_tolerance(
      requested_tcp.x,
      requested_tcp.y,
      requested_tcp.z,
      measured_tcp.position.x,
      measured_tcp.position.y,
      measured_tcp.position.z,
      position_tolerance_m);
  const bool orientation_valid = tcp_approach_axis_is_downward(
      measured_tcp.orientation.x,
      measured_tcp.orientation.y,
      measured_tcp.orientation.z,
      measured_tcp.orientation.w,
      config_->downward_axis_tolerance_deg);
  if (!position_valid || !orientation_valid) {
    ROS_ERROR(
        "tube TCP invariant failed: requested=(%.6f, %.6f, %.6f) "
        "measured=(%.6f, %.6f, %.6f) position_ok=%d downward_ok=%d",
        requested_tcp.x,
        requested_tcp.y,
        requested_tcp.z,
        measured_tcp.position.x,
        measured_tcp.position.y,
        measured_tcp.position.z,
        position_valid,
        orientation_valid);
  }
  return position_valid && orientation_valid;
}

bool TubeGraspMission::CameraAxisIsDownwardAtTcpPose(
    const geometry_msgs::Pose& tcp_pose) const {
  return CameraAxisIsDownwardAtTcpPose(
      tcp_pose, config_->downward_axis_tolerance_deg);
}

bool TubeGraspMission::CameraAxisIsDownwardAtTcpPose(
    const geometry_msgs::Pose& tcp_pose,
    double tolerance_deg) const {
  if (!std::isfinite(tolerance_deg) ||
      tolerance_deg <= 0.0 ||
      tolerance_deg > 90.0) {
    return false;
  }
  tf2::Transform planning_to_grasp_tcp;
  tf2::fromMsg(tcp_pose, planning_to_grasp_tcp);
  tf2::Transform link6_to_grasp_tcp;
  tf2::fromMsg(RosTransform(config_->link6_to_grasp_tcp), link6_to_grasp_tcp);
  tf2::Transform link6_to_camera;
  tf2::fromMsg(RosTransform(config_->link6_to_camera), link6_to_camera);
  const tf2::Transform planning_to_link6 =
      planning_to_grasp_tcp * link6_to_grasp_tcp.inverse();
  const tf2::Transform planning_to_camera =
      planning_to_link6 * link6_to_camera;
  const tf2::Quaternion camera_orientation =
      planning_to_camera.getRotation();
  return tcp_approach_axis_is_downward(
      camera_orientation.x(),
      camera_orientation.y(),
      camera_orientation.z(),
      camera_orientation.w(),
      tolerance_deg);
}

bool TubeGraspMission::ObservationIsFresh(
    const TubeObservation& observation) const {
  return stamped_observation_is_fresh(
      observation.cap_top.header.frame_id == config_->planning_frame,
      observation.cap_top.header.stamp.toSec(),
      ros::Time::now().toSec(),
      config_->maximum_frame_age_sec);
}

bool TubeGraspMission::BaseStoppedIfAvailable() const {
  ros::master::V_TopicInfo topics;
  if (!ros::master::getTopics(topics)) {
    ROS_ERROR("could not inspect ROS topics for optional base-stop input");
    return false;
  }
  const bool input_available = std::any_of(
      topics.begin(), topics.end(),
      [](const ros::master::TopicInfo& topic) {
        return topic.name == "/rover/base_is_stopped";
      });
  const bool safe =
      headerless_optional_evidence_is_safe(input_available);
  if (!safe) {
    ROS_ERROR(
        "advertised /rover/base_is_stopped is headerless and cannot "
        "provide fresh base-stop evidence");
  }
  return safe;
}

bool TubeGraspMission::ValidateMeasuredTarget(
    const geometry_msgs::PointStamped& cap_top) const {
  const std::array<double, 3> point = {
      cap_top.point.x, cap_top.point.y, cap_top.point.z};
  const std::array<double, 3> workspace_min = {
      config_->workspace_min_xyz_m[0],
      config_->workspace_min_xyz_m[1],
      config_->workspace_min_xyz_m[2]};
  const std::array<double, 3> workspace_max = {
      config_->workspace_max_xyz_m[0],
      config_->workspace_max_xyz_m[1],
      config_->workspace_max_xyz_m[2]};
  return runtime_geometry_resolved_ &&
      target_cap_observation_is_valid(
      cap_top.header.frame_id == config_->planning_frame,
      cap_top.header.stamp.toSec(),
      ros::Time::now().toSec(),
      config_->maximum_frame_age_sec,
      point,
      workspace_min,
      workspace_max,
      runtime_cap_top_z_m_,
      config_->cap_top_height_band_m + config_->height_tolerance_m);
}

geometry_msgs::Pose TubeGraspMission::TcpPoseToLink6(
    const geometry_msgs::Pose& tcp_pose) const {
  tf2::Transform link6_to_tcp;
  tf2::fromMsg(RosTransform(config_->link6_to_grasp_tcp), link6_to_tcp);
  tf2::Transform planning_to_tcp;
  tf2::fromMsg(tcp_pose, planning_to_tcp);
  const tf2::Transform planning_to_link6 =
      planning_to_tcp * link6_to_tcp.inverse();

  geometry_msgs::Pose result;
  result.position.x = planning_to_link6.getOrigin().x();
  result.position.y = planning_to_link6.getOrigin().y();
  result.position.z = planning_to_link6.getOrigin().z();
  result.orientation = tf2::toMsg(planning_to_link6.getRotation());
  return result;
}

TubeMissionStatus TubeGraspMission::DetectObservation(
    const std::string& fixture_profile_id,
    const std::string& target_cap_color,
    std::uint8_t mode,
    const ros::Time& not_before,
    TubeObservation* observation) {
  if (observation == nullptr ||
      (mode != agx_pick_msg::DetectTubeCapRequest::TARGET_ONLY &&
       mode != agx_pick_msg::DetectTubeCapRequest::FULL_LAYOUT)) {
    return TubeMissionStatus::kInvalidRequest;
  }
  if (fixture_profile_id != config_->active_profile_id) {
    return TubeMissionStatus::kProfileMismatch;
  }
  if (target_cap_color != config_->target_cap_color ||
      target_cap_color != config_->profile_target_cap_color) {
    return TubeMissionStatus::kInvalidRequest;
  }

  agx_pick_msg::DetectTubeCapConfigured service;
  agx_xarm_pick::FillConfiguredTubePerceptionRequest(
      *resolved_config_, target_cap_color, &service.request);
  service.request.mode = mode;
  service.request.not_before = not_before;
  if (!ContinueRunning()) return TubeMissionStatus::kInterrupted;
  if (!detector_client_.call(service)) {
    return TubeMissionStatus::kDetectorUnavailable;
  }
  if (service.response.config_id != service.request.config_id ||
      service.response.fixture_instance_id != service.request.fixture_instance_id ||
      service.response.fixture_profile_id != service.request.fixture_profile_id ||
      service.response.projection_sha256 != service.request.projection_sha256)
    return TubeMissionStatus::kProfileMismatch;
  switch (service.response.status) {
    case agx_pick_msg::DetectTubeCapResponse::SUCCESS:
      break;
    case agx_pick_msg::DetectTubeCapResponse::INVALID_REQUEST:
      return TubeMissionStatus::kInvalidRequest;
    case agx_pick_msg::DetectTubeCapResponse::TARGET_ABSENT:
      return TubeMissionStatus::kTargetAbsent;
    case agx_pick_msg::DetectTubeCapResponse::AMBIGUOUS_TARGET:
      return TubeMissionStatus::kAmbiguousTarget;
    case agx_pick_msg::DetectTubeCapResponse::STALE_FRAME:
      return TubeMissionStatus::kStaleFrame;
    case agx_pick_msg::DetectTubeCapResponse::TF_UNAVAILABLE:
      return TubeMissionStatus::kTfUnavailable;
    case agx_pick_msg::DetectTubeCapResponse::OUT_OF_WORKSPACE:
      return TubeMissionStatus::kOutOfWorkspace;
    case agx_pick_msg::DetectTubeCapResponse::INCONSISTENT_TARGET:
      return TubeMissionStatus::kInconsistentTarget;
    case agx_pick_msg::DetectTubeCapResponse::LAYOUT_UNDERCONSTRAINED:
      return TubeMissionStatus::kLayoutUnderconstrained;
    case agx_pick_msg::DetectTubeCapResponse::PROFILE_MISMATCH:
      return TubeMissionStatus::kProfileMismatch;
    default:
      return TubeMissionStatus::kDetectorUnavailable;
  }

  const auto& response = service.response;
  const ros::Time now = ros::Time::now();
  if (response.cap_top.header.stamp.isZero() ||
      response.cap_top.header.stamp < not_before ||
      response.cap_top.header.stamp > now ||
      now - response.cap_top.header.stamp >
          ros::Duration(config_->maximum_frame_age_sec)) {
    return TubeMissionStatus::kStaleFrame;
  }
  if (response.cap_top.header.frame_id != config_->planning_frame) {
    return TubeMissionStatus::kTfUnavailable;
  }
  if (!ValidateMeasuredTarget(response.cap_top)) {
    return TubeMissionStatus::kOutOfWorkspace;
  }
  if (response.target_candidate_count != 1) {
    return TubeMissionStatus::kInconsistentTarget;
  }
  if (!std::isfinite(response.observed_cap_diameter_m) ||
      response.observed_cap_diameter_m <= 0.0 ||
      std::abs(
          response.observed_cap_diameter_m -
          config_->cap_diameter_m) > config_->diameter_tolerance_m) {
    return TubeMissionStatus::kInconsistentTarget;
  }
  if (!std::isfinite(response.target_u_px) ||
      !std::isfinite(response.target_v_px) ||
      !std::isfinite(response.principal_u_px) ||
      !std::isfinite(response.principal_v_px)) {
    return TubeMissionStatus::kInconsistentTarget;
  }
  const std::size_t point_count = response.observed_cap_tops.size();
  if (response.grid_u.size() != point_count ||
      response.grid_v.size() != point_count) {
    return TubeMissionStatus::kLayoutUnderconstrained;
  }
  if (mode == agx_pick_msg::DetectTubeCapRequest::TARGET_ONLY) {
    if (point_count != 0 ||
        response.lattice_inlier_count != 0 ||
        response.layout_candidate_count != 0 ||
        !std::isfinite(response.lattice_rms_error_m) ||
        !FiniteVector(response.grid_axis_u) ||
        !FiniteVector(response.grid_axis_v)) {
      return TubeMissionStatus::kLatticeRejected;
    }
  } else {
    const std::size_t maximum_slots = static_cast<std::size_t>(
        static_cast<std::int64_t>(config_->rows) *
        static_cast<std::int64_t>(config_->columns));
    if (point_count <
            static_cast<std::size_t>(config_->minimum_layout_inliers) ||
        point_count > maximum_slots ||
        response.lattice_inlier_count != point_count ||
        response.layout_candidate_count < point_count) {
      return TubeMissionStatus::kLayoutUnderconstrained;
    }
    std::vector<std::array<double, 3>> measured_points;
    measured_points.reserve(point_count);
    for (std::size_t index = 0; index < point_count; ++index) {
      geometry_msgs::PointStamped cap;
      cap.header = response.cap_top.header;
      cap.point = response.observed_cap_tops[index];
      if (!FinitePoint(cap.point) || !ValidateMeasuredTarget(cap)) {
        return TubeMissionStatus::kOutOfWorkspace;
      }
      measured_points.push_back(
          {cap.point.x, cap.point.y, cap.point.z});
    }
    const std::array<double, 3> measured_target = {
        response.cap_top.point.x,
        response.cap_top.point.y,
        response.cap_top.point.z};
    const std::array<double, 3> measured_axis_u = {
        response.grid_axis_u.x,
        response.grid_axis_u.y,
        response.grid_axis_u.z};
    const std::array<double, 3> measured_axis_v = {
        response.grid_axis_v.x,
        response.grid_axis_v.y,
        response.grid_axis_v.z};
    if (!measured_lattice_is_valid(
            measured_target,
            measured_points,
            response.grid_u,
            response.grid_v,
            measured_axis_u,
            measured_axis_v,
            config_->rows,
            config_->columns,
            config_->minimum_layout_inliers,
            config_->center_pitch_m,
            config_->pitch_tolerance_m,
            config_->maximum_lattice_rms_error_m,
            response.lattice_rms_error_m)) {
      return TubeMissionStatus::kLatticeRejected;
    }
  }

  TubeObservation validated;
  validated.cap_top = response.cap_top;
  validated.observed_cap_tops = response.observed_cap_tops;
  validated.grid_u = response.grid_u;
  validated.grid_v = response.grid_v;
  validated.grid_axis_u = response.grid_axis_u;
  validated.grid_axis_v = response.grid_axis_v;
  validated.target_u_px = response.target_u_px;
  validated.target_v_px = response.target_v_px;
  validated.principal_u_px = response.principal_u_px;
  validated.principal_v_px = response.principal_v_px;
  validated.lattice_rms_error_m = response.lattice_rms_error_m;
  *observation = std::move(validated);

  ROS_INFO(
      "tube detector: mode=%u target_candidates=%u layout_inliers=%u "
      "stamp=%.9f frame=%s xyz=(%.6f, %.6f, %.6f)",
      static_cast<unsigned int>(mode),
      response.target_candidate_count,
      response.lattice_inlier_count,
      observation->cap_top.header.stamp.toSec(),
      observation->cap_top.header.frame_id.c_str(),
      observation->cap_top.point.x,
      observation->cap_top.point.y,
      observation->cap_top.point.z);
  return TubeMissionStatus::kSuccess;
}


TubeMissionStatus TubeGraspMission::DetectLayout(
    const std::string& fixture_profile_id,
    const std::string& target_cap_color,
    const TubeObservation& centered_target,
    TubeObservation* observation,
    bool require_centered_reference,
    const TubeObservation* stationary_layout) {
  if (require_centered_reference &&
      !ObservationIsFresh(centered_target)) {
    return TubeMissionStatus::kStaleFrame;
  }
  ros::Time not_before;
  const TubeMissionStatus settle_status =
      SettleForObservation(&not_before);
  if (settle_status != TubeMissionStatus::kSuccess) {
    return settle_status;
  }
  if (stationary_layout != nullptr) {
    TubeObservation refreshed_target;
    const TubeMissionStatus target_status = DetectObservation(
        fixture_profile_id,
        target_cap_color,
        agx_pick_msg::DetectTubeCapRequest::TARGET_ONLY,
        not_before,
        &refreshed_target);
    if (target_status != TubeMissionStatus::kSuccess) {
      return target_status;
    }
    const std::array<double, 3> reference_target_point = {
        centered_target.cap_top.point.x,
        centered_target.cap_top.point.y,
        centered_target.cap_top.point.z};
    const std::array<double, 3> stationary_target_point = {
        stationary_layout->cap_top.point.x,
        stationary_layout->cap_top.point.y,
        stationary_layout->cap_top.point.z};
    const std::array<double, 3> refreshed_target_point = {
        refreshed_target.cap_top.point.x,
        refreshed_target.cap_top.point.y,
        refreshed_target.cap_top.point.z};
    if (!same_target_observation(
            reference_target_point,
            refreshed_target_point,
            config_->max_interframe_spread_m) ||
        !same_target_observation(
            stationary_target_point,
            refreshed_target_point,
            config_->max_interframe_spread_m)) {
      return TubeMissionStatus::kInconsistentTarget;
    }
    const double refreshed_pixel_error = std::hypot(
        refreshed_target.target_u_px - refreshed_target.principal_u_px,
        refreshed_target.target_v_px - refreshed_target.principal_v_px);
    if (!std::isfinite(refreshed_pixel_error) ||
        (require_centered_reference &&
         refreshed_pixel_error > config_->maximum_target_center_error_px)) {
      return TubeMissionStatus::kInconsistentTarget;
    }
    *observation = *stationary_layout;
    observation->cap_top = centered_target.cap_top;
    observation->cap_top.header = refreshed_target.cap_top.header;
    observation->target_u_px = refreshed_target.target_u_px;
    observation->target_v_px = refreshed_target.target_v_px;
    observation->principal_u_px = refreshed_target.principal_u_px;
    observation->principal_v_px = refreshed_target.principal_v_px;
    return TubeMissionStatus::kSuccess;
  }
  const TubeMissionStatus status = DetectObservation(
      fixture_profile_id,
      target_cap_color,
      agx_pick_msg::DetectTubeCapRequest::FULL_LAYOUT,
      not_before,
      observation);
  if (status != TubeMissionStatus::kSuccess) {
    return status;
  }
  const std::array<double, 3> centered_target_point = {
      centered_target.cap_top.point.x,
      centered_target.cap_top.point.y,
      centered_target.cap_top.point.z};
  const std::array<double, 3> layout_target_point = {
      observation->cap_top.point.x,
      observation->cap_top.point.y,
      observation->cap_top.point.z};
  if (!same_target_observation(
          centered_target_point,
          layout_target_point,
          config_->max_interframe_spread_m)) {
    return TubeMissionStatus::kInconsistentTarget;
  }
  const double pixel_error = std::hypot(
      observation->target_u_px - observation->principal_u_px,
      observation->target_v_px - observation->principal_v_px);
  return std::isfinite(pixel_error) &&
          (!require_centered_reference ||
           pixel_error <= config_->maximum_target_center_error_px)
      ? TubeMissionStatus::kSuccess
      : TubeMissionStatus::kInconsistentTarget;
}

TubeMissionStatus TubeGraspMission::AlignTcpOverTarget(
    const TubeObservation& observation) {
  geometry_msgs::Point pregrasp_tcp = observation.cap_top.point;
  pregrasp_tcp.z =
      observation.cap_top.point.z -
      config_->grasp_tcp_offset_below_cap_top_m +
      config_->pregrasp_clearance_m;
  if (!FinitePoint(pregrasp_tcp) ||
      pregrasp_tcp.x < config_->workspace_min_xyz_m[0] ||
      pregrasp_tcp.x > config_->workspace_max_xyz_m[0] ||
      pregrasp_tcp.y < config_->workspace_min_xyz_m[1] ||
      pregrasp_tcp.y > config_->workspace_max_xyz_m[1] ||
      pregrasp_tcp.z < config_->workspace_min_xyz_m[2] ||
      pregrasp_tcp.z > config_->workspace_max_xyz_m[2]) {
    return TubeMissionStatus::kPregraspFailed;
  }
  const double desired_clearance_m =
      pregrasp_tcp.z - runtime_cap_top_z_m_;
  if (!std::isfinite(desired_clearance_m) ||
      desired_clearance_m < config_->view_region.clearance_min_m ||
      desired_clearance_m > config_->view_region.clearance_max_m) {
    return TubeMissionStatus::kPregraspFailed;
  }
  agx_xarm_pick::TopDownViewRegion pregrasp_region =
      config_->view_region;
  const double position_tolerance_m =
      execution_position_tolerance_m(
          config_->view_region.y_tolerance_m);
  pregrasp_region.x_min_m = std::max(
      config_->workspace_min_xyz_m[0],
      pregrasp_tcp.x - position_tolerance_m);
  pregrasp_region.x_max_m = std::min(
      config_->workspace_max_xyz_m[0],
      pregrasp_tcp.x + position_tolerance_m);
  pregrasp_region.preferred_x_m = pregrasp_tcp.x;
  pregrasp_region.preferred_clearance_m = desired_clearance_m;
  pregrasp_region.maximum_candidate_attempts = 1;
  geometry_msgs::Pose measured_pregrasp_tcp;
  return MoveTopDown(
      ArmMotion::PregraspAdjustment,
      pregrasp_region,
      pregrasp_tcp.y,
      config_->view_region.clearance_min_m,
      TubeMissionStatus::kPregraspFailed,
      TubeMissionStatus::kPregraspFailed,
      &measured_pregrasp_tcp,
      &observation);
}

TubeMissionStatus TubeGraspMission::SettleForObservation(
    ros::Time* not_before) const {
  if (not_before == nullptr) {
    return TubeMissionStatus::kInvalidRequest;
  }
  const auto settle_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
              config_->search_settle_duration_sec));
  while (ContinueRunning() &&
         std::chrono::steady_clock::now() < settle_deadline) {
    std::this_thread::sleep_for(kPollPeriod);
  }
  if (!ContinueRunning()) {
    return TubeMissionStatus::kInterrupted;
  }
  *not_before = ros::Time::now();
  return TubeMissionStatus::kSuccess;
}

TubeMissionStatus TubeGraspMission::RotateTcpAtPregrasp(
    const geometry_msgs::Pose& validated_pregrasp,
    double selected_yaw,
    const TubeObservation& required_fresh_observation) {
  if (!FinitePoint(validated_pregrasp.position) ||
      !std::isfinite(selected_yaw)) {
    return TubeMissionStatus::kPregraspFailed;
  }
  if (!ObservationIsFresh(required_fresh_observation)) {
    return TubeMissionStatus::kStaleFrame;
  }
  const auto planning_deadline =
      PlanningDeadline(config_->planning_timeout_sec);
  geometry_msgs::Pose measured_start_tcp;
  moveit::core::RobotStatePtr planning_state;
  if (!CurrentTcpPose(
          std::chrono::steady_clock::time_point{},
          planning_deadline,
          &measured_start_tcp,
          &planning_state)) {
    return TubeMissionStatus::kPregraspFailed;
  }
  const std::array<double, 3> anchor = {
      validated_pregrasp.position.x,
      validated_pregrasp.position.y,
      validated_pregrasp.position.z};
  const std::vector<std::array<double, 3>> measured_start_positions = {{
      measured_start_tcp.position.x,
      measured_start_tcp.position.y,
      measured_start_tcp.position.z,
  }};
  const std::vector<std::array<double, 4>> measured_start_orientations = {{
      measured_start_tcp.orientation.x,
      measured_start_tcp.orientation.y,
      measured_start_tcp.orientation.z,
      measured_start_tcp.orientation.w,
  }};
  if (!in_place_yaw_trajectory_is_safe(
          anchor,
          measured_start_positions,
          measured_start_orientations,
          kMaximumInPlaceYawDriftM,
          config_->downward_axis_tolerance_deg) ||
      !CameraAxisIsDownwardAtTcpPose(measured_start_tcp)) {
    return TubeMissionStatus::kPregraspFailed;
  }

  tf2::Quaternion selected_top_down;
  selected_top_down.setRPY(kPi, 0.0, selected_yaw);
  const std::array<double, 4> selected_orientation = {
      selected_top_down.x(),
      selected_top_down.y(),
      selected_top_down.z(),
      selected_top_down.w()};
  const std::array<double, 4> measured_orientation = {
      measured_start_tcp.orientation.x,
      measured_start_tcp.orientation.y,
      measured_start_tcp.orientation.z,
      measured_start_tcp.orientation.w};
  tf2::Quaternion measured_quaternion;
  tf2::fromMsg(measured_start_tcp.orientation, measured_quaternion);
  double measured_roll = 0.0;
  double measured_pitch = 0.0;
  double measured_yaw = 0.0;
  tf2::Matrix3x3(measured_quaternion).getRPY(
      measured_roll, measured_pitch, measured_yaw);
  ROS_INFO(
      "tube pregrasp yaw planning: current=%.6f selected=%.6f "
      "anchor=(%.6f, %.6f, %.6f)",
      measured_yaw,
      selected_yaw,
      anchor[0],
      anchor[1],
      anchor[2]);
  if (quaternion_orientation_within_tolerance(
          selected_orientation,
          measured_orientation,
          config_->downward_axis_tolerance_deg)) {
    return ObservationIsFresh(required_fresh_observation)
        ? TubeMissionStatus::kSuccess
        : TubeMissionStatus::kStaleFrame;
  }

  geometry_msgs::Pose target_tcp = measured_start_tcp;
  target_tcp.orientation = tf2::toMsg(selected_top_down);
  const geometry_msgs::Pose target_link6 =
      TcpPoseToLink6(target_tcp);
  move_group_->setStartState(*planning_state);
  moveit_msgs::RobotTrajectory yaw_trajectory;
  const std::vector<geometry_msgs::Pose> waypoints = {target_link6};
  moveit_msgs::MoveItErrorCodes yaw_error;
  const double fraction = move_group_->computeCartesianPath(
      waypoints,
      config_->cartesian_step_m,
      yaw_trajectory,
      true,
      &yaw_error);
  if (fraction < 1.0 - 1e-6) {
    ROS_ERROR(
        "tube pregrasp yaw Cartesian path fraction %.6f was incomplete "
        "for current=%.6f selected=%.6f",
        fraction,
        measured_yaw,
        selected_yaw);
    return TubeMissionStatus::kPregraspFailed;
  }

  robot_trajectory::RobotTrajectory checked_yaw(
      move_group_->getRobotModel(), "xarm6");
  checked_yaw.setRobotTrajectoryMsg(*planning_state, yaw_trajectory);
  std::vector<std::array<double, 3>> sampled_positions;
  std::vector<std::array<double, 4>> sampled_orientations;
  sampled_positions.reserve(checked_yaw.getWayPointCount());
  sampled_orientations.reserve(checked_yaw.getWayPointCount());
  for (std::size_t index = 0;
       index < checked_yaw.getWayPointCount();
       ++index) {
    geometry_msgs::Pose sampled_tcp;
    if (!TcpPoseFromState(
            checked_yaw.getWayPoint(index), &sampled_tcp) ||
        !CameraAxisIsDownwardAtTcpPose(sampled_tcp)) {
      return TubeMissionStatus::kPregraspFailed;
    }
    sampled_positions.push_back(
        {
            sampled_tcp.position.x,
            sampled_tcp.position.y,
            sampled_tcp.position.z,
        });
    sampled_orientations.push_back(
        {
            sampled_tcp.orientation.x,
            sampled_tcp.orientation.y,
            sampled_tcp.orientation.z,
            sampled_tcp.orientation.w,
        });
  }
  if (!in_place_yaw_trajectory_is_safe(
          anchor,
          sampled_positions,
          sampled_orientations,
          kMaximumInPlaceYawDriftM,
          config_->downward_axis_tolerance_deg) ||
      sampled_orientations.empty() ||
      !quaternion_orientation_within_tolerance(
          selected_orientation,
          sampled_orientations.back(),
          config_->downward_axis_tolerance_deg)) {
    ROS_ERROR(
        "tube pregrasp yaw planned trajectory failed invariant checks; "
        "waypoints=%zu",
        sampled_orientations.size());
    return TubeMissionStatus::kPregraspFailed;
  }

  const auto scales = ArmScales(ArmMotion::PregraspAdjustment);
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(
          checked_yaw,
          scales.velocity,
          scales.acceleration)) {
    ROS_ERROR("tube pregrasp yaw time parameterization failed");
    return TubeMissionStatus::kPregraspFailed;
  }
  checked_yaw.getRobotTrajectoryMsg(yaw_trajectory);
  if (!ContinueRunning()) {
    return TubeMissionStatus::kInterrupted;
  }
  if (move_group_->execute(yaw_trajectory) !=
      moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    ROS_ERROR("tube pregrasp yaw execution failed");
    return TubeMissionStatus::kPregraspFailed;
  }

  const auto execution_complete = std::chrono::steady_clock::now();
  geometry_msgs::Pose measured_endpoint_tcp;
  if (!CurrentTcpPose(
          execution_complete + std::chrono::steady_clock::duration(1),
          PlanningDeadline(config_->planning_timeout_sec),
          &measured_endpoint_tcp)) {
    return TubeMissionStatus::kPregraspFailed;
  }
  const std::vector<std::array<double, 4>> endpoint_orientation = {{
      measured_endpoint_tcp.orientation.x,
      measured_endpoint_tcp.orientation.y,
      measured_endpoint_tcp.orientation.z,
      measured_endpoint_tcp.orientation.w,
  }};
  const bool yaw_endpoint_xy_safe = vertical_xy_path_within_tolerance(
      {anchor[0], anchor[1]},
      {{
          measured_endpoint_tcp.position.x,
          measured_endpoint_tcp.position.y,
      }},
      execution_position_tolerance_m(config_->cartesian_step_m));
  const bool yaw_endpoint_has_clearance =
      agx_xarm_pick::trajectory_sample_has_minimum_clearance(
          measured_endpoint_tcp.position.z,
          runtime_cap_top_z_m_,
          config_->view_region.clearance_min_m);
  if (!yaw_endpoint_xy_safe ||
      !yaw_endpoint_has_clearance ||
      !quaternion_orientation_within_tolerance(
          selected_orientation,
          endpoint_orientation.front(),
          config_->downward_axis_tolerance_deg) ||
      !CameraAxisIsDownwardAtTcpPose(measured_endpoint_tcp)) {
    ROS_ERROR(
        "tube pregrasp yaw measured endpoint failed invariant checks: "
        "measured=(%.6f, %.6f, %.6f) xy_safe=%d clearance_safe=%d",
        measured_endpoint_tcp.position.x,
        measured_endpoint_tcp.position.y,
        measured_endpoint_tcp.position.z,
        yaw_endpoint_xy_safe,
        yaw_endpoint_has_clearance);
    return TubeMissionStatus::kPregraspFailed;
  }
  return TubeMissionStatus::kSuccess;
}

TubeMissionStatus TubeGraspMission::CorrectTcpPlanar(
    const TubeObservation& required_fresh_observation,
    double correction_x_m,
    double correction_y_m,
    geometry_msgs::Pose* measured_tcp) {
  if (measured_tcp == nullptr ||
      !std::isfinite(correction_x_m) ||
      !std::isfinite(correction_y_m)) {
    return TubeMissionStatus::kPregraspFailed;
  }

  auto motion_config = BuildAlignmentMotionConfig(ArmMotion::PregraspAdjustment);
  const double execution_tolerance_m = execution_position_tolerance_m(
      config_->view_region.y_tolerance_m);
  motion_config.maximum_z_error_m = execution_tolerance_m;
  motion_config.endpoint_position_tolerance_m = execution_tolerance_m;
  const auto state_reader = [this](
      std::chrono::steady_clock::time_point minimum_receipt_time,
      std::chrono::steady_clock::time_point deadline,
      moveit::core::RobotStatePtr* state,
      geometry_msgs::Pose* grasp_tcp_pose) {
    return ReadAlignmentState(
        minimum_receipt_time, deadline, state, grasp_tcp_pose);
  };
  const auto target_is_fresh = [this](
      const agx_xarm_pick::VisualTargetObservation& observation) {
    return VisualTargetIsFresh(observation);
  };
  agx_xarm_pick::MoveItVisualAlignmentMotion motion(
      move_group_.get(),
      motion_config,
      state_reader,
      target_is_fresh);

  agx_xarm_pick::VisualTargetObservation target;
  target.stamp_ns = required_fresh_observation.cap_top.header.stamp.toNSec();
  target.frame_id = config_->color_frame;
  target.u_px = required_fresh_observation.target_u_px;
  target.v_px = required_fresh_observation.target_v_px;
  target.reference_u_px = required_fresh_observation.principal_u_px;
  target.reference_v_px = required_fresh_observation.principal_v_px;
  target.image_width = static_cast<std::uint32_t>(std::ceil(
      std::max(target.u_px, target.reference_u_px) + 1.0));
  target.image_height = static_cast<std::uint32_t>(std::ceil(
      std::max(target.v_px, target.reference_v_px) + 1.0));

  const auto result = motion.AttemptCorrection(
      target, correction_x_m, correction_y_m);
  if (result.status != agx_xarm_pick::AlignmentMotionStatus::kSuccess) {
    ROS_ERROR(
        "tube planar TCP correction failed: %s",
        result.detail.c_str());
    return result.status ==
            agx_xarm_pick::AlignmentMotionStatus::kStaleTargetBeforeExecution
        ? TubeMissionStatus::kStaleFrame
        : TubeMissionStatus::kPregraspFailed;
  }
  measured_tcp->position.x = result.after.tcp_x_m;
  measured_tcp->position.y = result.after.tcp_y_m;
  measured_tcp->position.z = result.after.tcp_z_m;
  measured_tcp->orientation.x = result.after.qx;
  measured_tcp->orientation.y = result.after.qy;
  measured_tcp->orientation.z = result.after.qz;
  measured_tcp->orientation.w = result.after.qw;
  if (!agx_xarm_pick::trajectory_sample_has_minimum_clearance(
          measured_tcp->position.z,
          runtime_cap_top_z_m_,
          config_->view_region.clearance_min_m) ||
      !CameraAxisIsDownwardAtTcpPose(*measured_tcp)) {
    return TubeMissionStatus::kPregraspFailed;
  }
  return TubeMissionStatus::kSuccess;
}

TubeMissionStatus TubeGraspMission::CommandAndWaitForAperture(
    double insertion_command) {
  if (payload_ != PayloadKnowledge::kKnownEmpty ||
      !std::isfinite(insertion_command) ||
      insertion_command < config_->close_command ||
      insertion_command > config_->fully_open_command) {
    return TubeMissionStatus::kInsertionApertureFailed;
  }
  payload_ = PayloadKnowledge::kInsertionAperture;
  if (!CommandGripper(insertion_command, GripperMotion::Preclose)) {
    return last_gripper_wait_status_;
  }
  if (!WaitForGripperPosition(insertion_command)) {
    return last_gripper_wait_status_;
  }
  return TubeMissionStatus::kSuccess;
}

TubeMissionStatus TubeGraspMission::MoveVertical(
    const geometry_msgs::Pose& start,
    double target_z,
    TubeMissionStatus failure,
    ArmMotion motion,
    const TubeObservation* required_fresh_observation,
    const double* required_aperture_command) {
  if (!ContinueRunning()) {
    return TubeMissionStatus::kInterrupted;
  }
  if (!FinitePoint(start.position) ||
      !std::isfinite(target_z) ||
      target_z < config_->workspace_min_xyz_m[2] ||
      target_z > config_->workspace_max_xyz_m[2] ||
      !tcp_approach_axis_is_downward(
          start.orientation.x,
          start.orientation.y,
          start.orientation.z,
          start.orientation.w,
          config_->downward_axis_tolerance_deg)) {
    return failure;
  }

  const auto planning_deadline =
      PlanningDeadline(config_->planning_timeout_sec);
  geometry_msgs::Pose measured_start_tcp;
  moveit::core::RobotStatePtr planning_state;
  const bool start_state_available = CurrentTcpPose(
          std::chrono::steady_clock::time_point{},
          planning_deadline,
          &measured_start_tcp,
          &planning_state);
  const bool start_xy_valid = start_state_available &&
      vertical_xy_path_within_tolerance(
          {start.position.x, start.position.y},
          {{
              measured_start_tcp.position.x,
              measured_start_tcp.position.y,
          }},
          kMaximumVerticalXyDriftM);
  const bool start_orientation_valid = start_state_available &&
      quaternion_orientation_within_tolerance(
          {
              start.orientation.x,
              start.orientation.y,
              start.orientation.z,
              start.orientation.w,
          },
          {
              measured_start_tcp.orientation.x,
              measured_start_tcp.orientation.y,
              measured_start_tcp.orientation.z,
              measured_start_tcp.orientation.w,
          },
          config_->downward_axis_tolerance_deg);
  const bool start_tcp_down = start_state_available &&
      tcp_approach_axis_is_downward(
          measured_start_tcp.orientation.x,
          measured_start_tcp.orientation.y,
          measured_start_tcp.orientation.z,
          measured_start_tcp.orientation.w,
          config_->downward_axis_tolerance_deg);
  const bool start_camera_down = start_state_available &&
      CameraAxisIsDownwardAtTcpPose(measured_start_tcp);
  if (!start_state_available || !start_xy_valid ||
      !start_orientation_valid || !start_tcp_down || !start_camera_down) {
    ROS_ERROR(
        "tube vertical start invariant failed: state=%d xy=%d "
        "orientation=%d tcp_down=%d camera_down=%d "
        "expected=(%.6f, %.6f, %.6f) measured=(%.6f, %.6f, %.6f)",
        start_state_available,
        start_xy_valid,
        start_orientation_valid,
        start_tcp_down,
        start_camera_down,
        start.position.x,
        start.position.y,
        start.position.z,
        measured_start_tcp.position.x,
        measured_start_tcp.position.y,
        measured_start_tcp.position.z);
    return failure;
  }

  const double endpoint_tolerance_m =
      execution_position_tolerance_m(config_->cartesian_step_m);
  if (std::abs(measured_start_tcp.position.z - target_z) <=
      endpoint_tolerance_m) {
    if (required_fresh_observation != nullptr &&
        !ObservationIsFresh(*required_fresh_observation)) {
      return TubeMissionStatus::kStaleFrame;
    }
    return TubeMissionStatus::kSuccess;
  }

  geometry_msgs::Pose target_tcp = measured_start_tcp;
  target_tcp.position.z = target_z;
  tf2::Transform planning_to_tcp;
  tf2::fromMsg(target_tcp, planning_to_tcp);
  tf2::Transform link6_to_tcp;
  tf2::fromMsg(RosTransform(config_->link6_to_grasp_tcp), link6_to_tcp);
  const tf2::Transform planning_to_link6 =
      planning_to_tcp * link6_to_tcp.inverse();
  geometry_msgs::Pose target_link6;
  target_link6.position.x = planning_to_link6.getOrigin().x();
  target_link6.position.y = planning_to_link6.getOrigin().y();
  target_link6.position.z = planning_to_link6.getOrigin().z();
  target_link6.orientation =
      tf2::toMsg(planning_to_link6.getRotation());

  move_group_->setStartState(*planning_state);
  moveit_msgs::RobotTrajectory trajectory;
  const std::vector<geometry_msgs::Pose> waypoints = {target_link6};
  moveit_msgs::MoveItErrorCodes cartesian_error;
  const double fraction = move_group_->computeCartesianPath(
      waypoints,
      config_->cartesian_step_m,
      trajectory,
      true,
      &cartesian_error);
  if (fraction < 1.0 - 1e-6) {
    ROS_ERROR(
        "tube vertical Cartesian path fraction %.6f was incomplete",
        fraction);
    return failure;
  }

  robot_trajectory::RobotTrajectory checked_trajectory(
      move_group_->getRobotModel(), "xarm6");
  checked_trajectory.setRobotTrajectoryMsg(*planning_state, trajectory);
  std::vector<std::array<double, 2>> sampled_xy;
  sampled_xy.reserve(checked_trajectory.getWayPointCount());
  for (std::size_t index = 0;
       index < checked_trajectory.getWayPointCount();
       ++index) {
    geometry_msgs::Pose sampled_tcp;
    const bool sampled_pose_valid = TcpPoseFromState(
        checked_trajectory.getWayPoint(index), &sampled_tcp);
    const bool sampled_orientation_valid = sampled_pose_valid &&
        quaternion_orientation_within_tolerance(
            {
                measured_start_tcp.orientation.x,
                measured_start_tcp.orientation.y,
                measured_start_tcp.orientation.z,
                measured_start_tcp.orientation.w,
            },
            {
                sampled_tcp.orientation.x,
                sampled_tcp.orientation.y,
                sampled_tcp.orientation.z,
                sampled_tcp.orientation.w,
            },
            config_->downward_axis_tolerance_deg);
    const bool sampled_tcp_down = sampled_pose_valid &&
        tcp_approach_axis_is_downward(
            sampled_tcp.orientation.x,
            sampled_tcp.orientation.y,
            sampled_tcp.orientation.z,
            sampled_tcp.orientation.w,
            config_->downward_axis_tolerance_deg);
    const bool sampled_camera_down = sampled_pose_valid &&
        CameraAxisIsDownwardAtTcpPose(sampled_tcp);
    if (!sampled_pose_valid || !sampled_orientation_valid ||
        !sampled_tcp_down || !sampled_camera_down) {
      ROS_ERROR(
          "tube vertical sampled waypoint %zu failed: pose=%d "
          "orientation=%d tcp_down=%d camera_down=%d",
          index,
          sampled_pose_valid,
          sampled_orientation_valid,
          sampled_tcp_down,
          sampled_camera_down);
      return failure;
    }
    sampled_xy.push_back(
        {sampled_tcp.position.x, sampled_tcp.position.y});
  }
  if (!vertical_xy_path_within_tolerance(
          {
              measured_start_tcp.position.x,
              measured_start_tcp.position.y,
          },
          sampled_xy,
          kMaximumVerticalXyDriftM)) {
    ROS_ERROR(
        "tube vertical Cartesian path exceeded %.6f m XY drift",
        kMaximumVerticalXyDriftM);
    return failure;
  }

  const auto scales = ArmScales(motion);
  trajectory_processing::IterativeParabolicTimeParameterization timing;
  if (!timing.computeTimeStamps(
          checked_trajectory,
          scales.velocity,
          scales.acceleration)) {
    return failure;
  }
  checked_trajectory.getRobotTrajectoryMsg(trajectory);
  if (!ContinueRunning()) {
    return TubeMissionStatus::kInterrupted;
  }
  if (required_fresh_observation != nullptr &&
      !ObservationIsFresh(*required_fresh_observation)) {
    return TubeMissionStatus::kStaleFrame;
  }
  if (required_aperture_command != nullptr &&
      !ApertureEvidenceIsCurrent(*required_aperture_command)) {
    return TubeMissionStatus::kInsertionApertureFailed;
  }
  if (move_group_->execute(trajectory) !=
      moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    return failure;
  }

  const auto execution_complete = std::chrono::steady_clock::now();
  geometry_msgs::Pose measured_endpoint_tcp;
  const geometry_msgs::Point requested_endpoint = target_tcp.position;
  const bool endpoint_state_available = CurrentTcpPose(
          execution_complete + std::chrono::steady_clock::duration(1),
          PlanningDeadline(config_->planning_timeout_sec),
          &measured_endpoint_tcp);
  const bool endpoint_tcp_valid = endpoint_state_available &&
      ValidateExecutedTcp(requested_endpoint, measured_endpoint_tcp);
  const bool endpoint_pose_valid = endpoint_state_available &&
      tcp_pose_within_tolerance(
          requested_endpoint.x,
          requested_endpoint.y,
          requested_endpoint.z,
          measured_endpoint_tcp.position.x,
          measured_endpoint_tcp.position.y,
          measured_endpoint_tcp.position.z,
          endpoint_tolerance_m);
  const bool endpoint_xy_valid = endpoint_state_available &&
      vertical_xy_path_within_tolerance(
          {
              measured_start_tcp.position.x,
              measured_start_tcp.position.y,
          },
          {{
              measured_endpoint_tcp.position.x,
              measured_endpoint_tcp.position.y,
          }},
          kMaximumVerticalXyDriftM);
  const bool endpoint_orientation_valid = endpoint_state_available &&
      quaternion_orientation_within_tolerance(
          {
              measured_start_tcp.orientation.x,
              measured_start_tcp.orientation.y,
              measured_start_tcp.orientation.z,
              measured_start_tcp.orientation.w,
          },
          {
              measured_endpoint_tcp.orientation.x,
              measured_endpoint_tcp.orientation.y,
              measured_endpoint_tcp.orientation.z,
              measured_endpoint_tcp.orientation.w,
          },
          config_->downward_axis_tolerance_deg);
  const bool endpoint_tcp_down = endpoint_state_available &&
      tcp_approach_axis_is_downward(
          measured_endpoint_tcp.orientation.x,
          measured_endpoint_tcp.orientation.y,
          measured_endpoint_tcp.orientation.z,
          measured_endpoint_tcp.orientation.w,
          config_->downward_axis_tolerance_deg);
  const bool endpoint_camera_down = endpoint_state_available &&
      CameraAxisIsDownwardAtTcpPose(measured_endpoint_tcp);
  if (!endpoint_state_available || !endpoint_tcp_valid ||
      !endpoint_pose_valid || !endpoint_xy_valid ||
      !endpoint_orientation_valid || !endpoint_tcp_down ||
      !endpoint_camera_down) {
    ROS_ERROR(
        "tube vertical measured endpoint failed: state=%d tcp=%d pose=%d "
        "xy=%d orientation=%d tcp_down=%d camera_down=%d "
        "requested=(%.6f, %.6f, %.6f) measured=(%.6f, %.6f, %.6f)",
        endpoint_state_available,
        endpoint_tcp_valid,
        endpoint_pose_valid,
        endpoint_xy_valid,
        endpoint_orientation_valid,
        endpoint_tcp_down,
        endpoint_camera_down,
        requested_endpoint.x,
        requested_endpoint.y,
        requested_endpoint.z,
        measured_endpoint_tcp.position.x,
        measured_endpoint_tcp.position.y,
        measured_endpoint_tcp.position.z);
    return failure;
  }
  return TubeMissionStatus::kSuccess;
}

TubeMissionStatus TubeGraspMission::RecoverPreCloseFailure(
    TubeMissionStatus primary_failure,
    const geometry_msgs::Pose& validated_pregrasp,
    double pregrasp_z) {
  if (!may_attempt_preclose_retreat(payload_)) {
    return Fail(primary_failure);
  }
  const TubeMissionStatus retreat_status = MoveVertical(
      validated_pregrasp,
      pregrasp_z,
      TubeMissionStatus::kPregraspFailed,
      ArmMotion::Recovery);
  if (retreat_status != TubeMissionStatus::kSuccess) {
    ROS_ERROR(
        "primary pre-close failure=%s; retreat failed=%s; "
        "leaving insertion aperture unchanged",
        tube_mission_status_name(primary_failure),
        tube_mission_status_name(retreat_status));
    return Fail(primary_failure);
  }
  if (!may_command_open_during_cleanup(payload_) ||
      !CommandGripper(config_->fully_open_command, GripperMotion::Recovery) ||
      !WaitForGripperPosition(config_->fully_open_command)) {
    ROS_ERROR(
        "primary pre-close failure=%s; retreat succeeded but cleanup "
        "open verification failed; leaving payload knowledge at "
        "insertion aperture",
        tube_mission_status_name(primary_failure));
    return Fail(primary_failure);
  }
  payload_ = payload_after_verified_preclose_cleanup(
      payload_, true, true);
  ROS_WARN(
      "primary pre-close failure=%s; retreated vertically and "
      "verified fully open",
      tube_mission_status_name(primary_failure));
  return Fail(primary_failure);
}

bool TubeGraspMission::CommandGripper(double position, GripperMotion motion) {
  if (!ContinueRunning() || position < 0.0 || position > 1000.0) {
    last_gripper_wait_status_ = ContinueRunning()
        ? TubeMissionStatus::kPayloadStateUnsafe
        : TubeMissionStatus::kInterrupted;
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(config_->gripper_timeout_sec));
  while (ContinueRunning() && gripper_publisher_.getNumSubscribers() == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(kPollPeriod);
  }
  if (!ContinueRunning() || gripper_publisher_.getNumSubscribers() == 0) {
    last_gripper_wait_status_ = ContinueRunning()
        ? TubeMissionStatus::kGripperTimeout
        : TubeMissionStatus::kInterrupted;
    return false;
  }
  dh_gripper_msgs::GripperCtrl command;
  command.initialize = false;
  command.position = position;
  command.force = config_->gripper_force;
  command.speed = config_->gripper_speeds.at(static_cast<std::size_t>(motion));
  gripper_command_wall_time_ = std::chrono::steady_clock::now();
  if (!ContinueRunning()) return false;
  gripper_publisher_.publish(command);
  return true;
}

TubeMissionStatus TubeGraspMission::ValidateAndPrepare(
    const std::string& fixture_profile_id,
    const std::string& target_cap_color) {
  if (fixture_profile_id != config_->active_profile_id) {
    return TubeMissionStatus::kProfileMismatch;
  }
  if (target_cap_color != config_->target_cap_color ||
      target_cap_color != config_->profile_target_cap_color) {
    return TubeMissionStatus::kInvalidRequest;
  }

  const auto setup_deadline =
      PlanningDeadline(config_->planning_timeout_sec);
  bool setup_ready = false;
  moveit::core::RobotStatePtr setup_state;
  while (ContinueRunning() && std::chrono::steady_clock::now() < setup_deadline) {
    if (detector_client_.exists() &&
        lift_client_.exists() &&
        gripper_publisher_.getNumSubscribers() > 0 &&
        gripper_subscriber_.getNumPublishers() > 0 &&
        joint_state_subscriber_.getNumPublishers() > 0 &&
        lift_state_subscriber_.getNumPublishers() > 0 &&
        WaitForSynchronizedRobotState(
            std::chrono::steady_clock::time_point{},
            setup_deadline,
            &setup_state)) {
      setup_ready = true;
      break;
    }
    std::this_thread::sleep_for(kPollPeriod);
  }
  if (!ContinueRunning()) {
    return TubeMissionStatus::kInterrupted;
  }
  if (!lift_client_.exists() ||
      lift_state_subscriber_.getNumPublishers() == 0) {
    return TubeMissionStatus::kLiftInterfaceUnavailable;
  }
  if (!detector_client_.exists() ||
      gripper_publisher_.getNumSubscribers() == 0 ||
      gripper_subscriber_.getNumPublishers() == 0 ||
      joint_state_subscriber_.getNumPublishers() == 0 ||
      !setup_ready ||
      !setup_state) {
    return TubeMissionStatus::kSetupUnavailable;
  }
  geometry_msgs::Pose initial_tcp;
  const double initial_joint1 =
      setup_state->getVariablePosition("joint1");
  if (!TcpPoseFromState(*setup_state, &initial_tcp) ||
      !initial_compact_preparation_state_is_safe(
          initial_tcp.position.x,
          initial_tcp.position.y,
          initial_tcp.position.z,
          initial_joint1,
          config_->initial_tcp_x_bounds_m[0],
          config_->initial_tcp_x_bounds_m[1],
          config_->search_centerline_y_m,
          config_->initial_centerline_tolerance_m,
          config_->initial_tcp_z_bounds_m[0],
          config_->initial_tcp_z_bounds_m[1],
          config_->phase_one_expected_tcp.z,
          config_->centerline_joint_position_rad,
          config_->centerline_joint_tolerance_rad)) {
    ROS_ERROR(
        "tube lift refused: initial TCP=(%.6f, %.6f, %.6f), "
        "joint1=%.6f is outside the configured centered home envelope",
        initial_tcp.position.x,
        initial_tcp.position.y,
        initial_tcp.position.z,
        initial_joint1);
    return TubeMissionStatus::kUnsafeInitialCenterline;
  }
  if (!WaitForKnownOpenState()) {
    return last_gripper_wait_status_;
  }
  if (!EmitGripperEffect(TubeEffectKind::KnownEmpty)) return last_gripper_wait_status_;
  return TubeMissionStatus::kSuccess;
}


agx_xarm_pick::MoveItVisualAlignmentMotionConfig
TubeGraspMission::BuildAlignmentMotionConfig(ArmMotion motion) const {
  const auto scales = ArmScales(motion);
  agx_xarm_pick::MoveItVisualAlignmentMotionConfig motion_config;
  motion_config.planning_frame = config_->planning_frame;
  motion_config.link6_to_grasp_tcp = RosTransform(config_->link6_to_grasp_tcp);
  motion_config.link6_to_camera = RosTransform(config_->link6_to_camera);
  std::copy_n(
      config_->workspace_min_xyz_m.begin(), 3,
      motion_config.workspace_min_xyz_m.begin());
  std::copy_n(
      config_->workspace_max_xyz_m.begin(), 3,
      motion_config.workspace_max_xyz_m.begin());
  motion_config.maximum_step_m = config_->visual_alignment.maximum_step_m;
  motion_config.maximum_downward_error_deg =
      config_->downward_axis_tolerance_deg;
  motion_config.maximum_z_error_m =
      execution_position_tolerance_m(config_->cartesian_step_m);
  motion_config.vertical_relief_step_m = runtime_geometry_resolved_
      ? config_->view_region.sampling_step_m
      : 0.0;
  motion_config.minimum_tcp_z_m = runtime_geometry_resolved_
      ? runtime_cap_top_z_m_ + config_->pregrasp_clearance_m
      : config_->workspace_min_xyz_m[2];
  motion_config.maximum_orientation_error_deg =
      config_->downward_axis_tolerance_deg;
  motion_config.endpoint_position_tolerance_m =
      execution_position_tolerance_m(config_->cartesian_step_m);
  motion_config.cartesian_step_m = config_->cartesian_step_m;
  motion_config.velocity_scaling = scales.velocity;
  motion_config.acceleration_scaling = scales.acceleration;
  motion_config.state_max_age_sec = config_->robot_state_max_age_sec;
  motion_config.state_wait_timeout_sec = config_->planning_timeout_sec;
  return motion_config;
}

bool TubeGraspMission::ReadAlignmentState(
    std::chrono::steady_clock::time_point minimum_receipt_time,
    std::chrono::steady_clock::time_point deadline,
    moveit::core::RobotStatePtr* state,
    geometry_msgs::Pose* grasp_tcp_pose) const {
  return state != nullptr && grasp_tcp_pose != nullptr &&
      WaitForSynchronizedRobotState(
          minimum_receipt_time, deadline, state) &&
      TcpPoseFromState(**state, grasp_tcp_pose);
}

bool TubeGraspMission::VisualTargetIsFresh(
    const agx_xarm_pick::VisualTargetObservation& observation) const {
  if (!ContinueRunning()) return false;
  if (!agx_xarm_pick::valid_visual_target_observation(observation) ||
      observation.frame_id != config_->color_frame) {
    return false;
  }
  ros::Time stamp;
  stamp.fromNSec(observation.stamp_ns);
  const ros::Time now = ros::Time::now();
  return !now.isZero() && stamp <= now &&
      now - stamp <= ros::Duration(config_->maximum_frame_age_sec);
}

TubeMissionStatus TubeGraspMission::StartLift(
    LiftOperation* operation) {
  if (operation == nullptr) {
    return TubeMissionStatus::kInvalidRequest;
  }
  runtime_geometry_resolved_ = false;
  if (!lift_target_.has_value() ||
      !lift_client_.exists() ||
      lift_state_subscriber_.getNumPublishers() == 0) {
    return TubeMissionStatus::kLiftInterfaceUnavailable;
  }

  bt_task_msgs::LiftMotorSrvRequest request;
  bt_task_msgs::LiftMotorSrvResponse response;
  if (!ContinueRunning()) return TubeMissionStatus::kInterrupted;
  request.mode = -5;
  request.val = config_->lift_maximum_speed_request_mm_s;
  if (!lift_client_.call(request, response) ||
      !agx_xarm_pick::accepted_lift_speed_service_state(
          response.state)) {
    return TubeMissionStatus::kLiftSpeedRequestRejected;
  }

  {
    std::lock_guard<std::mutex> lock(lift_mutex_);
    operation->target = *lift_target_;
    operation->command_time = std::chrono::steady_clock::now();
    operation->feedback_epoch_at_start = lift_feedback_epoch_;
  }

  if (!ContinueRunning()) return TubeMissionStatus::kInterrupted;
  request.mode = 0;
  request.val = operation->target.millimetres;
  if (!lift_client_.call(request, response) ||
      !agx_xarm_pick::accepted_lift_service_state(response.state)) {
    return TubeMissionStatus::kLiftRequestRejected;
  }
  return TubeMissionStatus::kSuccess;
}

TubeMissionStatus TubeGraspMission::FinishLift(
    const LiftOperation& operation) {
  const auto deadline = operation.command_time +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(
              config_->lift_motion_timeout_sec));
  bool received_post_command_feedback = false;
  bool received_matching_target_feedback = false;
  bool received_target_mismatch = false;
  while (ContinueRunning() && std::chrono::steady_clock::now() < deadline) {
    bt_task_msgs::LiftMotorMsg feedback;
    std::chrono::steady_clock::time_point receipt_time;
    bool has_feedback = false;
    std::uint64_t feedback_epoch = 0;
    std::uint64_t latest_fault_epoch = 0;
    std::chrono::steady_clock::time_point fault_receipt_time;
    {
      std::lock_guard<std::mutex> lock(lift_mutex_);
      has_feedback = has_lift_state_;
      feedback_epoch = lift_feedback_epoch_;
      latest_fault_epoch = latest_lift_fault_epoch_;
      fault_receipt_time = latest_lift_fault_wall_time_;
      if (has_feedback) {
        feedback = latest_lift_state_;
        receipt_time = latest_lift_wall_time_;
      }
    }

    if (agx_xarm_pick::lift_fault_evidence_belongs_to_operation(
            latest_fault_epoch,
            operation.feedback_epoch_at_start,
            SteadySeconds(fault_receipt_time),
            SteadySeconds(operation.command_time))) {
      return TubeMissionStatus::kLiftFeedbackUnavailable;
    }
    const auto wall_now = std::chrono::steady_clock::now();
    const ros::Time ros_now = ros::Time::now();
    const bool post_command =
        has_feedback &&
        feedback_epoch > operation.feedback_epoch_at_start &&
        receipt_time > operation.command_time;
    received_post_command_feedback =
        received_post_command_feedback || post_command;
    const bool stamp_valid =
        post_command &&
        !feedback.header.stamp.isZero() &&
        feedback.header.stamp <= ros::Time::now();
    const double wall_age_sec = post_command
        ? std::chrono::duration<double>(wall_now - receipt_time).count()
        : std::numeric_limits<double>::infinity();
    const double stamp_age_sec = stamp_valid
        ? (ros_now - feedback.header.stamp).toSec()
        : std::numeric_limits<double>::infinity();
    const bool fresh =
        stamp_valid &&
        wall_age_sec >= 0.0 &&
        wall_age_sec <= config_->lift_state_max_age_sec &&
        stamp_age_sec >= 0.0 &&
        stamp_age_sec <= config_->lift_state_max_age_sec;
    const bool fault_free =
        agx_xarm_pick::lift_feedback_is_fault_free(
            feedback.overflowI,
            feedback.overflowV,
            feedback.encodeErr,
            feedback.posBiasOver,
            feedback.underVol,
            feedback.overLoad,
            feedback.externalCtrlMode,
            feedback.stopFlag,
            feedback.fpsError);
    if (fresh && !fault_free) {
      return TubeMissionStatus::kLiftFeedbackUnavailable;
    }
    const bool usable =
        fresh && feedback.initState;
    if (usable &&
        feedback.targetHeight != operation.target.millimetres) {
      received_target_mismatch = true;
    }
    const bool target_matches =
        usable &&
        feedback.targetHeight == operation.target.millimetres;
    received_matching_target_feedback =
        received_matching_target_feedback || target_matches;
    const bool numeric_target_reached =
        agx_xarm_pick::lift_feedback_reached(
            target_matches,
            feedback.initState,
            feedback.backHeight,
            feedback.targetHeight,
            wall_age_sec,
            stamp_age_sec,
            config_->lift_state_max_age_sec,
            config_->lift_target_tolerance_mm);
    if (target_matches &&
        feedback.reachTargetPos &&
        !numeric_target_reached) {
      return TubeMissionStatus::kLiftTargetMismatch;
    }
    if (numeric_target_reached) {
      measured_lift_m_ =
          static_cast<double>(feedback.backHeight) / 1000.0;
      ResolveLiftedGeometry();
      if (!runtime_geometry_resolved_) {
        return TubeMissionStatus::kLiftTargetMismatch;
      }
      ROS_INFO(
          "tube fixture lift reached target=%ld mm measured=%d mm",
          static_cast<long>(operation.target.millimetres),
          feedback.backHeight);
      return TubeMissionStatus::kSuccess;
    }
    std::this_thread::sleep_for(kPollPeriod);
  }

  if (!ContinueRunning()) {
    return TubeMissionStatus::kInterrupted;
  }
  if (received_matching_target_feedback) {
    return TubeMissionStatus::kLiftMotionTimedOut;
  }
  if (received_target_mismatch) {
    return TubeMissionStatus::kLiftTargetMismatch;
  }
  if (!received_post_command_feedback) {
    ROS_ERROR("tube lift produced no post-command feedback");
  }
  return TubeMissionStatus::kLiftFeedbackUnavailable;
}


TubeMissionStatus TubeGraspMission::ResetPhaseOneMoveItTarget(
    TubeMissionStatus primary_status,
    bool stop_motion,
    const moveit::core::RobotStatePtr& synchronized_state) {
  bool cleanup_failed = false;
  if (stop_motion) {
    try {
      move_group_->stop();
    } catch (const std::exception& exception) {
      cleanup_failed = true;
      ROS_ERROR(
          "tube phase-one MoveIt stop threw during cleanup: %s",
          exception.what());
    } catch (...) {
      cleanup_failed = true;
      ROS_ERROR(
          "tube phase-one MoveIt stop threw an unknown exception "
          "during cleanup");
    }
  }

  moveit::core::RobotStatePtr reset_state = synchronized_state;
  if (!reset_state) {
    try {
      if (!WaitForSynchronizedRobotState(
              std::chrono::steady_clock::time_point{},
              PlanningDeadline(config_->planning_timeout_sec),
              &reset_state)) {
        cleanup_failed = true;
      }
    } catch (const std::exception& exception) {
      cleanup_failed = true;
      ROS_ERROR(
          "tube phase-one synchronized reset-state read threw: %s",
          exception.what());
    } catch (...) {
      cleanup_failed = true;
      ROS_ERROR(
          "tube phase-one synchronized reset-state read threw an "
          "unknown exception");
    }
  }

  if (reset_state) {
    try {
      const std::vector<double> measured_joint_target =
          OrderedMoveItArmPositions(*reset_state);
      if (!move_group_->setJointValueTarget(measured_joint_target)) {
        cleanup_failed = true;
      }
    } catch (const std::exception& exception) {
      cleanup_failed = true;
      ROS_ERROR(
          "tube phase-one measured joint-target reset threw: %s",
          exception.what());
    } catch (...) {
      cleanup_failed = true;
      ROS_ERROR(
          "tube phase-one measured joint-target reset threw an "
          "unknown exception");
    }
  }

  if (!cleanup_failed) {
    return primary_status;
  }
  if (primary_status == TubeMissionStatus::kSuccess) {
    ROS_ERROR(
        "tube phase-one succeeded but its MoveIt joint-target reset failed");
    return TubeMissionStatus::kPhaseOneExecutionFailed;
  }
  ROS_ERROR(
      "tube phase-one MoveIt cleanup failed; preserving primary status: %s",
      tube_mission_status_name(primary_status));
  return primary_status;
}

TubeMissionStatus TubeGraspMission::MoveToPhaseOne(
    ArmMotion motion,
    std::optional<double> preserved_tool_yaw_rad) {
  if (!ContinueRunning()) {
    return TubeMissionStatus::kInterrupted;
  }

  try {
    const auto planning_deadline =
        PlanningDeadline(config_->planning_timeout_sec);
    moveit::core::RobotStatePtr planning_state;
    if (!WaitForSynchronizedRobotState(
            std::chrono::steady_clock::time_point{},
            planning_deadline,
            &planning_state)) {
      return ContinueRunning()
          ? TubeMissionStatus::kPhaseOnePlanFailed
          : TubeMissionStatus::kInterrupted;
    }
    std::vector<double> phase_one_joint_target =
        config_->phase_one_joint_positions_rad;
    if (preserved_tool_yaw_rad.has_value()) {
      if (!std::isfinite(*preserved_tool_yaw_rad)) {
        return ResetPhaseOneMoveItTarget(
            TubeMissionStatus::kPhaseOnePlanFailed, false, planning_state);
      }
      // Joint 6 rotates about the top-down tool axis, with the opposite sign
      // to world-frame TCP yaw. Keep the diagonal grasp angle while returning
      // to the commissioned phase-one TCP location so the held tube is not
      // twisted between the fingertips.
      phase_one_joint_target.back() -= std::remainder(
          *preserved_tool_yaw_rad, 2.0 * kPi);
      ROS_INFO(
          "tube payload return preserving tool yaw=%.6f rad",
          *preserved_tool_yaw_rad);
    }
    move_group_->setStartState(*planning_state);
    if (!move_group_->setJointValueTarget(
            phase_one_joint_target)) {
      if (!ContinueRunning()) {
        return ResetPhaseOneMoveItTarget(
            TubeMissionStatus::kInterrupted, true, planning_state);
      }
      return ResetPhaseOneMoveItTarget(
          TubeMissionStatus::kPhaseOnePlanFailed, false, planning_state);
    }

    ApplyArmExecution(motion);
    moveit::planning_interface::MoveGroupInterface::Plan phase_one_plan;
    if (move_group_->plan(phase_one_plan) !=
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      const TubeMissionStatus failure = ContinueRunning()
          ? TubeMissionStatus::kPhaseOnePlanFailed
          : TubeMissionStatus::kInterrupted;
      return ResetPhaseOneMoveItTarget(
          failure, failure == TubeMissionStatus::kInterrupted);
    }

    moveit::core::RobotStatePtr execution_state;
    if (!WaitForSynchronizedRobotState(
            std::chrono::steady_clock::time_point{},
            planning_deadline,
            &execution_state)) {
      const TubeMissionStatus failure = ContinueRunning()
          ? TubeMissionStatus::kPhaseOnePlanFailed
          : TubeMissionStatus::kInterrupted;
      return ResetPhaseOneMoveItTarget(
          failure,
          failure == TubeMissionStatus::kInterrupted,
          execution_state);
    }
    if (!ContinueRunning()) {
      return ResetPhaseOneMoveItTarget(
          TubeMissionStatus::kInterrupted, true, execution_state);
    }

    if (move_group_->execute(phase_one_plan) !=
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
      const TubeMissionStatus failure = ContinueRunning()
          ? TubeMissionStatus::kPhaseOneExecutionFailed
          : TubeMissionStatus::kInterrupted;
      return ResetPhaseOneMoveItTarget(failure, true);
    }
    if (!ContinueRunning()) {
      return ResetPhaseOneMoveItTarget(
          TubeMissionStatus::kInterrupted, true);
    }

    const auto execution_completion_time =
        std::chrono::steady_clock::now();
    geometry_msgs::Pose measured_tcp;
    moveit::core::RobotStatePtr synchronized_state;
    if (!CurrentTcpPose(
            execution_completion_time +
                std::chrono::steady_clock::duration(1),
            PlanningDeadline(config_->planning_timeout_sec),
            &measured_tcp,
            &synchronized_state) ||
        !synchronized_state) {
      const TubeMissionStatus failure = ContinueRunning()
          ? TubeMissionStatus::kPhaseOneExecutionFailed
          : TubeMissionStatus::kInterrupted;
      return ResetPhaseOneMoveItTarget(
          failure, true, synchronized_state);
    }

    const bool position_valid = tcp_pose_within_tolerance(
        config_->phase_one_expected_tcp.x,
        config_->phase_one_expected_tcp.y,
        config_->phase_one_expected_tcp.z,
        measured_tcp.position.x,
        measured_tcp.position.y,
        measured_tcp.position.z,
        kPhaseOneTcpPositionToleranceM);
    const bool tcp_axis_downward = tcp_approach_axis_is_downward(
        measured_tcp.orientation.x,
        measured_tcp.orientation.y,
        measured_tcp.orientation.z,
        measured_tcp.orientation.w,
        config_->phase_one_downward_axis_tolerance_deg);
    const bool camera_axis_downward = CameraAxisIsDownwardAtTcpPose(
        measured_tcp, config_->phase_one_downward_axis_tolerance_deg);
    const std::vector<double> measured_joints =
        OrderedMoveItArmPositions(*synchronized_state);
    if (!position_valid || !tcp_axis_downward || !camera_axis_downward) {
      ROS_ERROR(
          "tube phase-one invariant failed: "
          "expected_tcp=(%.6f, %.6f, %.6f) "
          "measured_tcp=(%.6f, %.6f, %.6f) "
          "position_tolerance_m=%.6f "
          "tcp_downward_ok=%d camera_downward_ok=%d "
          "measured_joints=(%.6f, %.6f, %.6f, %.6f, %.6f, %.6f)",
          config_->phase_one_expected_tcp.x,
          config_->phase_one_expected_tcp.y,
          config_->phase_one_expected_tcp.z,
          measured_tcp.position.x,
          measured_tcp.position.y,
          measured_tcp.position.z,
          kPhaseOneTcpPositionToleranceM,
          tcp_axis_downward,
          camera_axis_downward,
          measured_joints[0],
          measured_joints[1],
          measured_joints[2],
          measured_joints[3],
          measured_joints[4],
          measured_joints[5]);
      return ResetPhaseOneMoveItTarget(
          TubeMissionStatus::kPhaseOneExecutionFailed,
          true,
          synchronized_state);
    }
    return ResetPhaseOneMoveItTarget(
        TubeMissionStatus::kSuccess, false, synchronized_state);
  } catch (const std::exception& exception) {
    ROS_ERROR(
        "tube phase-one MoveIt operation threw: %s", exception.what());
    const TubeMissionStatus failure = ContinueRunning()
        ? TubeMissionStatus::kPhaseOneExecutionFailed
        : TubeMissionStatus::kInterrupted;
    return ResetPhaseOneMoveItTarget(failure, true);
  } catch (...) {
    ROS_ERROR("tube phase-one MoveIt operation threw an unknown exception");
    const TubeMissionStatus failure = ContinueRunning()
        ? TubeMissionStatus::kPhaseOneExecutionFailed
        : TubeMissionStatus::kInterrupted;
    return ResetPhaseOneMoveItTarget(failure, true);
  }
}

TubeMissionStatus TubeGraspMission::PrepareThroughPhaseOne() {
  LiftOperation lift_operation;
  const auto lift_start = StartLift(&lift_operation);
  if (lift_start != TubeMissionStatus::kSuccess) {
    return lift_start;
  }
  TubeMissionStatus phase_one;
  try {
    phase_one = MoveToPhaseOne(ArmMotion::Phase1Preparation);
  } catch (const std::exception& exception) {
    ROS_ERROR(
        "tube phase-one escaped cleanup with exception: %s",
        exception.what());
    phase_one = ContinueRunning()
        ? TubeMissionStatus::kPhaseOneExecutionFailed
        : TubeMissionStatus::kInterrupted;
  } catch (...) {
    ROS_ERROR("tube phase-one escaped cleanup with unknown exception");
    phase_one = ContinueRunning()
        ? TubeMissionStatus::kPhaseOneExecutionFailed
        : TubeMissionStatus::kInterrupted;
  }
  const auto lift_finish = FinishLift(lift_operation);
  if (phase_one != TubeMissionStatus::kSuccess) {
    if (lift_finish != TubeMissionStatus::kSuccess) {
      ROS_ERROR(
          "tube lift also failed during phase-one failure: %s; "
          "preserving primary phase-one status: %s",
          tube_mission_status_name(lift_finish),
          tube_mission_status_name(phase_one));
    }
    return phase_one;
  }
  return lift_finish;
}

void TubeGraspMission::ResolveLiftedGeometry() {
  const double zero_lift_cap_top_z_m = lift_target_.has_value()
      ? lift_target_->metres +
          config_->rack_height_above_surface_m +
          config_->body_height_m +
          config_->cap_height_m
      : std::numeric_limits<double>::quiet_NaN();
  runtime_cap_top_z_m_ = agx_xarm_pick::resolve_runtime_cap_top_z(
      zero_lift_cap_top_z_m, measured_lift_m_);
  runtime_geometry_resolved_ =
      std::isfinite(runtime_cap_top_z_m_) &&
      runtime_cap_top_z_m_ >= config_->workspace_min_xyz_m[2] &&
      runtime_cap_top_z_m_ + config_->view_region.clearance_max_m <=
          config_->workspace_max_xyz_m[2];
  if (runtime_geometry_resolved_) {
    ROS_INFO(
        "tube cap plane resolved zero_lift=%.6f m "
        "measured_lift=%.6f m runtime=%.6f m",
        zero_lift_cap_top_z_m,
        measured_lift_m_,
        runtime_cap_top_z_m_);
  }
}

TubeGraspMission::SearchResult TubeGraspMission::PrepareAndSearch(
    const std::string& fixture_profile_id,
    const std::string& target_cap_color) {
  SearchResult result;
  if (!runtime_geometry_resolved_) {
    result.status = TubeMissionStatus::kLiftFeedbackUnavailable;
    return result;
  }
  geometry_msgs::Pose current_tcp;
  moveit::core::RobotStatePtr current_state;
  if (!CurrentTcpPose(
          std::chrono::steady_clock::time_point{},
          PlanningDeadline(config_->planning_timeout_sec),
          &current_tcp,
          &current_state)) {
    result.status = TubeMissionStatus::kSetupUnavailable;
    return result;
  }
  const double current_joint1 =
      current_state->getVariablePosition("joint1");
  const bool phase_one_position_valid = tcp_pose_within_tolerance(
      config_->phase_one_expected_tcp.x,
      config_->phase_one_expected_tcp.y,
      config_->phase_one_expected_tcp.z,
      current_tcp.position.x,
      current_tcp.position.y,
      current_tcp.position.z,
      kPhaseOneTcpPositionToleranceM);
  const bool phase_one_centerline_y_valid =
      std::abs(
          current_tcp.position.y -
          config_->phase_two_centerline_y_m) <=
      config_->initial_centerline_tolerance_m;
  const bool phase_one_centerline_joint_valid =
      std::abs(
          current_joint1 -
          config_->centerline_joint_position_rad) <=
      config_->centerline_joint_tolerance_rad;
  const bool phase_one_tcp_axis_downward =
      tcp_approach_axis_is_downward(
          current_tcp.orientation.x,
          current_tcp.orientation.y,
          current_tcp.orientation.z,
          current_tcp.orientation.w,
          config_->phase_one_downward_axis_tolerance_deg);
  const bool phase_one_camera_axis_downward =
      CameraAxisIsDownwardAtTcpPose(
          current_tcp,
          config_->phase_one_downward_axis_tolerance_deg);
  const bool phase_one_handoff_is_safe =
      phase_one_position_valid &&
      phase_one_centerline_y_valid &&
      phase_one_centerline_joint_valid &&
      phase_one_tcp_axis_downward &&
      phase_one_camera_axis_downward;
  if (!phase_one_handoff_is_safe) {
    ROS_ERROR(
        "tube search refused: Phase 1 TCP=(%.6f, %.6f, %.6f), "
        "joint1=%.6f is outside its commissioned compact state",
        current_tcp.position.x,
        current_tcp.position.y,
        current_tcp.position.z,
        current_joint1);
    result.status = TubeMissionStatus::kUnsafeInitialCenterline;
    return result;
  }

  result.status = MoveTopDown(
      ArmMotion::Phase2Preparation,
      config_->view_region,
      config_->view_entry_y_m,
      config_->view_region.clearance_min_m,
      TubeMissionStatus::kPreparationPlanFailed,
      TubeMissionStatus::kPreparationExecutionFailed);
  if (result.status != TubeMissionStatus::kSuccess) {
    return result;
  }
  ROS_INFO(
      "tube flexible Phase 2 reached the top-down entry band at y=%.6f",
      config_->view_entry_y_m);

  for (double observation_y_m : config_->observation_y_positions_m) {
    result.status = MoveTopDown(
        ArmMotion::Search,
        config_->view_region,
        observation_y_m,
        config_->view_region.clearance_min_m,
        TubeMissionStatus::kSearchWaypointPlanFailed,
        TubeMissionStatus::kSearchWaypointExecutionFailed);
    if (result.status != TubeMissionStatus::kSuccess) {
      return result;
    }

    ros::Time not_before;
    result.status = SettleForObservation(&not_before);
    if (result.status != TubeMissionStatus::kSuccess) {
      return result;
    }

    TubeMissionStatus detection_status = TubeMissionStatus::kTargetAbsent;
    for (int attempt = 0;
         attempt < config_->detector_attempts_per_waypoint;
         ++attempt) {
      TubeObservation observation;
      detection_status = DetectObservation(
          fixture_profile_id,
          target_cap_color,
          agx_pick_msg::DetectTubeCapRequest::TARGET_ONLY,
          not_before,
          &observation);
      if (detection_status == TubeMissionStatus::kSuccess) {
        // Alignment needs the cap in the current image. Continuing the sweep
        // can move it out of view even though an earlier detection succeeded.
        result.found = true;
        result.observation = observation;
        result.status = TubeMissionStatus::kSuccess;
        return result;
      }
      if (search_status_retries(detection_status)) {
        ROS_WARN(
            "tube search detector retry %d/%d at y=%.6f: %s",
            attempt + 1,
            config_->detector_attempts_per_waypoint,
            observation_y_m,
            tube_mission_status_name(detection_status));
        continue;
      }
      break;
    }

    if (search_status_advances(detection_status)) {
      continue;
    }
    result.status = detection_status;
    return result;
  }

  result.status = TubeMissionStatus::kSearchExhausted;
  return result;
}

TubeMissionStatus TubeGraspMission::RunSearchOnly(
    const std::string& fixture_profile_id,
    const std::string& target_cap_color) {
  std::unique_lock<std::mutex> mission_lock(
      mission_mutex_, std::try_to_lock);
  if (!mission_lock.owns_lock()) {
    return TubeMissionStatus::kBusy;
  }
  payload_ = PayloadKnowledge::kUnknown;

  TubeMissionStatus status = ValidateAndPrepare(
      fixture_profile_id, target_cap_color);
  if (status != TubeMissionStatus::kSuccess) {
    return status;
  }
  status = PrepareThroughPhaseOne();
  if (status != TubeMissionStatus::kSuccess) {
    return status;
  }
  ROS_INFO("tube mission stage complete: preparation");
  if (!BaseStoppedIfAvailable()) {
    return TubeMissionStatus::kBaseNotStopped;
  }
  payload_ = PayloadKnowledge::kKnownEmpty;
  const SearchResult search = PrepareAndSearch(
      fixture_profile_id, target_cap_color);
  if (search.status != TubeMissionStatus::kSuccess) {
    return Fail(search.status);
  }
  ROS_INFO("tube mission stage complete: search");
  ROS_INFO(
      "tube search completed; target=(%.6f, %.6f, %.6f), "
      "no grasp was attempted",
      search.observation.cap_top.point.x,
      search.observation.cap_top.point.y,
      search.observation.cap_top.point.z);
  return TubeMissionStatus::kSuccess;
}


TubeMissionStatus TubeGraspMission::RunVisualAlignmentUnlocked(
    const std::string& fixture_profile_id,
    agx_xarm_pick::VisualAlignmentResult* result) {
  if (result == nullptr) {
    return TubeMissionStatus::kInvalidRequest;
  }
  agx_xarm_pick::RosTubeTargetProvider provider(
      node_, resolved_config_, config_->target_cap_color);
  const auto state_reader = [this](
      std::chrono::steady_clock::time_point minimum_receipt_time,
      std::chrono::steady_clock::time_point deadline,
      moveit::core::RobotStatePtr* state,
      geometry_msgs::Pose* grasp_tcp_pose) {
    return ReadAlignmentState(
        minimum_receipt_time, deadline, state, grasp_tcp_pose);
  };
  const auto target_is_fresh = [this](
      const agx_xarm_pick::VisualTargetObservation& observation) {
    return VisualTargetIsFresh(observation);
  };
  agx_xarm_pick::MoveItVisualAlignmentMotion motion(
      move_group_.get(),
      BuildAlignmentMotionConfig(ArmMotion::VisualAlignment),
      state_reader,
      target_is_fresh);
  const auto settle = [this]() -> std::uint64_t {
    ros::Time not_before;
    return SettleForObservation(&not_before) == TubeMissionStatus::kSuccess
        ? not_before.toNSec()
        : 0;
  };
  agx_xarm_pick::VisualTargetAligner aligner(
      config_->visual_alignment,
      &provider,
      &motion,
      settle,
      [this]() { return !ContinueRunning(); },
      config_->downward_axis_tolerance_deg);
  *result = aligner.Run();
  const std::string alignment_trace_path =
      agx_xarm_pick::default_visual_alignment_trace_path("tube_pick");
  std::string alignment_trace_error;
  if (!agx_xarm_pick::write_visual_alignment_trace(
          alignment_trace_path,
          "tube_pick",
          *result,
          config_->visual_alignment.u_axis_gain_m_per_px,
          config_->visual_alignment.v_axis_gain_m_per_px,
          &alignment_trace_error)) {
    ROS_WARN(
        "alignment trace could not be written: %s",
        alignment_trace_error.c_str());
  } else {
    ROS_INFO(
        "tube visual alignment trace: %s",
        alignment_trace_path.c_str());
  }
  if (result->status != agx_xarm_pick::VisualAlignmentStatus::kSuccess) {
    ROS_WARN(
        "tube visual alignment failed: status=%s detail=%s "
        "pixel_error_px=%.3f completed_moves=%d displacement_m=%.6f",
        agx_xarm_pick::visual_alignment_status_name(result->status),
        result->detail.c_str(),
        result->final_pixel_error_px,
        result->completed_moves,
        result->cumulative_displacement_m);
  }
  return AlignmentMissionStatus(result->status);
}


TubeMissionStatus TubeGraspMission::Run(
    const std::string& fixture_profile_id,
    const std::string& target_cap_color) {
  std::unique_lock<std::mutex> mission_lock(
      mission_mutex_, std::try_to_lock);
  if (!mission_lock.owns_lock()) {
    return TubeMissionStatus::kBusy;
  }
  payload_ = PayloadKnowledge::kUnknown;
  TubeMissionStatus status = ValidateAndPrepare(
      fixture_profile_id, target_cap_color);
  if (status != TubeMissionStatus::kSuccess) {
    return status;
  }
  status = PrepareThroughPhaseOne();
  if (status != TubeMissionStatus::kSuccess) {
    return status;
  }
  ROS_INFO("tube mission stage complete: preparation");
  if (!BaseStoppedIfAvailable()) {
    return TubeMissionStatus::kBaseNotStopped;
  }
  payload_ = PayloadKnowledge::kKnownEmpty;
  SearchResult search = PrepareAndSearch(
      fixture_profile_id, target_cap_color);
  if (search.status != TubeMissionStatus::kSuccess) {
    return Fail(search.status);
  }
  ROS_INFO("tube mission stage complete: search");

  agx_xarm_pick::VisualAlignmentResult alignment_result;
  status = RunVisualAlignmentUnlocked(
      fixture_profile_id, &alignment_result);
  if (status != TubeMissionStatus::kSuccess) {
    return Fail(status);
  }
  ROS_INFO("tube mission stage complete: visual_alignment");

  ros::Time not_before;
  status = SettleForObservation(&not_before);
  if (status != TubeMissionStatus::kSuccess) {
    return Fail(status);
  }
  TubeObservation centered_target;
  status = DetectObservation(
      fixture_profile_id,
      target_cap_color,
      agx_pick_msg::DetectTubeCapRequest::TARGET_ONLY,
      not_before,
      &centered_target);
  if (status != TubeMissionStatus::kSuccess) {
    return Fail(status);
  }
  ROS_INFO("tube mission stage complete: centered_target");
  return ExecuteGraspFromCenteredTarget(
      fixture_profile_id,
      target_cap_color,
      centered_target,
      nullptr);
}

TubeMissionStatus TubeGraspMission::ExecuteGraspFromCenteredTarget(
    const std::string& fixture_profile_id,
    const std::string& target_cap_color,
    const TubeObservation& centered_target,
    const TubeObservation* stationary_layout) {
  TubeObservation layout;
  TubeMissionStatus status = DetectLayout(
      fixture_profile_id,
      target_cap_color,
      centered_target,
      &layout,
      true,
      stationary_layout);
  if (status != TubeMissionStatus::kSuccess) {
    return Fail(status);
  }
  ROS_INFO("tube mission stage complete: layout_snapshot");
  const TubeObservation* retained_layout = &layout;

  const double desired_inner_gap_m =
      config_->cap_diameter_m +
      2.0 * config_->insertion_radial_clearance_m;
  std::vector<GapSample> calibrated_gap_samples;
  calibrated_gap_samples.reserve(config_->command_gap_samples.size());
  for (const auto& sample : config_->command_gap_samples) {
    calibrated_gap_samples.push_back({sample[0], sample[1]});
  }
  const auto insertion_command = command_for_inner_gap(
      calibrated_gap_samples, desired_inner_gap_m);
  if (!insertion_command) {
    return Fail(TubeMissionStatus::kInsertionApertureFailed);
  }

  const Point2 target_xy = {
      layout.cap_top.point.x, layout.cap_top.point.y};
  std::vector<Point2> occupied;
  occupied.reserve(layout.observed_cap_tops.size());
  for (std::size_t index = 0;
       index < layout.observed_cap_tops.size();
       ++index) {
    if (layout.grid_u[index] == 0 && layout.grid_v[index] == 0) {
      occupied.push_back(target_xy);
    } else {
      const auto& cap_top = layout.observed_cap_tops[index];
      occupied.push_back({cap_top.x, cap_top.y});
    }
  }
  const Point2 axis_u_xy = {
      layout.grid_axis_u.x, layout.grid_axis_u.y};
  const Point2 axis_v_xy = {
      layout.grid_axis_v.x, layout.grid_axis_v.y};
  const auto diagonal = choose_grasp_diagonal(
      target_xy,
      occupied,
      axis_u_xy,
      axis_v_xy,
      desired_inner_gap_m,
      config_->finger_thickness_m,
      config_->finger_width_m,
      config_->cap_diameter_m,
      config_->tube_clearance_margin_m);
  if (!diagonal) {
    return Fail(TubeMissionStatus::kClearanceFailed);
  }
  const double grasp_line_yaw_rad =
      std::atan2(diagonal->direction.y, diagonal->direction.x);

  status = AlignTcpOverTarget(layout);
  if (status != TubeMissionStatus::kSuccess) {
    return Fail(status);
  }
  if (payload_ != PayloadKnowledge::kKnownEmpty) {
    return Fail(TubeMissionStatus::kPayloadStateUnsafe);
  }

  geometry_msgs::Pose translated_pregrasp_tcp;
  TubeObservation pre_rotation_layout = layout;
  bool tcp_centered_over_fresh_target = false;
  for (std::size_t correction_count = 0;
       correction_count <= kMaximumTcpRecenteringCorrections;
       ++correction_count) {
    TubeObservation refreshed_target;
    status = DetectLayout(
        fixture_profile_id,
        target_cap_color,
        pre_rotation_layout,
        &refreshed_target,
        false,
        retained_layout);
    if (status != TubeMissionStatus::kSuccess) {
      return Fail(status);
    }
    if (!refreshed_target_xy_shift_is_safe(
            pre_rotation_layout.cap_top.point.x,
            pre_rotation_layout.cap_top.point.y,
            refreshed_target.cap_top.point.x,
            refreshed_target.cap_top.point.y,
            config_->pitch_tolerance_m,
            config_->max_interframe_spread_m)) {
      return Fail(TubeMissionStatus::kInconsistentTarget);
    }
    pre_rotation_layout = refreshed_target;

    const bool pregrasp_state_available = CurrentTcpPose(
        std::chrono::steady_clock::time_point{},
        PlanningDeadline(config_->planning_timeout_sec),
        &translated_pregrasp_tcp);
    const bool pregrasp_camera_down = pregrasp_state_available &&
        CameraAxisIsDownwardAtTcpPose(translated_pregrasp_tcp);
    tcp_centered_over_fresh_target = pregrasp_state_available &&
        vertical_xy_path_within_tolerance(
            {
                pre_rotation_layout.cap_top.point.x,
                pre_rotation_layout.cap_top.point.y,
            },
            {{
                translated_pregrasp_tcp.position.x,
                translated_pregrasp_tcp.position.y,
            }},
            kMaximumVerticalXyDriftM);
    if (!pregrasp_state_available || !pregrasp_camera_down) {
      return Fail(TubeMissionStatus::kPregraspFailed);
    }
    if (tcp_centered_over_fresh_target) {
      break;
    }
    if (correction_count == kMaximumTcpRecenteringCorrections) {
      ROS_ERROR(
          "tube TCP recentering exhausted: target_xy=(%.6f, %.6f) "
          "measured_xy=(%.6f, %.6f) tolerance=%.6f",
          pre_rotation_layout.cap_top.point.x,
          pre_rotation_layout.cap_top.point.y,
          translated_pregrasp_tcp.position.x,
          translated_pregrasp_tcp.position.y,
          kMaximumVerticalXyDriftM);
      return Fail(TubeMissionStatus::kPregraspFailed);
    }
    const double residual_x_m =
        pre_rotation_layout.cap_top.point.x -
        translated_pregrasp_tcp.position.x;
    const double residual_y_m =
        pre_rotation_layout.cap_top.point.y -
        translated_pregrasp_tcp.position.y;
    const double residual_m = std::hypot(residual_x_m, residual_y_m);
    const double maximum_correction_m = execution_position_tolerance_m(
        config_->view_region.y_tolerance_m);
    if (!std::isfinite(residual_m) ||
        residual_m > maximum_correction_m) {
      ROS_ERROR(
          "tube TCP recentering residual %.6f exceeds correction bound "
          "%.6f",
          residual_m,
          maximum_correction_m);
      return Fail(TubeMissionStatus::kPregraspFailed);
    }
    TubeObservation correction_target = pre_rotation_layout;
    correction_target.cap_top.point.x += residual_x_m;
    correction_target.cap_top.point.y += residual_y_m;
    ROS_INFO(
        "tube TCP recentering correction %zu/%zu: "
        "target_xy=(%.6f, %.6f) measured_xy=(%.6f, %.6f) "
        "command_xy=(%.6f, %.6f)",
        correction_count + 1,
        kMaximumTcpRecenteringCorrections,
        pre_rotation_layout.cap_top.point.x,
        pre_rotation_layout.cap_top.point.y,
        translated_pregrasp_tcp.position.x,
        translated_pregrasp_tcp.position.y,
        correction_target.cap_top.point.x,
        correction_target.cap_top.point.y);
    status = AlignTcpOverTarget(correction_target);
    if (status != TubeMissionStatus::kSuccess) {
      return Fail(status);
    }
  }
  if (!tcp_centered_over_fresh_target) {
    return Fail(TubeMissionStatus::kPregraspFailed);
  }
  const Point2 pre_rotation_target_xy = {
      pre_rotation_layout.cap_top.point.x,
      pre_rotation_layout.cap_top.point.y};
  std::vector<Point2> pre_rotation_occupied;
  pre_rotation_occupied.reserve(
      pre_rotation_layout.observed_cap_tops.size());
  for (std::size_t index = 0;
       index < pre_rotation_layout.observed_cap_tops.size();
       ++index) {
    if (pre_rotation_layout.grid_u[index] == 0 &&
        pre_rotation_layout.grid_v[index] == 0) {
      pre_rotation_occupied.push_back(pre_rotation_target_xy);
    } else {
      const auto& cap_top =
          pre_rotation_layout.observed_cap_tops[index];
      pre_rotation_occupied.push_back({cap_top.x, cap_top.y});
    }
  }
  const auto pre_rotation_diagonal = choose_grasp_diagonal(
      pre_rotation_target_xy,
      pre_rotation_occupied,
      {
          pre_rotation_layout.grid_axis_u.x,
          pre_rotation_layout.grid_axis_u.y,
      },
      {
          pre_rotation_layout.grid_axis_v.x,
          pre_rotation_layout.grid_axis_v.y,
      },
      desired_inner_gap_m,
      config_->finger_thickness_m,
      config_->finger_width_m,
      config_->cap_diameter_m,
      config_->tube_clearance_margin_m);
  if (!pre_rotation_diagonal) {
    return Fail(TubeMissionStatus::kClearanceFailed);
  }
  const double pre_rotation_line_yaw_rad = std::atan2(
      pre_rotation_diagonal->direction.y,
      pre_rotation_diagonal->direction.x);
  if (grasp_line_yaw_change_requires_refresh(
          grasp_line_yaw_rad,
          pre_rotation_line_yaw_rad,
          config_->maximum_yaw_error_deg)) {
    return Fail(TubeMissionStatus::kInconsistentTarget);
  }
  tf2::Quaternion current_pregrasp_quaternion;
  tf2::fromMsg(
      translated_pregrasp_tcp.orientation,
      current_pregrasp_quaternion);
  tf2::Matrix3x3 current_pregrasp_orientation(
      current_pregrasp_quaternion);
  double current_pregrasp_roll_rad = 0.0;
  double current_pregrasp_pitch_rad = 0.0;
  double current_pregrasp_yaw_rad = 0.0;
  current_pregrasp_orientation.getRPY(
      current_pregrasp_roll_rad,
      current_pregrasp_pitch_rad,
      current_pregrasp_yaw_rad);
  const double commanded_grasp_yaw_rad = canonical_grasp_line_yaw(
      pre_rotation_line_yaw_rad, current_pregrasp_yaw_rad);
  if (!std::isfinite(current_pregrasp_roll_rad) ||
      !std::isfinite(current_pregrasp_pitch_rad) ||
      !std::isfinite(commanded_grasp_yaw_rad)) {
    return Fail(TubeMissionStatus::kPregraspFailed);
  }
  status = RotateTcpAtPregrasp(
      translated_pregrasp_tcp,
      commanded_grasp_yaw_rad,
      pre_rotation_layout);
  if (status != TubeMissionStatus::kSuccess) {
    return Fail(status);
  }

  geometry_msgs::Pose last_validated_pregrasp_tcp;
  tf2::Quaternion selected_top_down;
  selected_top_down.setRPY(kPi, 0.0, commanded_grasp_yaw_rad);
  if (!CurrentTcpPose(
          std::chrono::steady_clock::time_point{},
          PlanningDeadline(config_->planning_timeout_sec),
          &last_validated_pregrasp_tcp) ||
      !CameraAxisIsDownwardAtTcpPose(last_validated_pregrasp_tcp) ||
      !quaternion_orientation_within_tolerance(
          {
              selected_top_down.x(),
              selected_top_down.y(),
              selected_top_down.z(),
              selected_top_down.w(),
          },
          {
              last_validated_pregrasp_tcp.orientation.x,
              last_validated_pregrasp_tcp.orientation.y,
              last_validated_pregrasp_tcp.orientation.z,
              last_validated_pregrasp_tcp.orientation.w,
          },
          config_->downward_axis_tolerance_deg)) {
    return Fail(TubeMissionStatus::kPregraspFailed);
  }
  TubeObservation refreshed_layout = pre_rotation_layout;
  bool post_yaw_tcp_centered = false;
  for (std::size_t correction_count = 0;
       correction_count <= kMaximumTcpRecenteringCorrections;
       ++correction_count) {
    TubeObservation post_yaw_target;
    status = DetectLayout(
        fixture_profile_id,
        target_cap_color,
        refreshed_layout,
        &post_yaw_target,
        false,
        retained_layout);
    if (status != TubeMissionStatus::kSuccess) {
      return Fail(status);
    }
    if (!refreshed_target_xy_shift_is_safe(
            refreshed_layout.cap_top.point.x,
            refreshed_layout.cap_top.point.y,
            post_yaw_target.cap_top.point.x,
            post_yaw_target.cap_top.point.y,
            config_->pitch_tolerance_m,
            config_->max_interframe_spread_m)) {
      return Fail(TubeMissionStatus::kInconsistentTarget);
    }
    refreshed_layout = post_yaw_target;
    if (!CurrentTcpPose(
            std::chrono::steady_clock::time_point{},
            PlanningDeadline(config_->planning_timeout_sec),
            &last_validated_pregrasp_tcp) ||
        !CameraAxisIsDownwardAtTcpPose(last_validated_pregrasp_tcp) ||
        !quaternion_orientation_within_tolerance(
            {
                selected_top_down.x(),
                selected_top_down.y(),
                selected_top_down.z(),
                selected_top_down.w(),
            },
            {
                last_validated_pregrasp_tcp.orientation.x,
                last_validated_pregrasp_tcp.orientation.y,
                last_validated_pregrasp_tcp.orientation.z,
                last_validated_pregrasp_tcp.orientation.w,
            },
            config_->downward_axis_tolerance_deg)) {
      return Fail(TubeMissionStatus::kPregraspFailed);
    }
    const double residual_x_m =
        refreshed_layout.cap_top.point.x -
        last_validated_pregrasp_tcp.position.x;
    const double residual_y_m =
        refreshed_layout.cap_top.point.y -
        last_validated_pregrasp_tcp.position.y;
    const double residual_m = std::hypot(residual_x_m, residual_y_m);
    post_yaw_tcp_centered = residual_m <= kMaximumVerticalXyDriftM;
    if (post_yaw_tcp_centered) {
      break;
    }
    const double maximum_correction_m = execution_position_tolerance_m(
        config_->view_region.y_tolerance_m);
    if (!std::isfinite(residual_m) ||
        residual_m > maximum_correction_m ||
        correction_count == kMaximumTcpRecenteringCorrections) {
      ROS_ERROR(
          "tube post-yaw TCP correction exhausted: residual=%.6f "
          "bound=%.6f",
          residual_m,
          maximum_correction_m);
      return Fail(TubeMissionStatus::kPregraspFailed);
    }
    const double correction_scale = std::min(
        2.0, maximum_correction_m / residual_m);
    const double correction_x_m = residual_x_m * correction_scale;
    const double correction_y_m = residual_y_m * correction_scale;
    ROS_INFO(
        "tube post-yaw TCP correction %zu/%zu: "
        "target_xy=(%.6f, %.6f) measured_xy=(%.6f, %.6f) "
        "command_delta=(%.6f, %.6f)",
        correction_count + 1,
        kMaximumTcpRecenteringCorrections,
        refreshed_layout.cap_top.point.x,
        refreshed_layout.cap_top.point.y,
        last_validated_pregrasp_tcp.position.x,
        last_validated_pregrasp_tcp.position.y,
        correction_x_m,
        correction_y_m);
    status = CorrectTcpPlanar(
        refreshed_layout,
        correction_x_m,
        correction_y_m,
        &last_validated_pregrasp_tcp);
    if (status != TubeMissionStatus::kSuccess) {
      return Fail(status);
    }
    ROS_INFO(
        "tube post-yaw TCP correction %zu measured endpoint=(%.6f, %.6f, %.6f)",
        correction_count + 1,
        last_validated_pregrasp_tcp.position.x,
        last_validated_pregrasp_tcp.position.y,
        last_validated_pregrasp_tcp.position.z);
  }
  if (!post_yaw_tcp_centered) {
    return Fail(TubeMissionStatus::kPregraspFailed);
  }
  const double last_validated_pregrasp_z =
      last_validated_pregrasp_tcp.position.z;
  const Point2 refreshed_target_xy = {
      refreshed_layout.cap_top.point.x,
      refreshed_layout.cap_top.point.y};
  std::vector<Point2> refreshed_occupied;
  refreshed_occupied.reserve(refreshed_layout.observed_cap_tops.size());
  for (std::size_t index = 0;
       index < refreshed_layout.observed_cap_tops.size();
       ++index) {
    if (refreshed_layout.grid_u[index] == 0 &&
        refreshed_layout.grid_v[index] == 0) {
      refreshed_occupied.push_back(refreshed_target_xy);
    } else {
      const auto& cap_top =
          refreshed_layout.observed_cap_tops[index];
      refreshed_occupied.push_back({cap_top.x, cap_top.y});
    }
  }
  const auto refreshed_selected_clearance = diagonal_clearance(
      refreshed_target_xy,
      refreshed_occupied,
      pre_rotation_diagonal->direction,
      desired_inner_gap_m,
      config_->finger_thickness_m,
      config_->finger_width_m,
      config_->cap_diameter_m,
      config_->tube_clearance_margin_m);
  const auto refreshed_diagonal = choose_grasp_diagonal(
      refreshed_target_xy,
      refreshed_occupied,
      {
          refreshed_layout.grid_axis_u.x,
          refreshed_layout.grid_axis_u.y,
      },
      {
          refreshed_layout.grid_axis_v.x,
          refreshed_layout.grid_axis_v.y,
      },
      desired_inner_gap_m,
      config_->finger_thickness_m,
      config_->finger_width_m,
      config_->cap_diameter_m,
      config_->tube_clearance_margin_m);
  if (!refreshed_selected_clearance ||
      *refreshed_selected_clearance <= 0.0 ||
      !refreshed_diagonal) {
    return Fail(TubeMissionStatus::kClearanceFailed);
  }
  const double refreshed_yaw_rad = std::atan2(
      refreshed_diagonal->direction.y,
      refreshed_diagonal->direction.x);
  if (grasp_line_yaw_change_requires_refresh(
          commanded_grasp_yaw_rad,
          refreshed_yaw_rad,
          config_->downward_axis_tolerance_deg) ||
      !refreshed_target_xy_shift_is_safe(
          pre_rotation_layout.cap_top.point.x,
          pre_rotation_layout.cap_top.point.y,
          refreshed_layout.cap_top.point.x,
          refreshed_layout.cap_top.point.y,
          config_->pitch_tolerance_m,
          config_->max_interframe_spread_m) ||
      !vertical_xy_path_within_tolerance(
          {
              refreshed_layout.cap_top.point.x,
              refreshed_layout.cap_top.point.y,
          },
          {{
              last_validated_pregrasp_tcp.position.x,
              last_validated_pregrasp_tcp.position.y,
          }},
          kMaximumVerticalXyDriftM)) {
    return Fail(TubeMissionStatus::kInconsistentTarget);
  }

  status = CommandAndWaitForAperture(*insertion_command);
  if (status != TubeMissionStatus::kSuccess) {
    return RecoverPreCloseFailure(
        status,
        last_validated_pregrasp_tcp,
        last_validated_pregrasp_z);
  }

  TubeObservation post_aperture_layout;
  status = DetectLayout(
      fixture_profile_id,
      target_cap_color,
      refreshed_layout,
      &post_aperture_layout,
      false,
      retained_layout);
  if (status != TubeMissionStatus::kSuccess) {
    return RecoverPreCloseFailure(
        status,
        last_validated_pregrasp_tcp,
        last_validated_pregrasp_z);
  }
  if (!ObservationIsFresh(post_aperture_layout)) {
    return RecoverPreCloseFailure(
        TubeMissionStatus::kStaleFrame,
        last_validated_pregrasp_tcp,
        last_validated_pregrasp_z);
  }
  const auto post_aperture_insertion_command = command_for_inner_gap(
      calibrated_gap_samples, desired_inner_gap_m);
  if (!post_aperture_insertion_command ||
      std::abs(
          *post_aperture_insertion_command - *insertion_command) >
          config_->gripper_command_tolerance) {
    return RecoverPreCloseFailure(
        TubeMissionStatus::kInsertionApertureFailed,
        last_validated_pregrasp_tcp,
        last_validated_pregrasp_z);
  }
  const std::array<double, 3> accepted_layout_target_point = {
      refreshed_layout.cap_top.point.x,
      refreshed_layout.cap_top.point.y,
      refreshed_layout.cap_top.point.z};
  const std::array<double, 3> post_aperture_target_point = {
      post_aperture_layout.cap_top.point.x,
      post_aperture_layout.cap_top.point.y,
      post_aperture_layout.cap_top.point.z};
  if (!same_target_observation(
          accepted_layout_target_point,
          post_aperture_target_point,
          config_->max_interframe_spread_m) ||
      !refreshed_target_xy_shift_is_safe(
          refreshed_layout.cap_top.point.x,
          refreshed_layout.cap_top.point.y,
          post_aperture_layout.cap_top.point.x,
          post_aperture_layout.cap_top.point.y,
          config_->pitch_tolerance_m,
          config_->max_interframe_spread_m)) {
    return RecoverPreCloseFailure(
        TubeMissionStatus::kInconsistentTarget,
        last_validated_pregrasp_tcp,
        last_validated_pregrasp_z);
  }

  const Point2 post_aperture_target_xy = {
      post_aperture_layout.cap_top.point.x,
      post_aperture_layout.cap_top.point.y};
  std::vector<Point2> post_aperture_occupied;
  post_aperture_occupied.reserve(
      post_aperture_layout.observed_cap_tops.size());
  for (std::size_t index = 0;
       index < post_aperture_layout.observed_cap_tops.size();
       ++index) {
    if (post_aperture_layout.grid_u[index] == 0 &&
        post_aperture_layout.grid_v[index] == 0) {
      post_aperture_occupied.push_back(post_aperture_target_xy);
    } else {
      const auto& cap_top =
          post_aperture_layout.observed_cap_tops[index];
      post_aperture_occupied.push_back({cap_top.x, cap_top.y});
    }
  }
  const auto post_aperture_diagonal = choose_grasp_diagonal(
      post_aperture_target_xy,
      post_aperture_occupied,
      {
          post_aperture_layout.grid_axis_u.x,
          post_aperture_layout.grid_axis_u.y,
      },
      {
          post_aperture_layout.grid_axis_v.x,
          post_aperture_layout.grid_axis_v.y,
      },
      desired_inner_gap_m,
      config_->finger_thickness_m,
      config_->finger_width_m,
      config_->cap_diameter_m,
      config_->tube_clearance_margin_m);
  const auto post_aperture_selected_clearance = diagonal_clearance(
      post_aperture_target_xy,
      post_aperture_occupied,
      pre_rotation_diagonal->direction,
      desired_inner_gap_m,
      config_->finger_thickness_m,
      config_->finger_width_m,
      config_->cap_diameter_m,
      config_->tube_clearance_margin_m);
  if (!post_aperture_diagonal ||
      !post_aperture_selected_clearance ||
      *post_aperture_selected_clearance <= 0.0) {
    return RecoverPreCloseFailure(
        TubeMissionStatus::kClearanceFailed,
        last_validated_pregrasp_tcp,
        last_validated_pregrasp_z);
  }
  const double post_aperture_yaw_rad = std::atan2(
      post_aperture_diagonal->direction.y,
      post_aperture_diagonal->direction.x);
  if (grasp_line_yaw_change_requires_refresh(
          commanded_grasp_yaw_rad,
          post_aperture_yaw_rad,
          config_->downward_axis_tolerance_deg) ||
      !vertical_xy_path_within_tolerance(
          {
              post_aperture_layout.cap_top.point.x,
              post_aperture_layout.cap_top.point.y,
          },
          {{
              last_validated_pregrasp_tcp.position.x,
              last_validated_pregrasp_tcp.position.y,
          }},
          kMaximumVerticalXyDriftM)) {
    return RecoverPreCloseFailure(
        TubeMissionStatus::kInconsistentTarget,
        last_validated_pregrasp_tcp,
        last_validated_pregrasp_z);
  }
  const double descent_aperture_command =
      *post_aperture_insertion_command;
  if (!ApertureEvidenceIsCurrent(descent_aperture_command)) {
    return RecoverPreCloseFailure(
        TubeMissionStatus::kInsertionApertureFailed,
        last_validated_pregrasp_tcp,
        last_validated_pregrasp_z);
  }

  const double fresh_cap_top_z =
      post_aperture_layout.cap_top.point.z;
  const double grasp_z =
      fresh_cap_top_z - config_->grasp_tcp_offset_below_cap_top_m;
  status = MoveVertical(
      last_validated_pregrasp_tcp,
      grasp_z,
      TubeMissionStatus::kDescentFailed,
      ArmMotion::Insertion,
      &post_aperture_layout,
      &descent_aperture_command);
  if (status != TubeMissionStatus::kSuccess) {
    return RecoverPreCloseFailure(
        status,
        last_validated_pregrasp_tcp,
        last_validated_pregrasp_z);
  }

  if (!EmitGripperEffect(TubeEffectKind::KnownEmpty)) return Fail(last_gripper_wait_status_);
  payload_ = PayloadKnowledge::kCloseIssuedUnknown;
  if (!EmitGripperEffect(TubeEffectKind::TransferBegun)) return Fail(last_gripper_wait_status_);
  if (!CommandGripper(config_->close_command, GripperMotion::FinalClose)) {
    return Fail(last_gripper_wait_status_);
  }
  if (!WaitForCaughtState()) {
    return Fail(last_gripper_wait_status_);
  }
  if (!EmitGripperEffect(TubeEffectKind::Captured)) return Fail(last_gripper_wait_status_);
  ROS_INFO("tube mission stage complete: grasp_closed");

  status = MoveVertical(
      last_validated_pregrasp_tcp,
      grasp_z + config_->lift_distance_m,
      TubeMissionStatus::kLiftFailed,
      ArmMotion::Extraction);
  if (status != TubeMissionStatus::kSuccess) {
    return Fail(status);
  }
  if (!HoldCaughtState()) {
    return Fail(last_gripper_wait_status_);
  }
  if (!EmitGripperEffect(TubeEffectKind::Retained) ||
      !RecordEffect(TubeEffectKind::ExtractedSafeClear, ArmEvidence())) return Fail(last_gripper_wait_status_);
  ROS_INFO("tube mission stage complete: extraction");
  status = MoveToPhaseOne(ArmMotion::ReturnWithTube, commanded_grasp_yaw_rad);
  if (status != TubeMissionStatus::kSuccess) {
    return Fail(status);
  }
  if (!HoldCaughtState()) {
    return Fail(last_gripper_wait_status_);
  }
  if (!EmitGripperEffect(TubeEffectKind::Retained)) return Fail(last_gripper_wait_status_);
  ROS_INFO("tube mission stage complete: phase_one_return");
  ROS_INFO("tube grasp succeeded; payload remains physically held");
  return TubeMissionStatus::kSuccess;
}


TubeMissionStatus TubeGraspMission::RecoverKnownEmptySearchFailure(TubeMissionStatus status) {
  if ((status != TubeMissionStatus::kTargetAbsent && status != TubeMissionStatus::kSearchExhausted) ||
      payload_ != PayloadKnowledge::kKnownEmpty) return status;
  if (!ContinueRunning()) return TubeMissionStatus::kInterrupted;
  if (!held_base_ || !held_base_()) return TubeMissionStatus::kBaseNotStopped;
  if (!EmitGripperEffect(TubeEffectKind::KnownEmpty)) return last_gripper_wait_status_;
  // The effect sink may change workflow state: recheck the held lease at the command boundary.
  if (!ContinueRunning()) return TubeMissionStatus::kInterrupted;
  if (!held_base_ || !held_base_()) return TubeMissionStatus::kBaseNotStopped;
  const auto recovery = MoveToPhaseOne(ArmMotion::Recovery);
  if (recovery != TubeMissionStatus::kSuccess) return recovery;
  if (!ContinueRunning()) return TubeMissionStatus::kInterrupted;
  if (!held_base_ || !held_base_()) return TubeMissionStatus::kBaseNotStopped;
  if (!EmitGripperEffect(TubeEffectKind::KnownEmpty) ||
      !RecordEffect(TubeEffectKind::EmptySafeClear, ArmEvidence())) return last_gripper_wait_status_;
  ROS_WARN("target search failed; verified empty arm returned to the measured phase-one endpoint");
  return status;
}

TubeMissionStatus TubeGraspMission::Fail(TubeMissionStatus status) {
  RefreshPayloadEvidence();
  status = RecoverKnownEmptySearchFailure(status);
  if (move_group_) {
    move_group_->stop();
    move_group_->clearPoseTargets();
  }
  if (payload_ == PayloadKnowledge::kKnownEmpty) {
    ROS_WARN("tube mission failed while payload was known empty");
  } else {
    ROS_ERROR(
        "tube mission failed with payload unknown or held; "
        "keeping the gripper command and arm pose unchanged");
  }
  return status;
}

}  // namespace agxcore
