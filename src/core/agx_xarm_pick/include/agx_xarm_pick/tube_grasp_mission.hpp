#ifndef AGX_XARM_PICK_TUBE_GRASP_MISSION_HPP
#define AGX_XARM_PICK_TUBE_GRASP_MISSION_HPP

#include <chrono>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <agx_pick_msg/DetectTubeCap.h>
#include <agx_pick_msg/DetectTubeCapConfigured.h>
#include <bt_task_msgs/LiftMotorMsg.h>
#include <bt_task_msgs/LiftMotorSrv.h>
#include <dh_gripper_msgs/GripperCtrl.h>
#include <dh_gripper_msgs/GripperState.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "agx_xarm_pick/tube_grasp_contract.hpp"
#include "agx_xarm_pick/arm_execution.hpp"
#include "agx_xarm_pick/visual_alignment_motion.hpp"
#include "agx_xarm_pick/tube_lift_contract.hpp"
#include "agx_xarm_pick/tube_preparation_contract.hpp"
#include "agx_xarm_pick/tube_search_contract.hpp"
#include "agx_xarm_pick/visual_alignment.hpp"
#include "agx_xarm_pick/tube_view_region.hpp"
#include "agx_xarm_pick/resolved_tube_config.hpp"
#include "agx_xarm_pick/tube_effects.hpp"
#include "agx_xarm_pick/tube_live_observer.hpp"

namespace agxcore {

enum class GripperMotion { Preclose, FinalClose, Recovery };

class TubeGraspMission {
 public:
  explicit TubeGraspMission(ros::NodeHandle* node);
  TubeGraspMission(ros::NodeHandle* node,
      std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig> config,
      const std::string& target_cap_color, TubeEffectSink effect_sink = {},
      std::function<bool()> held_base = {});
  ~TubeGraspMission();
  TubeGraspMission(const TubeGraspMission&) = delete;
  TubeGraspMission& operator=(const TubeGraspMission&) = delete;
  TubeMissionStatus Run(
      const std::string& fixture_profile_id,
      const std::string& target_cap_color);
  TubeMissionStatus RunSearchOnly(
      const std::string& fixture_profile_id,
      const std::string& target_cap_color);
  // Safe from the monitoring thread while Run is waiting in a child API.
  // Stop requests are permanent for this one mission instance.
  void RequestStop();
  // Called after the mission worker joins. Coalesce a final stop behind any
  // earlier RPC, then join within the caller's remaining stop budget.
  bool FinishStopRequests(double timeout_s);
  bool StopRequested() const { return stop_requested_.load(); }
  // Read-only monitoring can outlive the worker and its MoveIt object.
  std::shared_ptr<TubeLiveObserver> Observer() const { return observer_; }
  void RefreshPayloadEvidence();
  bool ObserveStopped(const TubeEvidence& stop_request);
  TubeMissionOutcome Outcome(TubeMissionStatus status);

 private:
  void RequestStopInternal(bool final);
  void LaunchLiftStopLocked();
  void ReapLiftStopLocked();
  bool ContinueRunning() const { return ros::ok() && !StopRequested(); }
  struct TubeObservation {
    geometry_msgs::PointStamped cap_top;
    std::vector<geometry_msgs::Point> observed_cap_tops;
    std::vector<std::int32_t> grid_u;
    std::vector<std::int32_t> grid_v;
    geometry_msgs::Vector3 grid_axis_u;
    geometry_msgs::Vector3 grid_axis_v;
    double target_u_px = 0.0;
    double target_v_px = 0.0;
    double principal_u_px = 0.0;
    double principal_v_px = 0.0;
    double lattice_rms_error_m = 0.0;
  };
  struct SearchResult {
    TubeMissionStatus status = TubeMissionStatus::kSearchExhausted;
    bool found = false;
    TubeObservation observation;
  };
  struct RobotStateSnapshot {
    bool valid = false;
    ros::Time source_stamp;
    std::chrono::steady_clock::time_point receipt_time;
    sensor_msgs::JointState message;
    std::vector<double> ordered_positions;
  };
  struct LiftOperation {
    agx_xarm_pick::LiftTarget target{0, 0.0};
    std::chrono::steady_clock::time_point command_time{};
    std::uint64_t feedback_epoch_at_start = 0;
  };

  bool WaitForKnownOpenState();
  bool RecordEffect(TubeEffectKind kind, const TubeEvidence& evidence);
  bool EmitGripperEffect(TubeEffectKind kind);
  TubeEvidence ArmEvidence() const;
  bool WaitForGripperPosition(double target_position);
  bool ApertureEvidenceIsCurrent(double target_position) const;
  bool WaitForCaughtState();
  bool HoldCaughtState();
  bool WaitForSynchronizedRobotState(
      std::chrono::steady_clock::time_point minimum_receipt_time,
      std::chrono::steady_clock::time_point deadline,
      moveit::core::RobotStatePtr* synchronized_state) const;
  TubeMissionStatus MoveTopDown(
      agx_xarm_pick::ArmMotion motion,
      const agx_xarm_pick::TopDownViewRegion& region,
      double desired_y_m,
      double minimum_clearance_m,
      TubeMissionStatus plan_failure,
      TubeMissionStatus execution_failure,
      geometry_msgs::Pose* measured_tcp = nullptr,
      const TubeObservation* required_fresh_observation = nullptr);
  TubeMissionStatus ValidateAndPrepare(
      const std::string& fixture_profile_id,
      const std::string& target_cap_color);
  TubeMissionStatus RunVisualAlignmentUnlocked(
      const std::string& fixture_profile_id,
      agx_xarm_pick::VisualAlignmentResult* result);
  agx_xarm_pick::MoveItVisualAlignmentMotionConfig
      BuildAlignmentMotionConfig(agx_xarm_pick::ArmMotion motion) const;
  agx_xarm_pick::ArmExecutionScales ArmScales(agx_xarm_pick::ArmMotion motion) const;
  void ApplyArmExecution(agx_xarm_pick::ArmMotion motion);
  bool ReadAlignmentState(
      std::chrono::steady_clock::time_point minimum_receipt_time,
      std::chrono::steady_clock::time_point deadline,
      moveit::core::RobotStatePtr* state,
      geometry_msgs::Pose* grasp_tcp_pose) const;
  bool VisualTargetIsFresh(
      const agx_xarm_pick::VisualTargetObservation& observation) const;
  TubeMissionStatus StartLift(LiftOperation* operation);
  TubeMissionStatus FinishLift(const LiftOperation& operation);
  TubeMissionStatus MoveToPhaseOne(
      agx_xarm_pick::ArmMotion motion,
      std::optional<double> preserved_tool_yaw_rad = std::nullopt);
  TubeMissionStatus ResetPhaseOneMoveItTarget(
      TubeMissionStatus primary_status,
      bool stop_motion,
      const moveit::core::RobotStatePtr& synchronized_state =
          moveit::core::RobotStatePtr());
  TubeMissionStatus PrepareThroughPhaseOne();
  void ResolveLiftedGeometry();
  SearchResult PrepareAndSearch(
      const std::string& fixture_profile_id,
      const std::string& target_cap_color);
  bool CurrentTcpPose(
      std::chrono::steady_clock::time_point minimum_receipt_time,
      std::chrono::steady_clock::time_point deadline,
      geometry_msgs::Pose* tcp_pose,
      moveit::core::RobotStatePtr* synchronized_state = nullptr) const;
  bool TcpPoseFromState(
      const moveit::core::RobotState& state,
      geometry_msgs::Pose* tcp_pose) const;
  bool ValidateExecutedTcp(
      const geometry_msgs::Point& requested_tcp,
      const geometry_msgs::Pose& measured_tcp) const;
  bool CameraAxisIsDownwardAtTcpPose(
      const geometry_msgs::Pose& tcp_pose) const;
  bool CameraAxisIsDownwardAtTcpPose(
      const geometry_msgs::Pose& tcp_pose,
      double tolerance_deg) const;
  bool ObservationIsFresh(
      const TubeObservation& observation) const;
  bool BaseStoppedIfAvailable() const;
  bool ValidateMeasuredTarget(
      const geometry_msgs::PointStamped& cap_top) const;
  geometry_msgs::Pose TcpPoseToLink6(
      const geometry_msgs::Pose& tcp_pose) const;
  TubeMissionStatus DetectObservation(
      const std::string& fixture_profile_id,
      const std::string& target_cap_color,
      std::uint8_t mode,
      const ros::Time& not_before,
      TubeObservation* observation);
  TubeMissionStatus DetectLayout(
      const std::string& fixture_profile_id,
      const std::string& target_cap_color,
      const TubeObservation& centered_target,
      TubeObservation* observation,
      bool require_centered_reference = true,
      const TubeObservation* stationary_layout = nullptr);
  TubeMissionStatus AlignTcpOverTarget(
      const TubeObservation& observation);
  TubeMissionStatus ExecuteGraspFromCenteredTarget(
      const std::string& fixture_profile_id,
      const std::string& target_cap_color,
      const TubeObservation& centered_target,
      const TubeObservation* stationary_layout = nullptr);
  TubeMissionStatus RotateTcpAtPregrasp(
      const geometry_msgs::Pose& validated_pregrasp,
      double selected_yaw,
      const TubeObservation& required_fresh_observation);
  TubeMissionStatus CorrectTcpPlanar(
      const TubeObservation& required_fresh_observation,
      double correction_x_m,
      double correction_y_m,
      geometry_msgs::Pose* measured_tcp);
  TubeMissionStatus CommandAndWaitForAperture(
      double insertion_command);
  TubeMissionStatus MoveVertical(
      const geometry_msgs::Pose& start,
      double target_z,
      TubeMissionStatus failure,
      agx_xarm_pick::ArmMotion motion,
      const TubeObservation* required_fresh_observation = nullptr,
      const double* required_aperture_command = nullptr);
  TubeMissionStatus RecoverPreCloseFailure(
      TubeMissionStatus primary_failure,
      const geometry_msgs::Pose& validated_pregrasp,
      double pregrasp_z);
  TubeMissionStatus SettleForObservation(ros::Time* not_before) const;
  TubeMissionStatus Fail(TubeMissionStatus status);
  TubeMissionStatus RecoverKnownEmptySearchFailure(TubeMissionStatus status);
  bool CommandGripper(double position, GripperMotion motion);
  void GripperStateCallback(
      const dh_gripper_msgs::GripperState::ConstPtr& message);
  void JointStateCallback(
      const sensor_msgs::JointState::ConstPtr& message);
  void LiftStateCallback(
      const bt_task_msgs::LiftMotorMsg::ConstPtr& message);

  ros::NodeHandle* node_;
  ros::ServiceClient detector_client_;
  ros::ServiceClient lift_client_;
  ros::ServiceClient lift_stop_client_;
  ros::Publisher arm_stop_publisher_;
  std::atomic<bool> stop_requested_{false};
  std::shared_ptr<std::atomic<bool>> lift_stop_sent_{std::make_shared<std::atomic<bool>>(false)};
  std::mutex stop_rpc_mutex_;
  std::thread lift_stop_worker_;
  bool stop_rpcs_sealed_ = false, lift_stop_retry_ = false, stop_rpc_failed_ = false;
  std::chrono::steady_clock::time_point stop_rpc_deadline_ =
      std::chrono::steady_clock::time_point::max();
  ros::Publisher gripper_publisher_;
  ros::Subscriber gripper_subscriber_;
  ros::Subscriber joint_state_subscriber_;
  ros::Subscriber lift_state_subscriber_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig> resolved_config_;
  std::shared_ptr<const agx_xarm_pick::TubeTaskMotionConfig> config_;
  std::shared_ptr<TubeLiveObserver> observer_;
  TubeEffectJournal effects_;
  const std::function<bool()> held_base_;
  mutable std::mutex effects_state_mutex_;
  std::mutex mission_mutex_;
  mutable std::mutex gripper_mutex_;
  mutable std::mutex joint_state_mutex_;
  mutable std::mutex lift_mutex_;
  dh_gripper_msgs::GripperState latest_gripper_state_;
  std::chrono::steady_clock::time_point latest_gripper_wall_time_;
  std::chrono::steady_clock::time_point gripper_command_wall_time_;
  RobotStateSnapshot latest_robot_state_snapshot_;
  bt_task_msgs::LiftMotorMsg latest_lift_state_;
  std::chrono::steady_clock::time_point latest_lift_wall_time_;
  bool has_gripper_state_ = false;
  bool has_lift_state_ = false;
  std::uint64_t lift_feedback_epoch_ = 0;
  std::uint64_t latest_lift_fault_epoch_ = 0;
  std::chrono::steady_clock::time_point latest_lift_fault_wall_time_{};
  std::optional<agx_xarm_pick::LiftTarget> lift_target_;
  double measured_lift_m_ = 0.0;
  double runtime_cap_top_z_m_ = 0.0;
  bool runtime_geometry_resolved_ = false;
  std::atomic<TubeMissionStatus> last_gripper_wait_status_{TubeMissionStatus::kGripperStateUnavailable};
  std::atomic<PayloadKnowledge> payload_{PayloadKnowledge::kUnknown};
};

}  // namespace agxcore

#endif  // AGX_XARM_PICK_TUBE_GRASP_MISSION_HPP
