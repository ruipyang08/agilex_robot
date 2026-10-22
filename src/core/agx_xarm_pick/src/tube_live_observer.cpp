#include "agx_xarm_pick/tube_live_observer.hpp"
#include "agx_xarm_pick/resolved_tube_config.hpp"
#include "agx_xarm_pick/tube_lift_contract.hpp"
#include <chrono>
#include <algorithm>
#include <ros/ros.h>
#include <dh_gripper_msgs/GripperState.h>
#include <sensor_msgs/JointState.h>
#include <bt_task_msgs/LiftMotorMsg.h>

namespace agxcore {
namespace {
double SteadyNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
class RosTubeLiveObserver final : public TubeLiveObserver {
 public:
  RosTubeLiveObserver(ros::NodeHandle node, const TubeObserverInterfaces& interfaces,
      const TubeObserverConfig& config) : buffer_(config) {
    ValidateTubeObserverInterfaces(interfaces);
    gripper_ = node.subscribe<dh_gripper_msgs::GripperState>(interfaces.gripper_state_topic, 5,
      [this, topic = interfaces.gripper_state_topic](const dh_gripper_msgs::GripperState::ConstPtr& msg) {
        TubeGripperObservation value;
        value.evidence = {topic, msg->header.stamp.toSec(), SteadyNow()};
        value.valid = true; value.initialized = msg->is_initialized; value.grip_state = msg->grip_state;
        value.position = msg->position; value.target_position = msg->target_position;
        buffer_.Gripper(value);
      });
    arm_ = node.subscribe<sensor_msgs::JointState>(interfaces.joint_state_topic, 5,
      [this, topic = interfaces.joint_state_topic](const sensor_msgs::JointState::ConstPtr& msg) {
        TubeArmObservation value;
        value.evidence = {topic, msg->header.stamp.toSec(), SteadyNow()}; value.valid = true;
        for (std::size_t i = 0; i < 6; ++i) {
          const auto name = "joint" + std::to_string(i + 1);
          const auto found = std::find(msg->name.begin(), msg->name.end(), name);
          const auto index = static_cast<std::size_t>(found - msg->name.begin());
          if (found == msg->name.end() || index >= msg->position.size() || index >= msg->velocity.size() ||
              std::count(msg->name.begin(), msg->name.end(), name) != 1) { value.valid = false; continue; }
          value.positions[i] = msg->position[index]; value.velocities[i] = msg->velocity[index];
        }
        buffer_.Arm(value);
      });
    lift_ = node.subscribe<bt_task_msgs::LiftMotorMsg>(interfaces.lift_state_topic, 5,
      [this, topic = interfaces.lift_state_topic](const bt_task_msgs::LiftMotorMsg::ConstPtr& msg) {
        TubeLiftObservation value;
        value.evidence = {topic, msg->header.stamp.toSec(), SteadyNow()};
        value.valid = msg->initState && agx_xarm_pick::lift_feedback_is_fault_free(
            msg->overflowI, msg->overflowV, msg->encodeErr, msg->posBiasOver,
            msg->underVol, msg->overLoad, msg->externalCtrlMode, false, msg->fpsError);
        // A commanded stopFlag inhibits movement; it does not invalidate the
        // independently measured speed/position used for stop acknowledgement.
        value.stationary = !msg->runState && msg->backSpeed == 0;
        value.position_m = static_cast<double>(msg->backHeight) * .001;
        value.velocity_mps = static_cast<double>(msg->backSpeed) * .001;
        buffer_.Lift(value);
      });
  }
  ~RosTubeLiveObserver() override {
    gripper_.shutdown(); arm_.shutdown(); lift_.shutdown();
  }
  TubeLiveSnapshot Snapshot() const override {
    return buffer_.SnapshotAfterClockCatchUp([] { return ros::Time::now().toSec(); }, SteadyNow,
        .05, .25);
  }
  TubeMotionWindowToken ArmMotionWindow() override {
    return buffer_.ArmMotionWindow([] { return ros::Time::now().toSec(); }, SteadyNow);
  }
 private:
  TubeObservationBuffer buffer_;
  ros::Subscriber gripper_, arm_, lift_;
};
}
std::shared_ptr<TubeLiveObserver> MakeTubeLiveObserver(ros::NodeHandle node,
    const agx_xarm_pick::ResolvedTubeTaskConfig& config,
    const agx_xarm_pick::TubeTaskMotionConfig& motion) {
  return MakeTubeLiveObserver(std::move(node),
      {config.robot.joint_state_topic, config.robot.gripper_state_topic, motion.lift_state_topic},
      {motion.gripper_state_max_age_sec, motion.robot_state_max_age_sec,
       motion.lift_state_max_age_sec, motion.fully_open_command, motion.close_command,
       motion.gripper_command_tolerance});
}
std::shared_ptr<TubeLiveObserver> MakeTubeLiveObserver(ros::NodeHandle node,
    const TubeObserverInterfaces& interfaces, const TubeObserverConfig& config) {
  return std::make_shared<RosTubeLiveObserver>(std::move(node), interfaces, config);
}
}  // namespace agxcore
