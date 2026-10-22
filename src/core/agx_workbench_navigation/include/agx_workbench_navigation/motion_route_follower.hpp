#pragma once
#include <agx_workbench_navigation/motion_route.hpp>
namespace agx_workbench_navigation {
enum class FollowerState {BrakeToStop,SettleMode,FollowForward,FollowSpin,CorrectSpin,VerifyEndpoint,Blocked};
struct FollowerObservation {
  MotionPose pose;double forward_mps=0,lateral_mps=0,yaw_radps=0,source_stamp_s=0;
  std::uint64_t revision=0;
};
struct FollowerCommand {
  bool ok=false,complete=false;double vx_mps=0,wz_radps=0;
  FollowerState state=FollowerState::Blocked;std::size_t segment=0,sample=0;
  std::string reason;
  bool correcting=false;double correction_target_yaw_rad=0;unsigned correction_attempts=0;
};
class MotionRouteFollower {
 public:
  MotionRouteFollower(MotionRoute route,MotionLimits limits,double tracking_bound_m,
      double curvature_tracking_slowdown_per_m=.25,double goal_position_tolerance_m=.005,
      double goal_yaw_tolerance_rad=.015,std::size_t first_segment=0);
  FollowerCommand Tick(const FollowerObservation&,double source_now_s);
  // Remember the command actually accepted by the producer limiter, so the
  // next request cannot accumulate an unexecuted speed or curvature ramp.
  void AcceptLimitedCommand(double vx_mps,double wz_radps);
  void Block(const std::string& reason);
 private:
  MotionRoute route_;MotionLimits limits_;double tracking_bound_,curvature_tracking_slowdown_;
  double goal_position_tolerance_;
  double goal_yaw_tolerance_;
  FollowerState state_=FollowerState::BrakeToStop;
  std::size_t segment_=0,sample_=0;
  double last_now_=-1,stopped_since_=-1,settle_since_=-1,previous_v_=0,previous_w_=0;
  bool finishing_segment_=false;
  unsigned correction_attempts_=0;double correction_began_s_=-1;
  FollowerObservation last_observation_;
  double previous_curvature_=0;
  std::string reason_;
};
}
