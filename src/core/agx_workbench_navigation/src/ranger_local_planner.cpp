#include <agx_workbench_navigation/ranger_local_planner.hpp>
#include <agx_workbench_navigation/motion_route_store.hpp>
#include <agx_workbench_navigation/motion_command_sweep.hpp>
#include <agx_workbench_navigation/motion_speed_profile.hpp>
#include <agx_workbench_navigation/motion_evidence_ros.hpp>
#include <agx_workbench_navigation/rover_execution.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include <angles/angles.h>
#include <boost/thread/locks.hpp>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace agx_workbench_navigation {
namespace {

template <typename T>
void requireParam(const ros::NodeHandle& nh, const std::string& name, T* value) {
  if (!nh.getParam(name, *value)) {
    throw std::runtime_error("missing required RangerLocalPlanner parameter: " + name);
  }
}

double requireNumberParam(const ros::NodeHandle& nh,const std::string& name) {
  XmlRpc::XmlRpcValue value;requireParam(nh,name,&value);
  if(value.getType()==XmlRpc::XmlRpcValue::TypeInt)return static_cast<int>(value);
  if(value.getType()==XmlRpc::XmlRpcValue::TypeDouble)return static_cast<double>(value);
  throw std::runtime_error("RangerLocalPlanner requires a numeric parameter: "+name);
}

void requireFinitePositive(double value, const std::string& name) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(
        "RangerLocalPlanner parameter must be finite and positive: " + name);
  }
}

RangerControlDecision makeBlockedDecision(const std::string& reason) {
  return RangerControlDecision{
      PlannerState::BLOCKED, 0.0, 0.0, 0.0, 0.0, reason};
}

double steadyNowS() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

CommandGoalEndpoint commandEndpoint(const geometry_msgs::PoseStamped& pose) {
  const auto& p=pose.pose.position;const auto& q=pose.pose.orientation;
  return {pose.header.frame_id,pose.header.stamp.toNSec(),{p.x,p.y,p.z},{q.x,q.y,q.z,q.w}};
}

}  // namespace

RangerLocalPlanner::RangerLocalPlanner()
    : initialized_(false),
      tf_(nullptr),
      costmap_ros_(nullptr),
      last_goal_available_(false) {}

void RangerLocalPlanner::initialize(
    std::string name,
    tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("RangerLocalPlanner has already been initialized");
    return;
  }
  if (tf == nullptr || costmap_ros == nullptr) {
    throw std::invalid_argument("RangerLocalPlanner requires TF and costmap handles");
  }

  ros::NodeHandle private_nh("~/" + name);
  try {
    requireParam(private_nh,"state_topic",&config_.state_topic);
    requireParam(private_nh,"local_plan_topic",&config_.local_plan_topic);
    config_.rate_limit_max_dt_s=requireNumberParam(private_nh,"rate_limit_max_dt_s");
    config_.xy_goal_tolerance_m=requireNumberParam(private_nh,"xy_goal_tolerance_m");
    config_.yaw_goal_tolerance_rad=requireNumberParam(private_nh,"yaw_goal_tolerance_rad");
    config_.curvature_tracking_slowdown_per_m=requireNumberParam(private_nh,"curvature_tracking_slowdown_per_m");
    requireFinitePositive(config_.rate_limit_max_dt_s,"rate_limit_max_dt_s");
    requireFinitePositive(config_.xy_goal_tolerance_m,"xy_goal_tolerance_m");
    requireFinitePositive(config_.yaw_goal_tolerance_rad,"yaw_goal_tolerance_rad");
    requireFinitePositive(config_.curvature_tracking_slowdown_per_m,"curvature_tracking_slowdown_per_m");
    config_.progress_timeout_s=requireNumberParam(private_nh,"progress_timeout_s");
    config_.progress_min_translation_m=requireNumberParam(private_nh,"progress_min_translation_m");
    config_.progress_min_rotation_rad=requireNumberParam(private_nh,"progress_min_rotation_rad");
    // Startup-requested value only: private move_base dynamic-reconfigure state
    // is not observable through getParam. The local guard is always required.
    config_.stock_oscillation_timeout_requested_s=requireNumberParam(ros::NodeHandle("~"),"oscillation_timeout");
    if(config_.stock_oscillation_timeout_requested_s!=0)
      throw std::runtime_error("RangerLocalPlanner requires startup oscillation_timeout=0 with its finite progress guard");
    navigation_progress_.reset(new NavigationProgressGuard(config_.progress_timeout_s,
        config_.progress_min_translation_m,config_.progress_min_rotation_rad));

    XmlRpc::XmlRpcValue execution_policy;
    if (ros::param::get("/rover_execution", execution_policy)) {
      effective_rover_execution_=snapshotRoverExecution(execution_policy);
      const auto scales=resolveRoverScales(execution_policy,"coarse_navigation");
      if(scales.first!=1.0 || scales.second!=1.0)
        throw std::runtime_error("non-unit coarse_navigation scaling is unsupported by admitted motion routes");
    }
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("RangerLocalPlanner configuration error: " << error.what());
    throw;
  }

  std::string policy_path;
  requireParam(private_nh,"motion_policy_file",&policy_path);
  {
    route_policy_.reset(new InstalledMotionPolicy(LoadInstalledMotionPolicy(policy_path)));
    std::string mode;requireParam(private_nh,"operation_mode",&mode);
    if(route_policy_->execution_enabled && mode!="simulation")
      throw std::runtime_error("experimental navigation policy requires simulation mode");
    const auto& limits=route_policy_->preflight.limits;
    // One installed limit set controls both the route and the final source slew.
    config_.max_linear_speed_mps=limits.max_speed_mps;
    config_.max_linear_accel_mps2=limits.acceleration_mps2;
    config_.max_linear_decel_mps2=limits.deceleration_mps2;
    config_.max_spin_speed_radps=limits.max_yaw_rate_radps;
    config_.max_angular_accel_radps2=limits.angular_acceleration_radps2;
    config_.max_lateral_accel_mps2=limits.lateral_acceleration_mps2;
    config_.max_curvature_rate_per_m_s=limits.curvature_rate_per_m_s;
    config_.min_turn_radius_m=limits.min_turn_radius_m;
  }
  tf_ = tf;
  ROS_INFO_STREAM("Ranger execution limits: forward=" << config_.max_linear_speed_mps
      << " m/s, spin=" << config_.max_spin_speed_radps
      << " rad/s, acceleration=" << config_.max_linear_accel_mps2 << " m/s^2");
  costmap_ros_ = costmap_ros;
  command_limiter_.reset(new RangerCommandLimiter(config_));
  const auto body_frame=costmap_ros_->getBaseFrameID();
  if(body_frame!="base_link"&&body_frame!="dummy_base_link")
    throw std::runtime_error("route execution requires base_link or its identity alias");
  state_publisher_ = private_nh.advertise<CoarseNavigationState>(
      config_.state_topic, 10, true);
  local_plan_publisher_ = private_nh.advertise<nav_msgs::Path>(
      config_.local_plan_topic, 1, true);
  authority_subscriber_=private_nh.subscribe("/workbench_nav/authority_state",10,
      &RangerLocalPlanner::authorityCallback,this);
  initialized_ = true;
  authorized_publisher_=private_nh.advertise<AuthorizedBaseCommand>(
      "/workbench_nav/cmd_vel/navigation",10);
  binding_service_=private_nh.advertiseService("/workbench_nav/bind_navigation_source",
      &RangerLocalPlanner::bindSource,this);
  effective_config_session_=NewEffectivePlannerSession();
  effective_config_publisher_=private_nh.advertise<std_msgs::String>(
      "/workbench_nav/effective_config/local_planner",1,true);
  publishEffectiveConfiguration(ros::WallTimerEvent());
  effective_config_timer_=private_nh.createWallTimer(ros::WallDuration(.2),
      &RangerLocalPlanner::publishEffectiveConfiguration,this);
}

void RangerLocalPlanner::publishEffectiveConfiguration(const ros::WallTimerEvent&) {
  if(!initialized_ || !costmap_ros_) return;
  try {
    // This WallTimer shares stock Noetic move_base's single-threaded global
    // callback queue with footprint/reconfigure setters. That serialization
    // protects their vector; the costmap mutex alone does not protect setters.
    // Loaded controller limits/collision geometry remain immutable.
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap_ros_->getCostmap()->getMutex());
    const auto live=EffectivePlannerFootprint(costmap_ros_->getRobotFootprint());
    std_msgs::String report;
    report.data=LocalPlannerEffectiveReport(effective_config_session_,config_,
        effective_rover_execution_,live,route_policy_->preflight.geometry.footprint);
    effective_config_publisher_.publish(report);
  } catch(const std::exception& error) {
    ROS_ERROR_STREAM_THROTTLE(1.0,"Cannot report local planner effective configuration: " << error.what());
  }
}

bool RangerLocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) return false;
  if (plan.empty()) {
    command_gate_.InvalidatePlan([this]{
      global_plan_.clear();command_limiter_->reset();
    });
    return false;
  }
  bool accepted=false;
  const auto disposition=command_gate_.DeliverPlan(commandEndpoint(plan.back()),[&]{
    accepted=setCurrentPlan(plan);
  });
  // Acknowledge only the superseded delivery so stock move_base cannot abort
  // the current operation for surfacing an old pending plan. Nothing installed.
  return disposition==PlanDisposition::SupersededDelivery ||
      (disposition==PlanDisposition::Installed && accepted);
}

bool RangerLocalPlanner::bindSource(BindBaseCommandSource::Request& request,
                                   BindBaseCommandSource::Response& response) {
  const CommandBinding binding{request.session_id,request.grant_id,request.source_goal_id,
      commandEndpoint(request.expected_goal),request.invocation_id,request.route_id,request.task_deadline_s};
  response.accepted=initialized_ && command_gate_.BindValidated(binding,[&] {
    const auto route=SharedMotionRouteStore().get(binding.route_id,binding.invocation_id);
    return route_policy_->execution_enabled && route &&
        route->model_digest==route_policy_->preflight.model_digest && SharedMotionRouteStore().bind(binding);
  },[this,&binding] {
    route_follower_.reset();route_observation_={};route_complete_=false;route_binding_=binding;
    {
      const auto route=SharedMotionRouteStore().bound(binding);
      std::size_t first_segment=0;
      SharedMotionRouteStore().boundGoal(binding.endpoint,&first_segment);
      if(route)route_follower_.reset(new MotionRouteFollower(*route,route_policy_->preflight.limits,.02,
          config_.curvature_tracking_slowdown_per_m,std::min(.02,config_.xy_goal_tolerance_m),
          std::min(.15,config_.yaw_goal_tolerance_rad),first_segment));
    }
    global_plan_.clear();last_goal_available_=false;
    plan_receipt_time_=ros::Time();command_limiter_->reset();
    navigation_progress_->Begin(ros::Time::now().toSec(),steadyNowS());
    resolved_progress_pose_={};progress_reason_.clear();
  });
  if (response.accepted) {
    response.accepted=command_gate_.Compute([&](const CommandBinding& current,std::uint64_t sequence,bool) {
      if (current.session_id!=binding.session_id || current.grant_id!=binding.grant_id ||
          current.source_goal_id!=binding.source_goal_id) return false;
      publishAuthorized(current,sequence,ros::Time::now(),geometry_msgs::Twist());
      return true;
    });
  }
  response.reason=response.accepted?"producer bound; waiting for matching new plan":
      "invalid, stale or superseded producer binding";
  return true;
}

void RangerLocalPlanner::publishAuthorized(const CommandBinding& binding,std::uint64_t sequence,
    const ros::Time& generated_at,const geometry_msgs::Twist& command) {
  AuthorizedBaseCommand message;
  message.header.stamp=generated_at;
  message.session_id=binding.session_id;message.grant_id=binding.grant_id;
  message.source_goal_id=binding.source_goal_id;message.sequence=sequence;
  message.twist=command;authorized_publisher_.publish(message);
}

bool RangerLocalPlanner::setCurrentPlan(
    const std::vector<geometry_msgs::PoseStamped>& global_plan) {
  if (!initialized_) {
    ROS_ERROR("RangerLocalPlanner::setPlan called before initialize");
    return false;
  }
  if (global_plan.empty()) {
    global_plan_.clear();
    return false;
  }

  const geometry_msgs::PoseStamped& goal = global_plan.back();
  bool goal_changed = !last_goal_available_;
  if (last_goal_available_) {
    const double position_delta = std::hypot(
        goal.pose.position.x - last_goal_.pose.position.x,
        goal.pose.position.y - last_goal_.pose.position.y);
    const double yaw_delta = std::abs(angles::shortest_angular_distance(
        tf2::getYaw(last_goal_.pose.orientation),
        tf2::getYaw(goal.pose.orientation)));
    goal_changed = goal.header.frame_id != last_goal_.header.frame_id ||
        position_delta > 0.01 || yaw_delta > 0.01;
  }
  if (goal_changed) {

    command_limiter_->reset();
  }
  global_plan_ = global_plan;
  // Identity was checked against the original map goal above. The controller
  // needs today's map-to-base transform, not a transform at goal-dispatch time.
  // This private geometry copy is never used to mint command authorization.
  for (auto& pose:global_plan_) pose.header.stamp=ros::Time(0);
  last_goal_ = goal;
  last_goal_available_ = true;
  plan_receipt_time_ = ros::Time::now();
  return true;
}

bool RangerLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  cmd_vel=geometry_msgs::Twist();
  return command_gate_.Compute([&](const CommandBinding& binding,std::uint64_t sequence,bool ready) {
    const auto generated_at=ros::Time::now();
    bool ok=ready && computeRouteCommand(cmd_vel,binding,generated_at);
    // Computation cannot extend the original operation deadline.
    if(ok && !checkNavigationProgress(binding,ros::Time::now()))ok=false;
    if(!ok)cmd_vel=geometry_msgs::Twist();
    publishAuthorized(binding,sequence,generated_at,cmd_vel);
    if(!ok) {
      // Diagnostics follow owned zero; a diagnostic exception cannot suppress it.
      try {publishDecision(makeBlockedDecision(progress_reason_),false,false,false,0);} catch(...) {}
    }
    return ok;
  });
}

bool RangerLocalPlanner::computeRouteCommand(geometry_msgs::Twist& command,
    const CommandBinding& binding,const ros::Time& now) {
  const auto fail=[&](const std::string& reason) {
    if(route_follower_)route_follower_->Block(reason);
    SharedMotionRouteStore().block(binding.route_id,binding.invocation_id,reason);
    command=geometry_msgs::Twist();route_complete_=false;command_limiter_->reset();
    progress_reason_=reason;
    return false;
  };
  const auto route=SharedMotionRouteStore().bound(binding);
  const auto evidence=SharedMotionEvidence();const double steady=steadyNowS();
  if(!route || !route_follower_) {
    const auto reason=SharedMotionRouteStore().blocked(binding);
    return fail(reason.empty()?"active_route_expired_or_missing":reason);
  }
  if(!evidence || !evidence->unavailable_detail.empty() || evidence->observed_at_s>steady ||
      evidence->pose_deadline_s<=steady || evidence->obstacle_deadline_s<=steady ||
      route->model_digest!=route_policy_->preflight.model_digest ||
      route->map_digest!=evidence->static_map_digest || route->registry_digest!=evidence->registry_digest)
    return fail("route_sources_stale_or_changed"+std::string(
        evidence && !evidence->unavailable_detail.empty()?": "+evidence->unavailable_detail:""));
  if(!checkNavigationProgress(binding,now))return fail(progress_reason_);
  CanonicalMotionProgress motion;
  {std::lock_guard<std::mutex> lock(evidence_mutex_);
    motion=motion_feedback_.currentAtRead(binding,now.toSec(),steadyNowS);}
  if(!motion.valid)return fail(std::string("canonical_motion_unavailable: ")+motion.unavailable_reason);
  if(motion.revision!=route_observation_.revision) {
    ros::Time stamp;stamp.fromNSec(motion.source_stamp_ns);
    try {
      const auto transform=tf_->lookupTransform("map","base_link",stamp);
      if(transform.header.stamp!=stamp)return fail("route_exact_pose_stamp_mismatch");
      const auto& p=transform.transform.translation;const auto& q=transform.transform.rotation;
      route_observation_={{p.x,p.y,tf2::getYaw(q)},motion.forward_mps,motion.lateral_mps,
          motion.yaw_radps,motion.source_stamp_s,motion.revision};
    } catch(const tf2::TransformException&) {return fail("route_exact_pose_unavailable");}
  }
  const auto intent=route_follower_->Tick(route_observation_,now.toSec());
  if(!intent.ok)return fail(intent.reason);
  RangerControlDecision desired{intent.complete?PlannerState::GOAL_REACHED:
      intent.vx_mps>0?PlannerState::FOLLOW_ARC:PlannerState::SPIN_ALIGN,
      intent.vx_mps,0,intent.wz_radps,intent.wz_radps!=0?intent.vx_mps/intent.wz_radps:0,intent.reason};
  const auto limited=command_limiter_->limit(desired,now.toSec(),motion);
  if(limited.state==PlannerState::BLOCKED)return fail(limited.reason);
  const auto sweeps=CommandStoppingSweeps(route_observation_.pose,motion.forward_mps,motion.lateral_mps,
      motion.yaw_radps,limited.linear_x_mps,limited.angular_z_radps,route_policy_->preflight.limits,
      std::max(0.,now.toSec()-motion.source_stamp_s)+.1);
  if(sweeps.empty())return fail("invalid_command_stopping_sweep");
  MotionSweepChecker checker(evidence->grid);
  auto geometry=route_policy_->preflight.geometry;geometry.integration_error_m=.001;
  if(intent.correcting) {
    // Cover the entire correction from the actual stopped/drifting pose. This
    // can extend outside the admitted spin samples; no original route waypoint
    // is substituted for the physical start. Command stopping sweeps below also
    // cover response beyond the correction target.
    const auto correction=MakeSpinPrimitive(route_observation_.pose,
        MotionAngleDifference(intent.correction_target_yaw_rad,route_observation_.pose.yaw_rad),
        route_policy_->preflight.limits,.004);
    std::vector<MotionPose> poses;
    for(const auto& sample:correction.samples)poses.push_back(sample.pose);
    if(poses.empty() || !checker.Check(geometry,poses).clear())return fail("spin_correction_sweep_blocked");
    for(std::size_t i=intent.segment+1;i<route->segments.size();++i) {
      poses.clear();for(const auto& sample:route->segments[i].samples)poses.push_back(sample.pose);
      auto remaining=geometry;remaining.integration_error_m+=route->segments[i].geometry_error_m;
      if(!checker.Check(remaining,poses).clear())return fail("spin_correction_remaining_route_blocked");
    }
  }
  for(const auto& sweep:sweeps)if(!checker.Check(geometry,sweep).clear())
    return fail("finite_command_stopping_sweep_blocked");
  if(!SharedMotionRouteStore().progress(binding,intent.segment,intent.sample))
    return fail("route_progress_invalid_or_expired");
  route_follower_->AcceptLimitedCommand(limited.linear_x_mps,limited.angular_z_radps);
  command.linear.x=limited.linear_x_mps;command.angular.z=limited.angular_z_radps;
  route_complete_=intent.complete;
  publishDecision(limited,true,true,true,0);return true;
}

bool RangerLocalPlanner::isGoalReached() {
  return command_gate_.Compute([&](const CommandBinding& binding,std::uint64_t sequence,bool ready) {
    const auto computed_at=ros::Time::now();
    if(!ready)return false;
    if(!currentGoalReached(binding,computed_at)) {
      if(!progress_reason_.empty()) {
        publishAuthorized(binding,sequence,computed_at,geometry_msgs::Twist{});
        publishDecision(makeBlockedDecision(progress_reason_),false,false,false,0.0);
      }
      return false;
    }
    // move_base's final Twist is diagnostic only. End the authoritative stream
    // with a zero carrying this same operation's identity before reporting done.
    publishAuthorized(binding,sequence,computed_at,geometry_msgs::Twist{});
    return true;
  });
}

bool RangerLocalPlanner::currentGoalReached(const CommandBinding& binding,const ros::Time& now) {
  if(!route_complete_)return false;
  geometry_msgs::Twist verified;
  return computeRouteCommand(verified,binding,now) && route_complete_ &&
      verified.linear.x==0 && verified.angular.z==0 && command_limiter_->commandStopped() &&
      checkNavigationProgress(binding,ros::Time::now());
}

bool RangerLocalPlanner::checkNavigationProgress(const CommandBinding& binding,const ros::Time& now) {
  progress_reason_.clear();
  const auto reject=[&](const std::string& reason) {
    progress_reason_=reason;
    if(navigation_progress_ && navigation_progress_->faulted())
      progress_reason_=navigation_progress_->detail()+" session="+binding.session_id.substr(0,64)+
          " grant="+binding.grant_id.substr(0,64)+" operation="+binding.source_goal_id.substr(0,64);
    return false;
  };
  double receipt=0;
  CanonicalMotionProgress motion;
  {std::lock_guard<std::mutex> lock(evidence_mutex_);
    motion=motion_feedback_.currentAtRead(binding,now.toSec(),[&]{return receipt=steadyNowS();});}
  if(!binding.route_id.empty() && receipt>=binding.task_deadline_s)return reject("navigation_task_deadline_expired");
  if(!initialized_ || !navigation_progress_ || !tf_)return reject("navigation_progress_unavailable");
  if(!navigation_progress_->Check(now.toSec(),receipt))return reject("navigation_progress_not_started");
  if(!motion.valid)return reject(std::string("canonical_motion_unavailable: ")+motion.unavailable_reason);
  if(motion.revision==0 || motion.source_stamp_ns==0)return reject("navigation_progress_missing_original_stamp");
  if(motion.revision==resolved_progress_pose_.revision &&
     motion.source_stamp_ns==resolved_progress_pose_.stamp_ns &&
     motion.source_stamp_s==resolved_progress_pose_.stamp_s) {
    // Canonical identity/freshness and guard clocks were checked above. Reuse
    // the immutable observation, not another numerical TF interpolation of it.
    if(!navigation_progress_->Observe(resolved_progress_pose_,now.toSec(),receipt))
      return reject("navigation_progress_pose_not_current");
    return true;
  }
  ros::Time stamp;stamp.fromNSec(motion.source_stamp_ns);
  geometry_msgs::TransformStamped transform;
  try {
    // Never use latest/map pose. This is the accepted canonical measurement's
    // odom/base origin, including the original-stamp sensor extrinsic.
    transform=tf_->lookupTransform("odom","base_link",stamp);
  } catch(const tf2::TransformException&) {return reject("navigation_progress_exact_tf_unavailable");}
  const auto& p=transform.transform.translation;const auto& q=transform.transform.rotation;
  if(transform.header.frame_id!="odom" || transform.child_frame_id!="base_link" || transform.header.stamp!=stamp ||
     !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
     !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
     std::abs(std::hypot(std::hypot(q.x,q.y),std::hypot(q.z,q.w))-1.0)>1.e-5)
    return reject("navigation_progress_invalid_pose_transform");
  const NavigationProgressPose pose{motion.revision,motion.source_stamp_ns,motion.source_stamp_s,
      p.x,p.y,tf2::getYaw(q)};
  if(!navigation_progress_->Observe(pose,now.toSec(),receipt))return reject("navigation_progress_pose_not_current");
  if(pose.stamp_s<=now.toSec())resolved_progress_pose_=pose;
  return true;
}

void RangerLocalPlanner::authorityCallback(const BaseAuthorityState::ConstPtr& message) {
  CanonicalMotionState state;
  state.session_id=message->session_id;state.grant_id=message->grant_id;
  state.source_goal_id=message->source_goal_id;
  state.navigation_active=message->state==BaseAuthorityState::NAVIGATION &&
      message->selected_source=="navigation" && !message->manipulation_hold_active;
  state.motion_valid=message->motion_valid;state.revision=message->motion_revision;
  state.motion_stamp_ns=message->motion_stamp.toNSec();
  state.authority_stamp_s=message->header.stamp.toSec();state.motion_stamp_s=message->motion_stamp.toSec();
  state.frame_id=message->motion_frame_id;state.receipt_age_s=message->motion_receipt_age_s;
  state.receipt_limit_s=message->motion_receipt_limit_s;state.header_limit_s=message->motion_header_limit_s;
  state.future_tolerance_s=message->motion_future_tolerance_s;
  state.linear_lead_mps=message->linear_progress_lead_mps;
  state.angular_lead_radps=message->angular_progress_lead_radps;
  const auto& linear=message->motion.linear;const auto& angular=message->motion.angular;
  state.motion={{linear.x,linear.y,linear.z,angular.x,angular.y,angular.z}};
  // Never acquire the command gate here: generation owns gate -> evidence.
  const double receipt=steadyNowS();
  std::lock_guard<std::mutex> lock(evidence_mutex_);
  motion_feedback_.observe(state,receipt);
}

void RangerLocalPlanner::publishDecision(
    const RangerControlDecision& decision,
    bool arc_collision_free,
    bool rotation_costmap_clear,
    bool rotation_lidar_clear,
    double path_heading_error_rad) {
  CoarseNavigationState state;
  state.header.stamp = ros::Time::now();
  state.header.frame_id = costmap_ros_ == nullptr
      ? std::string()
      : costmap_ros_->getBaseFrameID();
  state.state = static_cast<uint8_t>(decision.state);
  state.arc_collision_free = arc_collision_free;
  state.rotation_costmap_clear = rotation_costmap_clear;
  state.rotation_lidar_clear = rotation_lidar_clear;
  state.path_heading_error_rad = path_heading_error_rad;
  state.command_radius_m = decision.command_radius_m;
  state.reason = decision.reason;
  state.invocation_id=route_binding_.invocation_id;state.route_id=route_binding_.route_id;
  state.session_id=route_binding_.session_id;state.grant_id=route_binding_.grant_id;
  state.source_goal_id=route_binding_.source_goal_id;
  state_publisher_.publish(state);
}

}  // namespace agx_workbench_navigation

PLUGINLIB_EXPORT_CLASS(
    agx_workbench_navigation::RangerLocalPlanner,
    nav_core::BaseLocalPlanner)
