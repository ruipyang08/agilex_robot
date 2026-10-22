#include <agx_workbench_navigation/ranger_local_planner.hpp>
#include <agx_workbench_navigation/motion_speed_profile.hpp>
#include <agx_workbench_navigation/motion_route_store.hpp>
#include <agx_workbench_navigation/motion_evidence_ros.hpp>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <memory>
#include <chrono>
#include <limits>
#include <std_msgs/String.h>
#include <boost/property_tree/json_parser.hpp>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <algorithm>
#include <map>
#include <sstream>
#include <locale>
#include <type_traits>

namespace agx_workbench_navigation {
class RangerLocalPlannerProgressTest : public testing::Test {
 protected:
  void SetUp() override {
    ros::Time::setNow(ros::Time(100));
    tf_.reset(new tf2_ros::Buffer(ros::Duration(500)));
    // The fixture synchronously populates every required transform before use.
    tf_->setUsingDedicatedThread(true);
    Transform("map","odom",100,0,0,0,true);
    Transform("rslidar","base_link",100,-.4,0,0,true);
    Pose(100,0,0,0);
    ros::NodeHandle nh("~/progress_costmap");
    nh.setParam("global_frame","map");nh.setParam("robot_base_frame","base_link");
    nh.setParam("update_frequency",0.0);nh.setParam("publish_frequency",0.0);
    nh.setParam("width",10.0);nh.setParam("height",10.0);nh.setParam("resolution",.05);
    nh.setParam("origin_x",-5.0);nh.setParam("origin_y",-5.0);
    nh.setParam("transform_tolerance",.25);
    nh.setParam("footprint","[[-0.3,-0.2],[-0.3,0.2],[0.3,0.2],[0.3,-0.2]]");
    XmlRpc::XmlRpcValue plugins;plugins.setSize(0);nh.setParam("plugins",plugins);
    ros::NodeHandle local("~/RangerLocalPlanner");
    local.setParam("progress_timeout_s",45.0);local.setParam("progress_min_translation_m",.2);
    local.setParam("progress_min_rotation_rad",.1);
    ros::NodeHandle("~").setParam("oscillation_timeout",0.0);
    map_.reset(new costmap_2d::Costmap2DROS("progress_costmap",*tf_));
    map_->getCostmap()->resizeMap(200,200,.05,-5,-5);
    // A test costmap with no sensor plugins is explicitly free, not unknown.
    for(unsigned int x=0;x<200;++x)for(unsigned int y=0;y<200;++y)map_->getCostmap()->setCost(x,y,0);
    planner_.reset(new RangerLocalPlanner());planner_->initialize("RangerLocalPlanner",tf_.get(),map_.get());
  }
  void TearDown() override {
    if(!binding_.route_id.empty())SharedMotionRouteStore().release(binding_.route_id,binding_.invocation_id);
    PublishSharedMotionEvidence({});planner_.reset();map_.reset();tf_.reset();
  }
  std::string CaptureConfiguration() {
    ros::NodeHandle nh;nh.setCallbackQueue(&command_queue_);std::vector<std::string> messages;
    auto subscriber=nh.subscribe<std_msgs::String>("/workbench_nav/effective_config/local_planner",10,
        [&messages](const std_msgs::String::ConstPtr& message){messages.push_back(message->data);});
    const auto deadline=ros::WallTime::now()+ros::WallDuration(2);
    while(messages.empty()&&ros::WallTime::now()<deadline)command_queue_.callAvailable(ros::WallDuration(.01));
    EXPECT_FALSE(messages.empty());messages.clear();
    planner_->publishEffectiveConfiguration(ros::WallTimerEvent());
    while(messages.empty()&&ros::WallTime::now()<deadline)command_queue_.callAvailable(ros::WallDuration(.01));
    EXPECT_FALSE(messages.empty());return messages.empty()?"{}":messages.back();
  }
  void Transform(const std::string& parent,const std::string& child,double stamp,double x,double y,double yaw,bool fixed=false) {
    geometry_msgs::TransformStamped value;value.header.frame_id=parent;value.child_frame_id=child;
    value.header.stamp=ros::Time(stamp);value.transform.translation.x=x;value.transform.translation.y=y;
    tf2::Quaternion q;q.setRPY(0,0,yaw);value.transform.rotation=tf2::toMsg(q);
    ASSERT_TRUE(tf_->setTransform(value,"progress_test",fixed));
  }
  void Pose(double stamp,double x,double y,double yaw) {
    // Published pose is at an offset sensor origin. The actual base pose is
    // obtained by composing the original-stamp full child-to-base transform.
    Transform("odom","rslidar",stamp,x+.4*std::cos(yaw),y+.4*std::sin(yaw),yaw);
  }
  geometry_msgs::PoseStamped Bind(double x,double yaw=0,const std::string& operation="goal") {
    BindBaseCommandSource::Request request;BindBaseCommandSource::Response response;
    request.session_id="session";request.grant_id="grant-"+operation;request.source_goal_id=operation;
    request.expected_goal.header.frame_id="map";request.expected_goal.header.stamp=ros::Time::now();
    request.expected_goal.pose.position.x=x;tf2::Quaternion q;q.setRPY(0,0,yaw);
    request.expected_goal.pose.orientation=tf2::toMsg(q);
    if(!binding_.route_id.empty())SharedMotionRouteStore().release(binding_.route_id,binding_.invocation_id);
    const auto& policy=*planner_->route_policy_;
    MotionRoute route;route.route_id="route-"+operation;route.invocation_id=operation;
    route.model_digest=policy.preflight.model_digest;route.map_digest="fixture-map";route.registry_digest="fixture-registry";
    if(x>0)route.segments.push_back(MakeForwardPrimitive({},0,0,x,0,0,.3,policy.preflight.limits));
    if(yaw!=0)route.segments.push_back(MakeSpinPrimitive({x,0,0},yaw,policy.preflight.limits));
    if(route.segments.empty())route.segments.push_back(MakeSpinPrimitive({},.1,policy.preflight.limits));
    route.goal=route.segments.back().samples.back().pose;
    for(const auto& part:route.segments)route.estimated_time_s+=part.estimated_time_s;
    request.invocation_id=operation;request.route_id=route.route_id;
    request.task_deadline_s=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count()+180;
    EXPECT_TRUE(SharedMotionRouteStore().put(route,policy.preflight.limits));
    EXPECT_TRUE(SharedMotionRouteStore().pin(route.route_id,operation,request.task_deadline_s));
    EXPECT_TRUE(planner_->bindSource(request,response));EXPECT_TRUE(response.accepted);
    binding_=request;return request.expected_goal;
  }
  void ReplayBinding() {BindBaseCommandSource::Response response;EXPECT_TRUE(planner_->bindSource(binding_,response));EXPECT_TRUE(response.accepted);}
  void Observe(double stamp,double x=0,double yaw=0,bool transform=true,double measured_yaw=0,double measured_forward=0) {
    ros::Time::setNow(ros::Time(stamp));if(transform)Pose(stamp,x,0,yaw);
    BaseAuthorityState::Ptr state(new BaseAuthorityState());state->header.stamp=ros::Time(stamp);
    state->session_id=binding_.session_id;state->grant_id=binding_.grant_id;state->source_goal_id=binding_.source_goal_id;
    state->state=BaseAuthorityState::NAVIGATION;state->selected_source="navigation";
    state->motion_valid=true;state->motion_revision=++revision_;state->motion_stamp=ros::Time(stamp);
    state->motion_frame_id="base_link";state->motion_receipt_age_s=0;state->motion_receipt_limit_s=.25;
    state->motion_header_limit_s=.35;state->motion_future_tolerance_s=.05;
    state->linear_progress_lead_mps=.002;state->angular_progress_lead_radps=.004;
    state->motion.angular.z=measured_yaw;state->motion.linear.x=measured_forward;
    last_authority_=state;planner_->authorityCallback(state);
    MotionPlanningEvidence e;e.grid.width=e.grid.height=200;e.grid.resolution_m=.05;
    e.grid.origin_x_m=e.grid.origin_y_m=-5;e.grid.cells.assign(40000,0);
    e.static_map_digest="fixture-map";e.registry_digest="fixture-registry";e.frame_id="map";
    e.observed_at_s=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    e.pose_deadline_s=e.obstacle_deadline_s=e.observed_at_s+.25;
    PublishSharedMotionEvidence(std::move(e));
  }
  bool Progress() {return planner_->checkNavigationProgress(
      {binding_.session_id,binding_.grant_id,binding_.source_goal_id,{}},ros::Time::now());}
  void RepeatAuthorityWithChangedYaw(double yaw) {
    BaseAuthorityState::Ptr changed(new BaseAuthorityState(*last_authority_));
    changed->motion.angular.z=yaw;planner_->authorityCallback(changed);
  }
  double CorrectionStart() {
    const auto goal=Bind(0,1.5707963267948966);EXPECT_TRUE(planner_->setPlan({goal}));
    auto route=SharedMotionRouteStore().get(binding_.route_id,binding_.invocation_id);EXPECT_TRUE(bool(route));
    const auto& policy=*planner_->route_policy_;
    planner_->route_follower_.reset(new MotionRouteFollower(*route,policy.preflight.limits,.075));
    FollowerObservation o;o.source_stamp_s=100;o.revision=1;
    auto intent=planner_->route_follower_->Tick(o,100);double stamp=100;bool turned=false;
    while(stamp<140 && !(turned && intent.state==FollowerState::BrakeToStop)) {
      stamp+=.1;o.pose.yaw_rad+=intent.wz_radps*.1;o.yaw_radps=intent.wz_radps;
      o.source_stamp_s=stamp;++o.revision;intent=planner_->route_follower_->Tick(o,stamp);
      EXPECT_TRUE(intent.ok);turned=turned || intent.wz_radps!=0;
    }
    EXPECT_TRUE(turned && stamp<140);
    o.pose.yaw_rad=route->goal.yaw_rad+.026;o.yaw_radps=0;
    // Canonical authority revision must monotonically follow these observations.
    for(int i=0;i<4;++i) {
      stamp+=.1;o.source_stamp_s=stamp;++o.revision;intent=planner_->route_follower_->Tick(o,stamp);
      if(intent.correcting)break;
    }
    EXPECT_TRUE(intent.correcting);revision_=o.revision;planner_->command_limiter_->reset();
    Observe(stamp,0,o.pose.yaw_rad);return stamp;
  }
  MotionPolygon RouteFootprint() {return planner_->route_policy_->preflight.geometry.footprint;}
  void AsymmetricCorrectionGeometry() {
    // A small synthetic body makes the newly swept region unambiguous. This
    // changes only the isolated fixture, never the installed robot geometry.
    auto& geometry=planner_->route_policy_->preflight.geometry;
    geometry.footprint={{{-.1,-.1}},{{.4,-.1}},{{.4,.1}},{{-.1,.1}}};
    geometry.hard_padding_m=0;geometry.translation_step_m=.0001;
    geometry.rotation_step_rad=.0005;geometry.integration_error_m=.001;
  }
  MotionGeometryPolicy CorrectionGeometry() {return planner_->route_policy_->preflight.geometry;}
  bool TryBind(BindBaseCommandSource::Request& request) {
    BindBaseCommandSource::Response response;EXPECT_TRUE(planner_->bindSource(request,response));return response.accepted;
  }
  struct PlanState {
    std::vector<geometry_msgs::PoseStamped> path;
    geometry_msgs::PoseStamped goal;bool goal_available,ready;ros::Time receipt;
    NavigationProgressPose pose;std::string reason;
    MotionRouteFollower* follower;RangerCommandLimiter limiter;NavigationProgressGuard guard;
  };
  PlanState Snapshot() {
    return {planner_->global_plan_,planner_->last_goal_,planner_->last_goal_available_,
      planner_->command_gate_.Inspect([]{return true;}),planner_->plan_receipt_time_,
      planner_->resolved_progress_pose_,planner_->progress_reason_,planner_->route_follower_.get(),
      *planner_->command_limiter_,*planner_->navigation_progress_};
  }
  void ExpectDecisionEqual(const RangerControlDecision& before,const RangerControlDecision& after) {
    EXPECT_EQ(before.state,after.state);EXPECT_EQ(before.reason,after.reason);
    EXPECT_DOUBLE_EQ(before.linear_x_mps,after.linear_x_mps);
    EXPECT_DOUBLE_EQ(before.linear_y_mps,after.linear_y_mps);
    EXPECT_DOUBLE_EQ(before.angular_z_radps,after.angular_z_radps);
    EXPECT_DOUBLE_EQ(before.command_radius_m,after.command_radius_m);
  }
  void ExpectUnchanged(PlanState before) {
    auto after=Snapshot();
    EXPECT_EQ(before.path,after.path);EXPECT_EQ(before.goal,after.goal);
    EXPECT_EQ(before.goal_available,after.goal_available);
    EXPECT_EQ(before.ready,after.ready);EXPECT_EQ(before.receipt,after.receipt);EXPECT_EQ(before.reason,after.reason);
    EXPECT_EQ(before.pose.revision,after.pose.revision);EXPECT_EQ(before.pose.stamp_ns,after.pose.stamp_ns);
    EXPECT_DOUBLE_EQ(before.pose.stamp_s,after.pose.stamp_s);EXPECT_DOUBLE_EQ(before.pose.x,after.pose.x);
    EXPECT_DOUBLE_EQ(before.pose.y,after.pose.y);EXPECT_DOUBLE_EQ(before.pose.yaw,after.pose.yaw);
    EXPECT_EQ(before.limiter.commandStopped(),after.limiter.commandStopped());
    EXPECT_EQ(before.guard.faulted(),after.guard.faulted());EXPECT_EQ(before.guard.detail(),after.guard.detail());
    EXPECT_EQ(before.follower,after.follower);
    // Probe copies, leaving the real limiter/control/guard untouched. An old
    // install/reset must not erase slew history or the original progress budget.
    const double now=ros::Time::now().toSec();
    CanonicalMotionProgress motion;motion.valid=true;motion.forward_mps=.3;
    motion.yaw_radps=.2;motion.linear_lead_mps=.002;motion.angular_lead_radps=.004;
    for(double dt:{0.,.01,.2}) {
      RangerControlDecision desired{PlannerState::FOLLOW_ARC,.3,0,.06,5,"probe"};
      ExpectDecisionEqual(before.limiter.limit(desired,now+dt,motion),after.limiter.limit(desired,now+dt,motion));
      }
    const double receipt=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    for(int dt=0;dt<=46;++dt) {
      EXPECT_EQ(before.guard.Check(now+dt,receipt+dt),after.guard.Check(now+dt,receipt+dt));
      EXPECT_EQ(before.guard.detail(),after.guard.detail());
    }
  }
  struct GateState {CommandBinding binding;std::uint64_t sequence=0;bool ready=false;};
  GateState ReadGate() {
    GateState result;
    EXPECT_TRUE(planner_->command_gate_.Compute([&](const auto& binding,auto sequence,bool ready) {
      result={binding,sequence,ready};return true;
    }));
    return result;
  }
  void ExpectDelivery(const std::vector<geometry_msgs::PoseStamped>& path,bool acknowledged) {
    auto before=Snapshot();const auto gate_before=ReadGate();
    EXPECT_EQ(acknowledged,planner_->setPlan(path));
    ExpectUnchanged(before);const auto gate_after=ReadGate();
    EXPECT_EQ(gate_before.binding.session_id,gate_after.binding.session_id);
    EXPECT_EQ(gate_before.binding.grant_id,gate_after.binding.grant_id);
    EXPECT_EQ(gate_before.binding.source_goal_id,gate_after.binding.source_goal_id);
    EXPECT_EQ(gate_before.binding.endpoint.stamp_ns,gate_after.binding.endpoint.stamp_ns);
    EXPECT_EQ(gate_before.binding.endpoint.position,gate_after.binding.endpoint.position);
    EXPECT_EQ(gate_before.binding.endpoint.orientation,gate_after.binding.endpoint.orientation);
    EXPECT_EQ(gate_before.ready,gate_after.ready);
    EXPECT_EQ(gate_before.sequence+1,gate_after.sequence); // setPlan minted no command identity.
  }
  void StartCommandCapture() {
    ros::NodeHandle nh;nh.setCallbackQueue(&command_queue_);
    command_subscriber_=nh.subscribe<AuthorizedBaseCommand>("/workbench_nav/cmd_vel/navigation",100,
        [this](const AuthorizedBaseCommand::ConstPtr& message){commands_.push_back(*message);});
    const auto deadline=ros::WallTime::now()+ros::WallDuration(2);
    while(planner_->authorized_publisher_.getNumSubscribers()==0 && ros::WallTime::now()<deadline)
      command_queue_.callAvailable(ros::WallDuration(.01));
    ASSERT_GT(planner_->authorized_publisher_.getNumSubscribers(),0u);
  }
  void ExpectOwnedZero() {
    EXPECT_FALSE(planner_->isGoalReached());const auto gate=ReadGate();
    geometry_msgs::Twist command;command.linear.x=1;command.angular.z=1;
    EXPECT_FALSE(planner_->computeVelocityCommands(command));EXPECT_EQ(geometry_msgs::Twist{},command);
    const auto expected_sequence=gate.sequence+1;
    const auto deadline=ros::WallTime::now()+ros::WallDuration(2);
    while((commands_.empty() || commands_.back().sequence<expected_sequence) && ros::WallTime::now()<deadline)
      command_queue_.callAvailable(ros::WallDuration(.01));
    ASSERT_FALSE(commands_.empty());const auto& published=commands_.back();
    EXPECT_EQ(binding_.session_id,published.session_id);EXPECT_EQ(binding_.grant_id,published.grant_id);
    EXPECT_EQ(binding_.source_goal_id,published.source_goal_id);EXPECT_EQ(expected_sequence,published.sequence);
    EXPECT_EQ(geometry_msgs::Twist{},published.twist);
  }
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<costmap_2d::Costmap2DROS> map_;
  std::unique_ptr<RangerLocalPlanner> planner_;
  BindBaseCommandSource::Request binding_;
  BaseAuthorityState::Ptr last_authority_;
  std::uint64_t revision_=0;
  ros::CallbackQueue command_queue_;ros::Subscriber command_subscriber_;
  std::vector<AuthorizedBaseCommand> commands_;
};
TEST_F(RangerLocalPlannerProgressTest, CompletionCannotBypassExpiryDespiteReplansAndBindReplay) {
  const auto goal=Bind(1);auto start=goal;start.pose.position.x=0;
  const std::vector<geometry_msgs::PoseStamped> path{start,goal};
  ASSERT_TRUE(planner_->setPlan(path));Observe(100,0,0,false);
  geometry_msgs::Twist command;ASSERT_TRUE(planner_->computeVelocityCommands(command));
  for(int i=1;i<45;i+=2) {Observe(100+i);ReplayBinding();ASSERT_TRUE(planner_->setPlan(path));}
  Observe(145,1); // apparent late completion must not erase the expired guard
  EXPECT_FALSE(planner_->isGoalReached());
  EXPECT_FALSE(planner_->computeVelocityCommands(command));
  EXPECT_DOUBLE_EQ(command.linear.x,0);EXPECT_DOUBLE_EQ(command.angular.z,0);
  ASSERT_TRUE(planner_->setPlan(path));Observe(146,1);
  EXPECT_FALSE(planner_->isGoalReached());EXPECT_FALSE(planner_->computeVelocityCommands(command));
}
TEST_F(RangerLocalPlannerProgressTest, SlowPointTurnExceedsFortyFiveSourceSeconds) {
  const auto goal=Bind(0,-3.);ASSERT_TRUE(planner_->setPlan({goal}));geometry_msgs::Twist command;
  for(int i=0;i<=60;++i) {
    Observe(100+i,0,-i*.02,i>0,-.02);
    ASSERT_TRUE(planner_->setPlan({goal}));ReplayBinding();
    EXPECT_FALSE(planner_->isGoalReached());
    ASSERT_TRUE(planner_->computeVelocityCommands(command))<<i;
  }
}
TEST_F(RangerLocalPlannerProgressTest, ExactOriginalTransformRequiredAndMapChangesDoNotCount) {
  const auto goal=Bind(1);ASSERT_TRUE(planner_->setPlan({goal}));Observe(100,0,0,false);
  ASSERT_TRUE(Progress());
  Observe(110,0,0,false); // latest TF is not a substitute for the measurement time
  EXPECT_FALSE(Progress());
  Observe(110.1);ASSERT_TRUE(Progress());
  Transform("map","odom",120,10,10,1,true);Observe(120);
  ASSERT_TRUE(Progress());
  Observe(145);EXPECT_FALSE(Progress());
}
TEST_F(RangerLocalPlannerProgressTest, SourceRegressionIsLatchedUntilNewAdmittedOperation) {
  const auto goal=Bind(1);ASSERT_TRUE(planner_->setPlan({goal}));Observe(100,0,0,false);ASSERT_TRUE(Progress());
  Observe(110);ASSERT_TRUE(Progress());ros::Time::setNow(ros::Time(109));EXPECT_FALSE(Progress());
  Observe(111,.3);EXPECT_FALSE(Progress());ReplayBinding();EXPECT_FALSE(Progress());
  ros::Time::setNow(ros::Time(112));const auto next=Bind(2,0,"next");ASSERT_TRUE(planner_->setPlan({next}));
  Observe(112,.3);EXPECT_TRUE(Progress());
}
TEST_F(RangerLocalPlannerProgressTest, CanonicalPoseIsStableAcrossTfNewestToInteriorTransition) {
  const auto goal=Bind(0,-3);ASSERT_TRUE(planner_->setPlan({goal}));Observe(100,0,0,false,-.02);
  ASSERT_TRUE(Progress());Pose(100.05,0,0,-.001);
  int numerical_changes=0;geometry_msgs::Twist command;
  for(int i=1;i<=300;++i) {
    const double stamp=100+.05*i;
    Observe(stamp,0,-.001*i,false,-.02); // This TF was inserted by the preceding iteration.
    ASSERT_TRUE(Progress());
    const auto newest=tf_->lookupTransform("odom","base_link",ros::Time(stamp));
    Pose(100+.05*(i+1),0,0,-.001*(i+1));
    const auto interior=tf_->lookupTransform("odom","base_link",ros::Time(stamp));
    if(newest.transform.translation.x!=interior.transform.translation.x ||
       newest.transform.translation.y!=interior.transform.translation.y ||
       tf2::getYaw(newest.transform.rotation)!=tf2::getYaw(interior.transform.rotation))++numerical_changes;
    // No canonical callback, revision, stamp or source transform was changed.
    // TF2 may nevertheless run endpoint slerp now that this is an interior time.
    ASSERT_TRUE(Progress())<<"same original observation at iteration "<<i;
    EXPECT_FALSE(planner_->isGoalReached());
    ASSERT_TRUE(planner_->computeVelocityCommands(command));
  }
  ASSERT_GT(numerical_changes,0)<<"fixture did not exercise installed TF2 endpoint arithmetic";
}
TEST_F(RangerLocalPlannerProgressTest, ResolvedPoseNeverBypassesFreshnessMutationOrOriginalDeadline) {
  const auto goal=Bind(1);ASSERT_TRUE(planner_->setPlan({goal}));Observe(100,0,0,false);ASSERT_TRUE(Progress());
  RepeatAuthorityWithChangedYaw(.001);EXPECT_FALSE(Progress()); // Same revision was mutated.
  Observe(100.05);ASSERT_TRUE(Progress());
  ros::WallDuration(.27).sleep();EXPECT_FALSE(Progress()); // Original receipt lease still expires.
  Observe(100.10);ASSERT_TRUE(Progress());
  ros::Time::setNow(ros::Time(100.46));EXPECT_FALSE(Progress()); // Original source lease still expires.
  Observe(100.50);ASSERT_TRUE(Progress());
  for(int i=101;i<145;++i){Observe(i);ASSERT_TRUE(Progress());}
  Observe(145);EXPECT_FALSE(Progress()); // Fresh zero poses cannot renew the original allowance.
  ReplayBinding();EXPECT_FALSE(Progress());
  ros::Time::setNow(ros::Time(146));const auto next=Bind(2,0,"next");ASSERT_TRUE(planner_->setPlan({next}));
  Observe(146);EXPECT_TRUE(Progress());
}
TEST_F(RangerLocalPlannerProgressTest, SupersededDeliveryBeforeCurrentPlanIsAcknowledgedWithoutInstalling) {
  StartCommandCapture();const auto old=Bind(1);ASSERT_TRUE(planner_->setPlan({old}));
  Observe(100,0,0,false);ASSERT_TRUE(Progress());
  ros::Time::setNow(ros::Time(110));const auto current=Bind(2,0,"next");Observe(110);
  const auto pending=Snapshot();ASSERT_TRUE(pending.path.empty());ASSERT_FALSE(pending.ready);
  ASSERT_EQ(0u,pending.pose.revision);ASSERT_TRUE(pending.receipt.isZero());
  ExpectDelivery({old},true);ExpectOwnedZero();
  auto unknown=old;unknown.header.stamp=ros::Time(105);unknown.pose.position.y=99;
  ExpectDelivery({unknown},true);ExpectOwnedZero();
  auto start=current;start.pose.position.x=0;
  ASSERT_TRUE(planner_->setPlan({start,current}));EXPECT_TRUE(Snapshot().ready);
  EXPECT_EQ(current,Snapshot().goal);EXPECT_TRUE(Progress());
  geometry_msgs::Twist command;EXPECT_TRUE(planner_->computeVelocityCommands(command));
}
TEST_F(RangerLocalPlannerProgressTest, SupersededDeliveryPreservesInstalledPathSlewAndLatchedFault) {
  StartCommandCapture();const auto old=Bind(1);ASSERT_TRUE(planner_->setPlan({old}));
  ros::Time::setNow(ros::Time(110));const auto current=Bind(2,0,"next");
  auto start=current;start.pose.position.x=0;
  ASSERT_TRUE(planner_->setPlan({start,current}));Observe(110);geometry_msgs::Twist command;
  ASSERT_TRUE(planner_->computeVelocityCommands(command));Observe(110.1);
  ASSERT_TRUE(planner_->computeVelocityCommands(command));
  Observe(110.3);ASSERT_TRUE(planner_->computeVelocityCommands(command));
  ASSERT_GT(command.linear.x,0);ASSERT_FALSE(Snapshot().limiter.commandStopped());
  ASSERT_GT(Snapshot().pose.revision,0u);
  Observe(110.4);ExpectDelivery({old},true);
  ASSERT_TRUE(planner_->computeVelocityCommands(command));EXPECT_GT(command.linear.x,0);
  Observe(155);ExpectOwnedZero();ASSERT_TRUE(Snapshot().guard.faulted());
  ExpectDelivery({old},true);ExpectOwnedZero();
  ASSERT_TRUE(planner_->setPlan({current}));ExpectOwnedZero(); // A matching replan cannot revive a fault.
}
TEST_F(RangerLocalPlannerProgressTest, SupersededDeliveriesDoNotRenewBudgetWhileWaitingForCurrentPlan) {
  StartCommandCapture();const auto old=Bind(1);ASSERT_TRUE(planner_->setPlan({old}));
  ros::Time::setNow(ros::Time(110));const auto current=Bind(2,0,"next");
  for(double stamp:{110.,130.,154.,155.}) {
    Observe(stamp);ExpectDelivery({old},true);ExpectOwnedZero();
    EXPECT_FALSE(Snapshot().ready);EXPECT_TRUE(Snapshot().path.empty());
  }
  ASSERT_TRUE(planner_->setPlan({current}));ExpectOwnedZero();
  const auto fault=Snapshot();ASSERT_TRUE(fault.guard.faulted());
  EXPECT_NE(std::string::npos,fault.guard.detail().find("last_progress=110"));
  EXPECT_NE(std::string::npos,fault.guard.detail().find("navigation_progress_no_progress"));
  ExpectDelivery({old},true);ExpectOwnedZero();
}
TEST_F(RangerLocalPlannerProgressTest, NonSupersededInvalidDeliveriesRejectAndEmptyPlanStillInvalidates) {
  geometry_msgs::PoseStamped unbound;unbound.header.frame_id="map";unbound.header.stamp=ros::Time(90);
  unbound.pose.orientation.w=1;EXPECT_FALSE(planner_->setPlan({unbound}));
  const auto old=Bind(1);ASSERT_TRUE(planner_->setPlan({old}));
  ros::Time::setNow(ros::Time(110));const auto current=Bind(2,0,"next");
  ASSERT_TRUE(planner_->setPlan({current}));Observe(110);ASSERT_TRUE(Progress());
  auto wrong=old;wrong.header.frame_id="odom";ExpectDelivery({wrong},false);
  wrong=old;wrong.header.stamp=ros::Time(0);ExpectDelivery({wrong},false);
  wrong=old;wrong.pose.position.x=std::numeric_limits<double>::quiet_NaN();ExpectDelivery({wrong},false);
  wrong=old;wrong.pose.position.z=std::numeric_limits<double>::infinity();ExpectDelivery({wrong},false);
  wrong=old;wrong.pose.orientation.w=.5;ExpectDelivery({wrong},false);
  wrong=old;wrong.pose.orientation.z=std::numeric_limits<double>::quiet_NaN();ExpectDelivery({wrong},false);
  wrong=current;wrong.header.stamp=ros::Time(111);ExpectDelivery({wrong},false);
  wrong=current;wrong.pose.position.x+=.1;ExpectDelivery({wrong},false);
  wrong=current;wrong.pose.orientation.z=1;wrong.pose.orientation.w=0;ExpectDelivery({wrong},false);
  auto sign=current;sign.pose.orientation.w=-1;ASSERT_TRUE(planner_->setPlan({sign}));
  EXPECT_TRUE(Snapshot().ready);EXPECT_EQ(sign,Snapshot().goal);
  EXPECT_FALSE(planner_->setPlan({}));EXPECT_FALSE(Snapshot().ready);EXPECT_TRUE(Snapshot().path.empty());
  EXPECT_FALSE(planner_->isGoalReached());geometry_msgs::Twist command;
  EXPECT_FALSE(planner_->computeVelocityCommands(command));EXPECT_EQ(geometry_msgs::Twist{},command);
}
TEST_F(RangerLocalPlannerProgressTest, MissingPolicyAndNonUnitRouteScalingRejectStartup) {
  ros::NodeHandle local("~/RangerLocalPlanner");std::string path;
  ASSERT_TRUE(local.getParam("motion_policy_file",path));local.deleteParam("motion_policy_file");
  RangerLocalPlanner missing;EXPECT_THROW(missing.initialize("RangerLocalPlanner",tf_.get(),map_.get()),std::runtime_error);
  local.setParam("motion_policy_file",path);
  XmlRpc::XmlRpcValue policy;policy["velocity_scale"]=.5;policy["acceleration_scale"]=1.;
  policy["default_coefficient"]=1.;policy["task_coefficients"]=XmlRpc::XmlRpcValue();
  policy["task_coefficients"]["coarse_navigation"]=1.;ros::param::set("/rover_execution",policy);
  RangerLocalPlanner scaled;EXPECT_THROW(scaled.initialize("RangerLocalPlanner",tf_.get(),map_.get()),std::runtime_error);
  ros::param::del("/rover_execution");
}
TEST_F(RangerLocalPlannerProgressTest, ReportUsesInstalledRouteFootprintInsteadOfLegacyCircle) {
  const auto text=CaptureConfiguration();boost::property_tree::ptree report;std::istringstream stream(text);
  boost::property_tree::read_json(stream,report);MotionPolygon reported;
  for(const auto& vertex:report.get_child("configuration.collision_footprint")) {
    auto item=vertex.second.begin();const double x=item++->second.get_value<double>();
    reported.push_back({{x,item->second.get_value<double>()}});
  }
  EXPECT_EQ(RouteFootprint(),reported);
}
TEST_F(RangerLocalPlannerProgressTest, UnadmittedBindingCannotDispatch) {
  BindBaseCommandSource::Request request;BindBaseCommandSource::Response response;
  request.session_id="s";request.grant_id="g";request.source_goal_id="missing";
  request.expected_goal.header.frame_id="map";request.expected_goal.header.stamp=ros::Time::now();
  request.expected_goal.pose.orientation.w=1;
  EXPECT_FALSE(TryBind(request));
  geometry_msgs::Twist command;EXPECT_FALSE(planner_->computeVelocityCommands(command));
  EXPECT_EQ(geometry_msgs::Twist{},command);
}
TEST_F(RangerLocalPlannerProgressTest, ActualCommandSweepRejectsOccupiedInteriorAndKeepsOwnedZero) {
  const auto goal=Bind(1);ASSERT_TRUE(planner_->setPlan({goal}));Observe(100,0,0,false);
  auto evidence=*SharedMotionEvidence();evidence.grid.cells[100*200+100]=254;
  PublishSharedMotionEvidence(evidence);StartCommandCapture();ExpectOwnedZero();
  EXPECT_EQ("finite_command_stopping_sweep_blocked",Snapshot().reason);
}
TEST_F(RangerLocalPlannerProgressTest, CheckedCorrectionStartsOnlyWithOriginalMeasuredState) {
  const double stamp=CorrectionStart();geometry_msgs::Twist command;
  ASSERT_TRUE(planner_->computeVelocityCommands(command));EXPECT_DOUBLE_EQ(command.linear.x,0);
  EXPECT_LE(std::abs(command.angular.z),.05);
  Observe(stamp+.1,0,1.5967963267948966,true,command.angular.z);
  ASSERT_TRUE(planner_->computeVelocityCommands(command));EXPECT_LT(command.angular.z,0);EXPECT_LE(std::abs(command.angular.z),.05);
}
TEST_F(RangerLocalPlannerProgressTest, CorrectionRejectsOccupiedCompleteSweepBeforeDispatch) {
  CorrectionStart();auto evidence=*SharedMotionEvidence();
  // An occupied footprint interior must reject on the complete correction check
  // before the limiter's independent one-command stopping check.
  evidence.grid.cells[100*200+100]=254;PublishSharedMotionEvidence(evidence);
  StartCommandCapture();ExpectOwnedZero();EXPECT_EQ("spin_correction_sweep_blocked",Snapshot().reason);
}
TEST_F(RangerLocalPlannerProgressTest, CorrectionChecksActualOvershootOutsideAdmittedSamples) {
  AsymmetricCorrectionGeometry();CorrectionStart();auto evidence=*SharedMotionEvidence();
  evidence.grid.width=evidence.grid.height=1200;evidence.grid.resolution_m=.001;
  evidence.grid.origin_x_m=evidence.grid.origin_y_m=-.6;
  evidence.grid.cells.assign(1200*1200,0);
  // Cell centre (-0.1095, 0.3945) is clear throughout the original 0..90-degree
  // turn, but touched by the body at the measured 91.49-degree overshoot.
  evidence.grid.cells[994*1200+490]=254;
  const auto route=SharedMotionRouteStore().get(binding_.route_id,binding_.invocation_id);
  ASSERT_TRUE(bool(route));std::vector<MotionPose> original;
  for(const auto& sample:route->segments.front().samples)original.push_back(sample.pose);
  ASSERT_TRUE(MotionSweepChecker(evidence.grid).Check(CorrectionGeometry(),original).clear());
  PublishSharedMotionEvidence(evidence);StartCommandCapture();ExpectOwnedZero();
  EXPECT_EQ("spin_correction_sweep_blocked",Snapshot().reason);
}
TEST_F(RangerLocalPlannerProgressTest, StaleAuthorityDuringCorrectionKeepsOwnedZero) {
  CorrectionStart();ros::Time::setNow(ros::Time::now()+ros::Duration(.4));
  StartCommandCapture();ExpectOwnedZero();EXPECT_NE(std::string::npos,Snapshot().reason.find("canonical_motion_unavailable"));
}
}  // namespace agx_workbench_navigation
int main(int argc,char** argv) {
  ros::init(argc,argv,"navigation_progress_ros");ros::NodeHandle keep_ros_started;
  testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
