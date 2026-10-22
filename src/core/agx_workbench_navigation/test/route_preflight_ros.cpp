#include <agx_workbench_navigation/motion_plan_service.hpp>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <thread>

namespace nav=agx_workbench_navigation;
double Steady() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
TEST(RoutePreflight, ReadOnlyCandidateAndIndependentCallbackQueue) {
  ros::NodeHandle node("~");ros::AsyncSpinner global(1);global.start();
  std::atomic<int> commands{0},ticks{0};
  auto command=node.subscribe<geometry_msgs::Twist>("/cmd_vel",10,[&](const geometry_msgs::Twist::ConstPtr&) {++commands;});
  auto timer=node.createWallTimer(ros::WallDuration(.01),[&](const ros::WallTimerEvent&) {++ticks;});
  nav::MotionRouteStore store(Steady);nav::MotionPreflightConfig config;
  config.model_digest="model";config.frame_id="map";config.geometry.hard_padding_m=.02;
  config.geometry.soft_clearance_m=0;config.search.max_expansions=1;
  std::mutex mutex;std::condition_variable ready;
  bool held=true,entered=false,obstacle=false,stale=false;
  auto evidence=[&] {
    std::unique_lock<std::mutex> lock(mutex);entered=true;ready.notify_all();
    ready.wait_for(lock,std::chrono::seconds(3),[&] {return !held;});
    nav::MotionPlanningEvidence e;e.grid.width=100;e.grid.height=100;
    e.grid.resolution_m=.1;e.grid.origin_x_m=-5;e.grid.origin_y_m=-5;e.grid.cells.assign(10000,0);
    if(obstacle)e.grid.cells[50*100+56]=254;
    e.frame_id="map";e.static_map_digest="map-a";e.registry_digest="registry-a";
    e.observed_at_s=Steady();e.pose_deadline_s=e.observed_at_s+(stale?0:2.);
    e.obstacle_deadline_s=e.observed_at_s+2.;return e;
  };
  nav::MotionPlanService service(node,config,store,evidence,Steady);
  auto client=node.serviceClient<nav::PlanNavigationRoute>("plan_navigation_route");
  ASSERT_TRUE(client.waitForExistence(ros::Duration(3)));
  nav::PlanNavigationRoute request;
  request.request.invocation_id="one";request.request.model_digest="model";request.request.registry_digest="registry-a";
  for(auto* pose:{&request.request.measured_start,&request.request.coarse_start,&request.request.goal}) {
    pose->header.frame_id="map";pose->header.stamp=ros::Time::now();pose->pose.orientation.w=1;
  }
  request.request.goal.pose.position.x=1;
  auto pending=std::async(std::launch::async,[&] {return client.call(request);});
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(ready.wait_for(lock,std::chrono::seconds(2),[&] {return entered;}));
  }
  auto other=node.serviceClient<nav::PlanNavigationRoute>("plan_navigation_route");
  auto busy=request;const int before=ticks;
  ASSERT_TRUE(other.call(busy));
  EXPECT_EQ(busy.response.status,busy.response.EVIDENCE_UNAVAILABLE);
  EXPECT_TRUE(busy.response.route.segments.empty());
  std::this_thread::sleep_for(std::chrono::milliseconds(60));EXPECT_GT(ticks.load(),before);
  {
    std::lock_guard<std::mutex> lock(mutex);held=false;ready.notify_all();
  }
  ASSERT_TRUE(pending.get());ASSERT_EQ(request.response.status,request.response.READY);
  const auto id=request.response.route.route_id;
  EXPECT_FALSE(id.empty());ASSERT_FALSE(request.response.route.segments.empty());
  EXPECT_EQ(request.response.route.header.frame_id,"map");
  EXPECT_EQ(request.response.route.model_digest,"model");
  EXPECT_EQ(request.response.route.map_digest,"map-a");
  EXPECT_EQ(request.response.route.invocation_id,"one");
  ASSERT_TRUE(store.get(id,"one"));
  auto bad=request;bad.request.measured_start.pose.orientation.w=2;
  ASSERT_TRUE(client.call(bad));EXPECT_EQ(bad.response.status,bad.response.INVALID_REQUEST);
  EXPECT_TRUE(bad.response.route.segments.empty());
  bad=request;bad.request.goal.header.frame_id="odom";
  ASSERT_TRUE(client.call(bad));EXPECT_EQ(bad.response.status,bad.response.INVALID_REQUEST);
  {
    std::lock_guard<std::mutex> lock(mutex);stale=true;
  }
  bad=request;ASSERT_TRUE(client.call(bad));EXPECT_EQ(bad.response.status,bad.response.EVIDENCE_UNAVAILABLE);
  {
    std::lock_guard<std::mutex> lock(mutex);stale=false;obstacle=true;
  }
  request.request.existing_route_id=id;
  ASSERT_TRUE(client.call(request));EXPECT_EQ(request.response.status,request.response.ROUTE_INVALIDATED);
  EXPECT_TRUE(request.response.route.segments.empty());EXPECT_FALSE(store.get(id,"one"));
  EXPECT_EQ(commands.load(),0);
}
TEST(RoutePreflight, RegisteredDepartureRequiresOwnedMeasuredStop) {
  ros::NodeHandle node("~departure");ros::AsyncSpinner global(1);global.start();
  nav::MotionRouteStore store(Steady);nav::MotionPreflightConfig config;
  config.model_digest="model";config.frame_id="map";config.geometry.hard_padding_m=.02;
  config.search.max_expansions=1;
  nav::MotionPose measured;bool stopped=false,blocked=false;
  auto evidence=[&] {
    nav::MotionPlanningEvidence e;e.grid.width=160;e.grid.height=100;
    e.grid.resolution_m=.05;e.grid.origin_x_m=-5;e.grid.origin_y_m=-2.5;e.grid.cells.assign(16000,0);
    if(blocked)e.grid.cells[50*160+90]=254;
    e.frame_id="map";e.static_map_digest="map";e.registry_digest="registry";
    e.measured_pose=measured;e.observed_at_s=Steady();e.pose_deadline_s=e.obstacle_deadline_s=e.observed_at_s+2;
    e.departure_policy.enabled=true;e.departure_policy.geometry.hard_padding_m=.25;
    e.departure_policy.geometry.footprint={{{-.5,-.33}},{{0,-.33}},{{0,.33}},{{-.5,.33}}};
    e.registered_departures={{"a",{.36,0,0},{-1,0,0}}};e.departure_stop_observed=stopped;
    return e;
  };
  nav::MotionPlanService service(node,config,store,evidence,Steady);
  auto planner=node.serviceClient<nav::PlanNavigationRoute>("plan_navigation_route");
  auto manager=node.serviceClient<nav::ManageNavigationRoute>("manage_navigation_route");
  ASSERT_TRUE(planner.waitForExistence(ros::Duration(3)));
  nav::PlanNavigationRoute request;request.request.invocation_id="second";
  request.request.model_digest="model";request.request.registry_digest="registry";
  request.request.departure_source_id="a";request.request.coarse_start.pose.position.x=-1;
  for(auto* pose:{&request.request.measured_start,&request.request.coarse_start,&request.request.goal}) {
    pose->header.frame_id="map";pose->header.stamp=ros::Time::now();pose->pose.orientation.w=1;
  }
  ASSERT_TRUE(planner.call(request));ASSERT_EQ(request.response.status,request.response.READY)<<request.response.detail;
  ASSERT_EQ(request.response.route.departure_source_id,"a");
  ASSERT_EQ(request.response.route.segments.front().kind,nav::PlannedMotionSegment::DOCK_DEPARTURE);
  nav::ManageNavigationRoute management;management.request.invocation_id="second";
  management.request.route_id=request.response.route.route_id;management.request.task_deadline_s=Steady()+30;
  ASSERT_TRUE(manager.call(management));ASSERT_TRUE(management.response.accepted);
  nav::CommandBinding binding{"session","grant","goal",{"map",1,{0,0,0},{0,0,0,1}}};
  binding.route_id=management.request.route_id;binding.invocation_id="second";
  binding.task_deadline_s=management.request.task_deadline_s;
  ASSERT_FALSE(store.bind(binding));
  management.request.complete_departure=true;
  ASSERT_TRUE(manager.call(management));ASSERT_FALSE(management.response.accepted);
  measured={-1,0,0};ASSERT_TRUE(manager.call(management));ASSERT_FALSE(management.response.accepted);
  stopped=true;blocked=true;
  ASSERT_TRUE(manager.call(management));ASSERT_FALSE(management.response.accepted);ASSERT_FALSE(store.bind(binding));
  blocked=false;
  ASSERT_TRUE(manager.call(management));ASSERT_TRUE(management.response.accepted)<<management.response.reason;
  ASSERT_TRUE(store.bind(binding));std::size_t segment=0;ASSERT_TRUE(store.boundGoal(binding.endpoint,&segment));EXPECT_EQ(segment,1);
}
int main(int argc,char** argv) {
  ros::init(argc,argv,"route_preflight_ros");testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
