#include <agx_workbench_navigation/motion_evidence_ros.hpp>
#include <agx_workbench_navigation/motion_policy_io.hpp>
#include <agx_workbench_navigation/motion_plan_service.hpp>
#include <agx_workbench_navigation/ranger_global_planner.hpp>
#include <agx_workbench_navigation/LidarObstacleSnapshot.h>
#include <agx_workbench_navigation/ManageNavigationRoute.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <gtest/gtest.h>
#include <chrono>
#include <fstream>
#include <thread>
#include <unistd.h>
namespace n=agx_workbench_navigation;
double Steady() {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
TEST(MotionEvidence, OriginalCaptureRawMapAndStaleSource) {
  ros::NodeHandle node("~");ros::AsyncSpinner spinner(1);spinner.start();
  std::string policy,filter;ASSERT_TRUE(node.getParam("policy_file",policy));ASSERT_TRUE(node.getParam("lidar_file",filter));
  const std::string prefix="/tmp/navigation_evidence_"+std::to_string(::getpid());
  const auto map_path=prefix+".yaml",image_path=prefix+".pgm",registry_path=prefix+"_registry.yaml";
  std::string pixels(10000,'\xff');pixels[89*100+10]='\0';pixels[39*100+65]='\0';
  pixels[79*100+10]=static_cast<char>(205);
  std::ofstream(image_path)<<"P5\n100 100\n255\n"<<pixels;
  std::ofstream(map_path)<<"image: "<<image_path<<"\nresolution: 0.1\norigin: [-5, -5, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  std::ofstream(registry_path)<<"fixture: installation-a\n";
  n::MotionGrid raw;raw.width=raw.height=100;raw.resolution_m=.1;raw.origin_x_m=raw.origin_y_m=-5;raw.cells.assign(10000,0);
  auto config=n::LoadInstalledMotionPolicy(policy).preflight;
  bool raw_stale=false;
  n::RosMotionEvidence evidence(node,policy,map_path,registry_path,filter,[&] {
    const double captured=Steady()-(raw_stale?.5:0);
    return n::MotionCostmapSnapshot{raw,captured,captured+.25};
  },Steady);
  n::MotionRouteStore store(Steady);
  n::MotionPlanService service(node,config,store,[&]{return evidence.Snapshot();},Steady);
  auto client=node.serviceClient<n::PlanNavigationRoute>("plan_navigation_route");
  auto map_pub=node.advertise<nav_msgs::OccupancyGrid>("/map",1,true);
  auto pose_pub=node.advertise<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose",1);
  auto odom_pub=node.advertise<nav_msgs::Odometry>("/odom",1);
  auto cloud_pub=node.advertise<n::LidarObstacleSnapshot>("/workbench_nav/lidar_obstacle_snapshot",1);
  tf2_ros::StaticTransformBroadcaster fixed;tf2_ros::TransformBroadcaster moving;
  geometry_msgs::TransformStamped map_tf;map_tf.header.frame_id="map";map_tf.child_frame_id="odom";map_tf.transform.rotation.w=1;
  auto lidar_tf=map_tf;lidar_tf.header.frame_id="rslidar";lidar_tf.child_frame_id="base_link";
  fixed.sendTransform(std::vector<geometry_msgs::TransformStamped>{map_tf,lidar_tf});
  auto odom_tf=map_tf;odom_tf.header.frame_id="odom";odom_tf.child_frame_id="rslidar";odom_tf.transform.translation.x=1;
  nav_msgs::OccupancyGrid map;map.header.frame_id="map";map.info.width=map.info.height=100;
  map.info.resolution=.1;map.info.origin.position.x=map.info.origin.position.y=-5;map.info.origin.orientation.w=1;map.data.assign(10000,0);
  map.data[10*100+10]=100;
  map.data[60*100+65]=100; // inside legacy circle, outside the shared travel sweep
  map.data[20*100+10]=-1;
  map_pub.publish(map);
  geometry_msgs::PoseWithCovarianceStamped pose;pose.header.frame_id="map";pose.pose.pose.position.x=1;pose.pose.pose.orientation.w=1;
  nav_msgs::Odometry odom;odom.header.frame_id="odom";odom.child_frame_id="rslidar";odom.pose.pose=pose.pose.pose;
  n::LidarObstacleSnapshot cloud;cloud.header.frame_id="base_link";cloud.source_frame="rslidar";cloud.state=cloud.CLEAR;
  cloud.policy_digest=n::MotionFileDigest(policy);cloud.filter_digest=n::MotionFileDigest(filter);
  cloud.capture_pose.header.frame_id="odom";cloud.capture_pose.pose=pose.pose.pose;
  cloud.obstacle_points.header.frame_id="base_link";
  sensor_msgs::PointCloud2Modifier mod(cloud.obstacle_points);mod.setPointCloud2Fields(3,"x",1,sensor_msgs::PointField::FLOAT32,"y",1,sensor_msgs::PointField::FLOAT32,"z",1,sensor_msgs::PointField::FLOAT32);mod.resize(1);
  *sensor_msgs::PointCloud2Iterator<float>(cloud.obstacle_points,"x")=1;
  *sensor_msgs::PointCloud2Iterator<float>(cloud.obstacle_points,"y")=2;
  *sensor_msgs::PointCloud2Iterator<float>(cloud.obstacle_points,"z")=.5;
  auto publish=[&](bool sensor) {
    const auto now=ros::Time::now();pose.header.stamp=odom.header.stamp=odom_tf.header.stamp=now;
    moving.sendTransform(odom_tf);pose_pub.publish(pose);odom_pub.publish(odom);
    if(sensor) {
      cloud.header.stamp=cloud.transform_stamp=cloud.capture_pose.header.stamp=cloud.obstacle_points.header.stamp=now;
      cloud.receipt_steady_s=Steady();++cloud.cloud_sequence;cloud_pub.publish(cloud);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  };
  n::MotionPlanningEvidence snapshot;
  const double end=Steady()+4;
  while(Steady()<end) {publish(true);snapshot=evidence.Snapshot();if(!snapshot.grid.cells.empty())break;}
  ASSERT_FALSE(snapshot.grid.cells.empty());ASSERT_EQ(snapshot.grid.cells[70*100+70],254);
  ASSERT_EQ(snapshot.grid.cells[50*100+60],0);
  ASSERT_EQ(snapshot.grid.cells[10*100+10],254); // raw obstacle free cannot erase static cell
  ASSERT_EQ(snapshot.grid.cells[20*100+10],255); // unknown remains unknown
  ASSERT_GT(snapshot.pose_deadline_s,Steady());ASSERT_GT(snapshot.obstacle_deadline_s,Steady());
  // Odometry and its original TF travel independently. A newer odometry
  // callback cannot invalidate a still-fresh joined sample while TF catches up.
  const double joined_deadline=snapshot.pose_deadline_s;
  odom.header.stamp=ros::Time::now();odom.pose.pose.position.x=1.003;
  odom_pub.publish(odom);std::this_thread::sleep_for(std::chrono::milliseconds(25));
  snapshot=evidence.Snapshot();
  ASSERT_FALSE(snapshot.grid.cells.empty())<<snapshot.unavailable_detail;
  ASSERT_NEAR(snapshot.measured_pose.x_m,1,1e-9);
  // ROS and steady clock reads are separate; allow sub-millisecond read
  // skew, but not the new sample's 25 ms renewal. Runtime leases are unchanged.
  ASSERT_NEAR(snapshot.pose_deadline_s,joined_deadline,.001);
  odom_tf.header.stamp=odom.header.stamp;odom_tf.transform.translation.x=1.003;
  moving.sendTransform(odom_tf);std::this_thread::sleep_for(std::chrono::milliseconds(25));
  snapshot=evidence.Snapshot();ASSERT_FALSE(snapshot.grid.cells.empty())<<snapshot.unavailable_detail;
  ASSERT_NEAR(snapshot.measured_pose.x_m,1.003,1e-9);
  odom.pose.pose.position.x=odom_tf.transform.translation.x=1;publish(true);
  odom.header.stamp=ros::Time::now();odom_pub.publish(odom);
  std::this_thread::sleep_for(std::chrono::milliseconds(270));
  snapshot=evidence.Snapshot();ASSERT_TRUE(snapshot.grid.cells.empty());
  ASSERT_NE(snapshot.unavailable_detail.find("original evidence is stale"),std::string::npos)
      <<snapshot.unavailable_detail; // Missing TF cannot keep the old join alive.
  publish(true);snapshot=evidence.Snapshot();ASSERT_FALSE(snapshot.grid.cells.empty());
  n::PlanNavigationRoute plan;plan.request.invocation_id="fixture";plan.request.model_digest=config.model_digest;
  plan.request.registry_digest=n::MotionFileDigest(registry_path);
  for(auto* p:{&plan.request.measured_start,&plan.request.coarse_start,&plan.request.goal}) {
    p->header.frame_id="map";p->header.stamp=ros::Time::now();p->pose.position.x=1;p->pose.orientation.w=1;
  }
  plan.request.goal.pose.position.x=2;
  ASSERT_TRUE(client.call(plan));ASSERT_EQ(plan.response.status,plan.response.READY);
  ASSERT_EQ(plan.response.route.map_digest,snapshot.static_map_digest);
  const auto expected_map=n::MotionBytesDigest(n::MotionFileDigest(map_path).substr(7)+"|"+n::MotionFileDigest(image_path).substr(7));
  ASSERT_EQ(snapshot.static_map_digest,expected_map);
  {
    // The actual nav_core plugin must own the live service, not just the test
    // service above. Keep source callbacks advancing during costmap startup.
    // Execution is explicitly experimental and uses a separate source identity.
    const auto candidate_path=prefix+"_candidate.yaml";
    std::ifstream input(policy);std::string candidate((std::istreambuf_iterator<char>(input)),{});
    for(const auto& change:std::vector<std::pair<std::string,std::string>>{
        {"schema_version: 1","schema_version: 2"},
        {"execution_enabled: false","execution_enabled: true"},
        {"qualification: unqualified","qualification: experimental_simulation"}}) {
      const auto index=candidate.find(change.first);ASSERT_NE(index,std::string::npos);
      candidate.replace(index,change.first.size(),change.second);
    }
    candidate+="\nfootprint_provenance:\n  scope: diagnostic_candidate\n  source_sha256: {fixture.json: '"+std::string(64,'a')+"'}\n";
    std::ofstream(candidate_path)<<candidate;
    const auto candidate_digest=n::MotionFileDigest(candidate_path);
    cloud.policy_digest=candidate_digest;
    node.setParam("RangerGlobalPlanner/motion_policy_file",candidate_path);
    node.setParam("RangerGlobalPlanner/route_map_file",map_path);
    node.setParam("RangerGlobalPlanner/route_registry_file",registry_path);
    node.setParam("RangerGlobalPlanner/route_lidar_config_file",filter);
    std::atomic<bool> feeding{true};
    std::thread feed([&]{while(feeding)publish(true);});
    struct Join {std::atomic<bool>& run;std::thread& thread;~Join(){run=false;thread.join();}} join{feeding,feed};
    tf2_ros::Buffer buffer;tf2_ros::TransformListener listener(buffer);
    costmap_2d::Costmap2DROS costmap("evidence_costmap",buffer);
    // Configuration faults must not silently select another planning algorithm.
    node.setParam("MissingPolicy/turn_penalty_m",.1);
    node.setParam("MissingPolicy/goal_heading_penalty_m",1.);
    node.setParam("MissingPolicy/traversal_cost_scale",.1);
    node.setParam("MissingPolicy/lethal_cost",253);
    node.setParam("MissingPolicy/allow_unknown",false);
    n::RangerGlobalPlanner missing;
    EXPECT_THROW(missing.initialize("MissingPolicy",&costmap),std::runtime_error);
    node.setParam("MissingPolicy/motion_policy_file",prefix+"_absent.yaml");
    n::RangerGlobalPlanner malformed;
    EXPECT_THROW(malformed.initialize("MissingPolicy",&costmap),std::runtime_error);
    n::RangerGlobalPlanner global("RangerGlobalPlanner",&costmap);
    auto actual=node.serviceClient<n::PlanNavigationRoute>("RangerGlobalPlanner/plan_navigation_route");
    ASSERT_TRUE(actual.waitForExistence(ros::Duration(2)));
    plan.request.invocation_id="actual-plugin";plan.request.model_digest=candidate_digest;
    const double deadline=Steady()+2;
    do {
      ASSERT_TRUE(actual.call(plan));
      if(plan.response.status==plan.response.READY)break;
      ASSERT_EQ(plan.response.status,plan.response.EVIDENCE_UNAVAILABLE)<<plan.response.detail;
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
    } while(Steady()<deadline);
    ASSERT_EQ(plan.response.status,plan.response.READY)<<plan.response.detail;
    ASSERT_EQ(plan.response.route.model_digest,candidate_digest);
    ASSERT_EQ(plan.response.route.registry_digest,n::MotionFileDigest(registry_path));
    for(const auto& segment:plan.response.route.segments)for(const auto& point:segment.samples)
      ASSERT_NEAR(point.pose.y,0,1e-9); // travel route is not vetoed by the old hard circle
    std::vector<geometry_msgs::PoseStamped> delivered;
    auto goal=plan.request.goal;goal.header.stamp=ros::Time::now();
    ASSERT_FALSE(global.makePlan(plan.request.measured_start,goal,delivered));
    auto manager=node.serviceClient<n::ManageNavigationRoute>("RangerGlobalPlanner/manage_navigation_route");
    n::ManageNavigationRoute manage;manage.request.invocation_id="actual-plugin";
    manage.request.route_id=plan.response.route.route_id;manage.request.task_deadline_s=Steady()+5;
    ASSERT_TRUE(manager.call(manage));ASSERT_TRUE(manage.response.accepted)<<manage.response.reason;
    const auto& gp=goal.pose.position;const auto& gq=goal.pose.orientation;
    n::CommandBinding binding{"session","grant","source",
        {"map",goal.header.stamp.toNSec(),{gp.x,gp.y,gp.z},{gq.x,gq.y,gq.z,gq.w}},
        manage.request.invocation_id,manage.request.route_id,manage.request.task_deadline_s};
    ASSERT_TRUE(n::SharedMotionRouteStore().bind(binding));
    const double cache_deadline=Steady()+.5;
    while(Steady()<cache_deadline) {
      const auto source=n::SharedMotionEvidence();
      if(source && source->unavailable_detail.empty() && !source->grid.cells.empty() &&
         source->pose_deadline_s>Steady() && source->obstacle_deadline_s>Steady())break;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    const auto source=n::SharedMotionEvidence();ASSERT_TRUE(source);
    ASSERT_TRUE(source->unavailable_detail.empty())<<source->unavailable_detail;
    ASSERT_GT(source->pose_deadline_s,Steady());ASSERT_GT(source->obstacle_deadline_s,Steady());
    ASSERT_TRUE(global.makePlan(plan.request.measured_start,goal,delivered));
    ASSERT_EQ(delivered.back().header.stamp,goal.header.stamp);
    const auto count=delivered.size();
    ASSERT_TRUE(global.makePlan(plan.request.measured_start,goal,delivered));
    ASSERT_EQ(delivered.size(),count); // Delivery cannot reset or replace progress.
    auto foreign=goal;foreign.header.stamp+=ros::Duration(.001);
    ASSERT_FALSE(global.makePlan(plan.request.measured_start,foreign,delivered));
    manage.request.release=true;ASSERT_TRUE(manager.call(manage));ASSERT_TRUE(manage.response.accepted);
    ASSERT_FALSE(global.makePlan(plan.request.measured_start,goal,delivered));
    std::remove(candidate_path.c_str());
  }
  cloud.policy_digest=n::MotionFileDigest(policy);plan.request.model_digest=config.model_digest;publish(true);
  const double original=cloud.receipt_steady_s;
  for(int i=0;i<18;++i) {publish(false);cloud_pub.publish(cloud);} // repeats cannot renew source
  snapshot=evidence.Snapshot();ASSERT_TRUE(snapshot.grid.cells.empty());ASSERT_EQ(cloud.receipt_steady_s,original);
  plan.request.existing_route_id=plan.response.route.route_id;
  ASSERT_TRUE(client.call(plan));ASSERT_EQ(plan.response.status,plan.response.EVIDENCE_UNAVAILABLE);
  publish(true);ASSERT_FALSE(evidence.Snapshot().grid.cells.empty());
  raw.cells[50*100+70]=253;
  ASSERT_TRUE(evidence.Snapshot().grid.cells.empty()); // inflation is not accepted as raw input
  raw.cells[50*100+70]=0;
  publish(true);ASSERT_FALSE(evidence.Snapshot().grid.cells.empty());
  raw_stale=true;ASSERT_TRUE(evidence.Snapshot().grid.cells.empty());
  raw_stale=false;ASSERT_FALSE(evidence.Snapshot().grid.cells.empty());
  cloud.state=cloud.INVALID;cloud_pub.publish(cloud);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  const auto invalid_capture=evidence.Snapshot();
  ASSERT_TRUE(invalid_capture.grid.cells.empty());
  ASSERT_NE(invalid_capture.unavailable_detail.find("invalid sensor health"),std::string::npos)
      <<invalid_capture.unavailable_detail;
  cloud.state=cloud.CLEAR;publish(true);ASSERT_FALSE(evidence.Snapshot().grid.cells.empty());
  // A repeated source with edited capture geometry must invalidate, not renew.
  cloud.capture_pose.pose.position.x+=1;cloud_pub.publish(cloud);
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  ASSERT_TRUE(evidence.Snapshot().grid.cells.empty());
  cloud.capture_pose.pose.position.x-=1;publish(true);ASSERT_FALSE(evidence.Snapshot().grid.cells.empty());
  mod.resize(0);publish(true);ASSERT_FALSE(evidence.Snapshot().grid.cells.empty());
  ASSERT_EQ(evidence.Snapshot().grid.cells[70*100+70],0); // valid empty filtered cloud
  // Exact source files are checked on every decision, not only startup.
  publish(true);std::ofstream(registry_path,std::ios::app)<<"changed: true\n";
  ASSERT_TRUE(evidence.Snapshot().grid.cells.empty());
  std::remove(map_path.c_str());std::remove(image_path.c_str());std::remove(registry_path.c_str());
}
int main(int argc,char** argv) {ros::init(argc,argv,"motion_evidence_ros");testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();}
