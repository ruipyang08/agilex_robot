#include <agx_workbench_navigation/ranger_global_planner.hpp>
#include <agx_workbench_navigation/effective_planner_config.hpp>
#include <agx_workbench_navigation/motion_evidence_ros.hpp>
#include <agx_workbench_navigation/motion_policy_io.hpp>
#include <agx_workbench_navigation/motion_plan_service.hpp>

#include <cmath>
#include <chrono>
#include <stdexcept>
#include <utility>

#include <boost/thread/locks.hpp>
#include <costmap_2d/static_layer.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/inflation_layer.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <std_msgs/String.h>

namespace agx_workbench_navigation {
namespace {

double RouteSteady() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
template <typename T>
void requireParam(const ros::NodeHandle& node, const std::string& name, T* value) {
  if (!node.getParam(name, *value)) {
    throw std::runtime_error(
        "missing required RangerGlobalPlanner parameter: " + name);
  }
}

}  // namespace

RangerGlobalPlanner::RangerGlobalPlanner()
    : initialized_(false), costmap_ros_(nullptr) {}

RangerGlobalPlanner::~RangerGlobalPlanner() {
  route_costmap_timer_.stop();
  route_service_.reset();
  route_evidence_.reset();
  PublishSharedMotionEvidence({});
}

RangerGlobalPlanner::RangerGlobalPlanner(
    std::string name,
    costmap_2d::Costmap2DROS* costmap_ros)
    : RangerGlobalPlanner() {
  initialize(std::move(name), costmap_ros);
}

void RangerGlobalPlanner::initialize(
    std::string name,
    costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("RangerGlobalPlanner has already been initialized");
    return;
  }
  if (costmap_ros == nullptr) {
    throw std::invalid_argument("RangerGlobalPlanner requires a costmap");
  }
  ros::NodeHandle private_node("~/" + name);
  std::string policy_path;
  requireParam(private_node,"motion_policy_file",&policy_path);
  route_policy_.reset(new InstalledMotionPolicy(LoadInstalledMotionPolicy(policy_path)));
  costmap_ros_ = costmap_ros;
  ros::NodeHandle plan_node("~/GlobalPlanner");
  global_plan_publisher_ =
      plan_node.advertise<nav_msgs::Path>("plan", 1, true);
  initialized_ = true;
  effective_config_session_=NewEffectivePlannerSession();
  effective_config_publisher_=private_node.advertise<std_msgs::String>(
      "/workbench_nav/effective_config/global_planner",1,true);
  publishEffectiveConfiguration(ros::WallTimerEvent());
  effective_config_timer_=private_node.createWallTimer(ros::WallDuration(.2),
      &RangerGlobalPlanner::publishEffectiveConfiguration,this);
  // Admission assembles original evidence; makePlan presents only a bound route.
  try {
    std::string map_path,registry_path,lidar_path;
    requireParam(private_node,"route_map_file",&map_path);
    requireParam(private_node,"route_registry_file",&registry_path);
    requireParam(private_node,"route_lidar_config_file",&lidar_path);
    route_evidence_.reset(new RosMotionEvidence(private_node,policy_path,map_path,registry_path,lidar_path,
        [this]{return routeCostmapSnapshot();},RouteSteady));
  } catch(const std::exception& error) {route_source_error_=error.what();}
  captureRouteCostmap(ros::WallTimerEvent());
  route_costmap_timer_=private_node.createWallTimer(ros::WallDuration(.05),
      &RangerGlobalPlanner::captureRouteCostmap,this);
  route_service_.reset(new MotionPlanService(private_node,route_policy_->preflight,SharedMotionRouteStore(),[this] {
    if(route_evidence_)return route_evidence_->Snapshot();
    MotionPlanningEvidence unavailable;unavailable.unavailable_detail=route_source_error_;return unavailable;
  },RouteSteady));
}

void RangerGlobalPlanner::captureRouteCostmap(const ros::WallTimerEvent&) {
  std::shared_ptr<const MotionGrid> snapshot;
  const double receipt=RouteSteady();
  try {
    if(!costmap_ros_ || costmap_ros_->getGlobalFrameID()!="map" ||
        costmap_ros_->getLayeredCostmap()->isRolling())throw std::runtime_error("route requires nonrolling map costmap");
    auto* master=costmap_ros_->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*master->getMutex());
    if(!costmap_ros_->isCurrent())throw std::runtime_error("raw obstacle costmap is not current");
    MotionGrid grid;grid.width=master->getSizeInCellsX();grid.height=master->getSizeInCellsY();
    grid.resolution_m=master->getResolution();grid.origin_x_m=master->getOriginX();grid.origin_y_m=master->getOriginY();
    if(grid.width==0 || grid.height==0 || grid.width>1000000/grid.height)
      throw std::runtime_error("route costmap exceeds cell bound");
    grid.cells.assign(grid.width*grid.height,0);
    std::size_t static_layers=0,obstacle_layers=0;
    // This timer runs on move_base's owner queue, serialized with static-map
    // callbacks and reconfiguration. The master mutex excludes updateMap;
    // obstacle-layer mutexes also exclude resetMaps. Do not read StaticLayer's
    // bytes from the search thread: its map callbacks do not take that mutex.
    for(const auto& layer:*costmap_ros_->getLayeredCostmap()->getPlugins()) {
      if(auto* obstacle=dynamic_cast<costmap_2d::ObstacleLayer*>(layer.get())) {
        if(!layer->isEnabled())throw std::runtime_error("raw obstacle layer disabled");
        ++obstacle_layers;
        boost::unique_lock<costmap_2d::Costmap2D::mutex_t> layer_lock(*obstacle->getMutex());
        if(obstacle->getSizeInCellsX()!=grid.width || obstacle->getSizeInCellsY()!=grid.height ||
            obstacle->getResolution()!=grid.resolution_m || obstacle->getOriginX()!=grid.origin_x_m ||
            obstacle->getOriginY()!=grid.origin_y_m)throw std::runtime_error("raw obstacle layer geometry mismatch");
        const auto* cells=obstacle->getCharMap();
        for(std::size_t i=0;i<grid.cells.size();++i) {
          if(cells[i]!=0 && cells[i]!=254 && cells[i]!=255)throw std::runtime_error("raw obstacle layer contains inflated costs");
          if(cells[i]==254)grid.cells[i]=254;
        }
      } else if(dynamic_cast<costmap_2d::StaticLayer*>(layer.get())) {
        if(!layer->isEnabled())throw std::runtime_error("static map layer disabled");++static_layers;
      } else if(!dynamic_cast<costmap_2d::InflationLayer*>(layer.get())) {
        throw std::runtime_error("unsupported route costmap layer");
      }
    }
    if(static_layers!=1 || obstacle_layers!=1)throw std::runtime_error("route requires one static and one raw obstacle layer");
    snapshot=std::make_shared<const MotionGrid>(std::move(grid));
  } catch(const std::exception&) {}
  {
    std::lock_guard<std::mutex> cache(route_costmap_mutex_);
    route_costmap_=std::move(snapshot);route_costmap_receipt_s_=receipt;
  }
  // Copy outside the cache lock: Snapshot reads that cache itself. Its original
  // sensor deadlines survive publication; control readers cannot renew them.
  try {
    PublishSharedMotionEvidence(route_evidence_?route_evidence_->Snapshot():MotionPlanningEvidence{});
  } catch(const std::exception&) {PublishSharedMotionEvidence({});}
}

MotionCostmapSnapshot RangerGlobalPlanner::routeCostmapSnapshot() {
  std::lock_guard<std::mutex> lock(route_costmap_mutex_);
  const double age=RouteSteady()-route_costmap_receipt_s_;
  if(!route_costmap_ || age<0 || age>=.25)throw std::runtime_error("owned raw costmap snapshot unavailable or stale");
  return {*route_costmap_,route_costmap_receipt_s_,route_costmap_receipt_s_+.25};
}

void RangerGlobalPlanner::publishEffectiveConfiguration(const ros::WallTimerEvent&) {
  if(!initialized_ || !costmap_ros_) return;
  try {
    // Stock move_base's global ros::spin() serializes this WallTimer with
    // footprint/reconfigure callbacks; their setter does not take this mutex.
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*costmap_ros_->getCostmap()->getMutex());
    std_msgs::String report;
    report.data=GlobalPlannerEffectiveReport(effective_config_session_,
        EffectivePlannerFootprint(costmap_ros_->getRobotFootprint()));
    effective_config_publisher_.publish(report);
  } catch(const std::exception& error) {
    ROS_ERROR_STREAM_THROTTLE(1.0,"Cannot report global planner effective configuration: " << error.what());
  }
}

bool RangerGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.clear();
  if (!initialized_ || costmap_ros_ == nullptr) {
    ROS_ERROR("RangerGlobalPlanner::makePlan called before initialize");
    return false;
  }
  const std::string& global_frame = costmap_ros_->getGlobalFrameID();
  if (start.header.frame_id != global_frame || goal.header.frame_id != global_frame) {
    ROS_ERROR_THROTTLE(
        1.0, "RangerGlobalPlanner requires start and goal in %s",
        global_frame.c_str());
    return false;
  }

  {
    const auto& p=goal.pose.position;const auto& q=goal.pose.orientation;
    std::size_t segment=0,sample=0;
    const auto route=SharedMotionRouteStore().boundGoal(
        {goal.header.frame_id,goal.header.stamp.toNSec(),{p.x,p.y,p.z},{q.x,q.y,q.z,q.w}},&segment,&sample);
    const auto evidence=SharedMotionEvidence();const double now=RouteSteady();
    const auto block=[&](const std::string& reason) {
      if(route)SharedMotionRouteStore().block(route->route_id,route->invocation_id,reason);
      return false;
    };
    if(!route || !route_policy_->execution_enabled || !evidence ||
       !evidence->unavailable_detail.empty() || evidence->pose_deadline_s<=now ||
       evidence->obstacle_deadline_s<=now || evidence->observed_at_s>now ||
       route->model_digest!=route_policy_->preflight.model_digest ||
       route->map_digest!=evidence->static_map_digest || route->registry_digest!=evidence->registry_digest)
      return block("route_sources_stale_or_changed"+std::string(
          evidence && !evidence->unavailable_detail.empty()?": "+evidence->unavailable_detail:""));
    // Revalidate only the unexecuted typed route; never search for another
    // branch or reinterpret progress from a nearby loop in the path.
    MotionSweepChecker checker(evidence->grid);
    for(std::size_t i=segment;i<route->segments.size();++i) {
      const auto& part=route->segments[i];std::vector<MotionPose> poses;
      for(std::size_t j=i==segment?sample:0;j<part.samples.size();++j)poses.push_back(part.samples[j].pose);
      auto geometry=route_policy_->preflight.geometry;geometry.integration_error_m=part.geometry_error_m;
      if(!checker.Check(geometry,poses).clear())return block("remaining_route_blocked");
      for(const auto& point:poses) {
        geometry_msgs::PoseStamped pose;pose.header.frame_id=global_frame;pose.header.stamp=ros::Time::now();
        pose.pose.position.x=point.x_m;pose.pose.position.y=point.y_m;
        pose.pose.orientation.z=std::sin(point.yaw_rad/2);pose.pose.orientation.w=std::cos(point.yaw_rad/2);
        plan.push_back(pose);
      }
    }
    plan.push_back(goal); // Preserve the exact operation nonce.
    nav_msgs::Path message;message.header.frame_id=global_frame;message.header.stamp=ros::Time::now();
    message.poses=plan;global_plan_publisher_.publish(message);return true;
  }

  return false;
}

}  // namespace agx_workbench_navigation

PLUGINLIB_EXPORT_CLASS(
    agx_workbench_navigation::RangerGlobalPlanner,
    nav_core::BaseGlobalPlanner)
