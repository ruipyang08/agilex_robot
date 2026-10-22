// Read-only stdin/stdout adapter for offline map experiments. No ROS publishers.
#include <agx_workbench_navigation/motion_search.hpp>
#include <iomanip>
#include <iostream>
#include <sstream>

int main(int argc,char** argv) {
  namespace nav=agx_workbench_navigation;
  nav::MotionGrid grid;nav::MotionGeometryPolicy geometry;nav::MotionRoute request;
  nav::MotionSearchOptions options;nav::MotionLimits limits;
  if(argc!=1) {
    if(argc!=5 || std::string(argv[1])!="--search-bins")return 1;
    std::istringstream values(std::string(argv[2])+" "+argv[3]+" "+argv[4]);
    std::string extra;
    if(!(values>>options.position_bin_m>>options.heading_bin_rad>>options.max_labels_per_bin) || values>>extra)return 1;
  }
  request.route_id="offline-preview";request.invocation_id="offline-preview";
  request.registry_digest="offline-unregistered";
  std::string protocol;
  if(!(std::cin>>protocol) || protocol!="AGX_MOTION_PREVIEW_V2")return 1;
  if(!(std::cin>>grid.width>>grid.height>>grid.resolution_m>>grid.origin_x_m>>grid.origin_y_m) ||
      grid.width==0 || grid.height==0 || grid.width>10000 || grid.height>10000 ||
      grid.width>1000000/grid.height)return 1;
  if(!(std::cin>>request.measured_start.x_m>>request.measured_start.y_m>>request.measured_start.yaw_rad
      >>request.goal.x_m>>request.goal.y_m>>request.goal.yaw_rad
      >>geometry.hard_padding_m>>geometry.soft_clearance_m>>geometry.translation_step_m>>geometry.rotation_step_rad
      >>request.model_digest>>request.map_digest>>options.timeout_s))return 1;
  if(!(std::cin>>limits.max_speed_mps>>limits.acceleration_mps2>>limits.deceleration_mps2
      >>limits.max_yaw_rate_radps>>limits.angular_acceleration_radps2>>limits.curvature_rate_per_m_s
      >>limits.lateral_acceleration_mps2>>limits.min_turn_radius_m>>limits.steering_settle_s>>limits.stop_dwell_s))return 1;
  std::size_t vertices=0;
  if(!(std::cin>>vertices) || vertices<3 || vertices>64)return 1;
  geometry.footprint.clear();geometry.footprint.resize(vertices);
  for(auto& point:geometry.footprint)if(!(std::cin>>point[0]>>point[1]))return 1;
  if(!nav::ValidMotionPolygon(geometry.footprint))return 1;
  request.coarse_start=request.measured_start;grid.cells.reserve(grid.width*grid.height);
  for(std::size_t i=0;i<grid.width*grid.height;++i) {
    int value;if(!(std::cin>>value) || value<0 || value>255)return 1;
    grid.cells.push_back(static_cast<unsigned char>(value));
  }
  std::string extra;if(std::cin>>extra)return 1;
  const auto result=nav::SearchMotionRoute(grid,geometry,limits,request,options);
  std::cout<<std::setprecision(17)<<"{\"preview_only\":true,\"status\":"<<static_cast<int>(result.result.status)
      <<",\"detail\":"<<std::quoted(result.result.detail)<<",\"elapsed_s\":"<<result.elapsed_s
      <<",\"expansions\":"<<result.expansions<<",\"generated\":"<<result.generated
      <<",\"budget_exhausted\":"<<(result.budget_exhausted?"true":"false")
      <<",\"budget_reason\":"<<std::quoted(result.budget_reason)
      <<",\"label_pruned\":"<<result.label_pruned<<",\"duplicate_rejected\":"<<result.duplicate_rejected
      <<",\"collision_rejected\":"<<result.collision_rejected
      <<",\"forward_generated\":"<<result.forward_generated<<",\"spin_generated\":"<<result.spin_generated
      <<",\"connector_attempts\":"<<result.connector_attempts
      <<",\"estimated_time_s\":"<<result.result.route.estimated_time_s<<",\"collision\":";
  if(result.collision_reported) {
    const auto& c=result.collision;
    std::cout<<"{\"kind\":"<<static_cast<int>(c.status)<<",\"cell_x\":"<<c.cell_x<<",\"cell_y\":"<<c.cell_y
        <<",\"pose\":["<<c.first_pose.x_m<<','<<c.first_pose.y_m<<','<<c.first_pose.yaw_rad<<"]}";
  } else std::cout<<"null";
  std::cout<<",\"model_digest\":"<<std::quoted(request.model_digest)
      <<",\"map_digest\":"<<std::quoted(request.map_digest)<<",\"footprint_xy_m\":[";
  for(std::size_t i=0;i<geometry.footprint.size();++i) {
    if(i)std::cout<<',';
    std::cout<<'['<<geometry.footprint[i][0]<<','<<geometry.footprint[i][1]<<']';
  }
  std::cout<<"],\"segments\":[";
  bool first_segment=true;
  for(const auto& segment:result.result.route.segments) {
    if(!first_segment)std::cout<<',';
    first_segment=false;
    std::cout<<"{\"kind\":"<<static_cast<int>(segment.kind)<<",\"estimated_time_s\":"<<segment.estimated_time_s
        <<",\"geometry_error_m\":"<<segment.geometry_error_m<<",\"samples\":[";
    bool first=true;
    for(const auto& s:segment.samples) {
      if(!first)std::cout<<',';
      first=false;
      std::cout<<'['<<s.pose.x_m<<','<<s.pose.y_m<<','<<s.pose.yaw_rad<<','<<s.curvature_per_m<<','
          <<s.speed_ceiling_mps<<','<<s.yaw_ceiling_radps<<','<<s.time_s<<']';
    }
    std::cout<<"]}";
  }
  std::cout<<"]}\n";
  return result.result.status==nav::RouteStatus::Ready?0:2;
}
