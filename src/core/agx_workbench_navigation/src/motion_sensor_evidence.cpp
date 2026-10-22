#include <agx_workbench_navigation/motion_sensor_evidence.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace agx_workbench_navigation {
namespace {
void Require(bool value,const char* reason) {if(!value)throw std::invalid_argument(reason);}
bool Finite(double value) {return std::isfinite(value);}
bool TransformValid(const MotionCaptureTransform& pose) {
  double norm=0;
  for(const auto v:pose.position)if(!Finite(v))return false;
  for(const auto v:pose.orientation) {if(!Finite(v))return false;norm+=v*v;}
  return Finite(norm) && std::abs(norm-1)<=1e-6;
}
std::array<double,3> Apply(const MotionCaptureTransform& t,const std::array<double,3>& p) {
  const auto& q=t.orientation;const double x=q[0],y=q[1],z=q[2],w=q[3];
  return {{t.position[0]+(1-2*(y*y+z*z))*p[0]+2*(x*y-z*w)*p[1]+2*(x*z+y*w)*p[2],
           t.position[1]+2*(x*y+z*w)*p[0]+(1-2*(x*x+z*z))*p[1]+2*(y*z-x*w)*p[2],
           t.position[2]+2*(x*z-y*w)*p[0]+2*(y*z+x*w)*p[1]+(1-2*(x*x+y*y))*p[2]}};
}
}
double MotionEvidenceDeadline(double steady_now,double source_now,double source_stamp,
    double original_receipt,double source_age_limit,double receipt_age_limit) {
  Require(Finite(steady_now) && steady_now>=0 && Finite(source_now) && source_now>0 &&
      Finite(source_stamp) && source_stamp>0 && Finite(original_receipt) && original_receipt>=0 &&
      Finite(source_age_limit) && source_age_limit>0 && Finite(receipt_age_limit) && receipt_age_limit>0,
      "invalid evidence clock or age limit");
  const double source_age=source_now-source_stamp,receipt_age=steady_now-original_receipt;
  Require(source_age>=0 && source_age<source_age_limit && receipt_age>=0 && receipt_age<receipt_age_limit,
      "original evidence is stale, future-dated or clock-regressed");
  const double deadline=steady_now+std::min(source_age_limit-source_age,receipt_age_limit-receipt_age);
  Require(Finite(deadline) && deadline>steady_now,"invalid evidence deadline");
  return deadline;
}
MotionObstacleOverlay OverlayCapturedObstacles(MotionGrid grid,const CapturedMotionCloud& cloud,
    const MotionCaptureTransform& map_from_odom,double source_now,double steady_now,
    const std::string& model_digest,const std::string& filter_digest,double max_source_age_s) {
  Require(ValidMotionGrid(grid) && grid.cells.size()<=1000000,"invalid or oversized map");
  Require(cloud.state<=1 && cloud.source_frame=="rslidar" && cloud.target_frame=="base_link" &&
      cloud.pose_frame=="odom" && !model_digest.empty() && !filter_digest.empty() &&
      cloud.model_digest==model_digest && cloud.filter_digest==filter_digest,
      "invalid sensor health, frame or installed source identity");
  Require(TransformValid(cloud.capture_pose) && TransformValid(map_from_odom),"invalid capture transform");
  Require(cloud.points.size()<=250000,"obstacle evidence exceeds bound");
  double deadline=MotionEvidenceDeadline(steady_now,source_now,cloud.source_stamp_s,
      cloud.receipt_steady_s,max_source_age_s,1.);
  Require(Finite(cloud.transform_stamp_s) && cloud.transform_stamp_s>=0 &&
      (cloud.transform_stamp_s>0 || cloud.transform_static),"invalid original sensor transform stamp");
  if(cloud.transform_stamp_s>0)deadline=std::min(deadline,MotionEvidenceDeadline(steady_now,source_now,
      cloud.transform_stamp_s,cloud.receipt_steady_s,max_source_age_s,1.));
  for(const auto& point:cloud.points) {
    for(const auto value:point)Require(Finite(value),"nonfinite filtered obstacle");
    const auto world=Apply(map_from_odom,Apply(cloud.capture_pose,point));
    for(const auto value:world)Require(Finite(value),"transformed obstacle overflows");
    const double x=(world[0]-grid.origin_x_m)/grid.resolution_m;
    const double y=(world[1]-grid.origin_y_m)/grid.resolution_m;
    // Outside-map cells already reject every swept footprint; never clamp an
    // outside obstacle into a free border cell or silently truncate the cloud.
    if(x>=0 && y>=0 && x<grid.width && y<grid.height)
      grid.cells[static_cast<std::size_t>(std::floor(y))*grid.width+
                 static_cast<std::size_t>(std::floor(x))]=254;
  }
  return {std::move(grid),deadline};
}
} // namespace agx_workbench_navigation
