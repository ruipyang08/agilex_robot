#pragma once
#include <agx_workbench_navigation/motion_geometry.hpp>
#include <cstdint>
#include <string>
namespace agx_workbench_navigation {
struct MotionCaptureTransform {
  std::array<double,3> position{{0,0,0}};
  std::array<double,4> orientation{{0,0,0,1}};
};
struct CapturedMotionCloud {
  std::uint32_t sequence=0;
  std::uint64_t source_stamp_ns=0;
  double source_stamp_s=0,transform_stamp_s=0,receipt_steady_s=0;
  bool transform_static=false;
  std::uint8_t state=0;
  std::string source_frame,target_frame,pose_frame,model_digest,filter_digest;
  MotionCaptureTransform capture_pose;
  std::vector<std::array<double,3>> points;
};
struct MotionObstacleOverlay {MotionGrid grid;double deadline_s=0;};
double MotionEvidenceDeadline(double steady_now,double source_now,double source_stamp,
    double original_receipt,double source_age_limit,double receipt_age_limit);
MotionObstacleOverlay OverlayCapturedObstacles(MotionGrid grid,const CapturedMotionCloud& cloud,
    const MotionCaptureTransform& map_from_odom,double source_now,double steady_now,
    const std::string& model_digest,const std::string& filter_digest,double max_source_age_s=.35);
} // namespace agx_workbench_navigation
