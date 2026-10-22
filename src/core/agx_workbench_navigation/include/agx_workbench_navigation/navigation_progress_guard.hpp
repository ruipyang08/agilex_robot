#pragma once
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace agx_workbench_navigation {
struct NavigationProgressPose {
  std::uint64_t revision=0, stamp_ns=0;
  double stamp_s=0, x=0, y=0, yaw=0;
};

// One caller-serialized operation. Only CommandSourceGate's new-binding reset
// calls Begin; replans and control/limiter resets never touch this state.
class NavigationProgressGuard {
 public:
  NavigationProgressGuard(double timeout_s,double translation_m,double rotation_rad)
      : timeout_(timeout_s),translation_(translation_m),rotation_(rotation_rad) {
    if(!std::isfinite(timeout_) || timeout_<=0 || !std::isfinite(translation_) || translation_<=0 ||
       !std::isfinite(rotation_) || rotation_<=0 || rotation_>=std::acos(-1.))
      throw std::invalid_argument("navigation progress requires finite positive timeout/displacements and rotation below pi");
  }
  void Begin(double source_s,double receipt_s) {
    begun_=true;faulted_=false;anchored_=false;detail_.clear();
    start_=last_progress_=last_source_=source_s;last_receipt_=receipt_s;
    anchor_={};last_={};translation_delta_=rotation_delta_=0;
    if(!std::isfinite(source_s) || source_s<=0 || !std::isfinite(receipt_s) || receipt_s<0 ||
       !std::isfinite(source_s+timeout_) || source_s+timeout_<=source_s)
      Fail("invalid_start_clock",source_s);
  }
  bool Check(double source_s,double receipt_s) {
    if(!begun_ || faulted_)return false;
    if(!std::isfinite(source_s) || !std::isfinite(receipt_s) ||
       source_s<last_source_ || receipt_s<last_receipt_)return Fail("clock_regression_or_nonfinite",source_s);
    last_source_=source_s;last_receipt_=receipt_s;
    if(source_s-last_progress_>=timeout_)return Fail("no_progress",source_s);
    return true;
  }
  bool Observe(const NavigationProgressPose& pose,double source_s,double receipt_s) {
    if(!Check(source_s,receipt_s))return false;
    if(pose.revision==0 || pose.stamp_ns==0 || !std::isfinite(pose.stamp_s) || pose.stamp_s<=0 ||
       !std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.yaw))
      return Fail("invalid_pose",source_s);
    // A tolerated future canonical sample grants no future progress time. Wait
    // for its original source stamp; the existing canonical lease still applies.
    if(pose.stamp_s>source_s)return anchored_;
    if(pose.stamp_s<start_)return false;
    if(anchored_) {
      if(pose.revision<last_.revision || pose.stamp_ns<last_.stamp_ns || pose.stamp_s<last_.stamp_s)
        return Fail("pose_order_regression",source_s);
      if(pose.revision==last_.revision) {
        if(pose.stamp_ns!=last_.stamp_ns || pose.stamp_s!=last_.stamp_s ||
           pose.x!=last_.x || pose.y!=last_.y || pose.yaw!=last_.yaw)
          return Fail("contradictory_pose_repeat",source_s);
        return true; // No new anchor, receipt or progress allowance.
      }
      if(pose.stamp_ns<=last_.stamp_ns || pose.stamp_s<=last_.stamp_s)
        return Fail("pose_stamp_not_advancing",source_s);
    }
    last_=pose;
    if(!anchored_) {anchor_=pose;anchored_=true;return true;}
    translation_delta_=std::hypot(pose.x-anchor_.x,pose.y-anchor_.y);
    rotation_delta_=std::abs(std::remainder(pose.yaw-anchor_.yaw,2*std::acos(-1.)));
    if(!std::isfinite(translation_delta_) || !std::isfinite(rotation_delta_))
      return Fail("invalid_pose_displacement",source_s);
    if(translation_delta_>=translation_ || rotation_delta_>=rotation_) {
      anchor_=pose;last_progress_=pose.stamp_s;
    }
    return true;
  }
  bool faulted() const {return faulted_;}
  const std::string& detail() const {return detail_;}
 private:
  bool Fail(const char* reason,double now) {
    if(!faulted_) {
      faulted_=true;
      std::ostringstream out;out<<std::setprecision(17)<<"navigation_progress_"<<reason
          <<" source="<<now<<" last_progress="<<last_progress_<<" timeout="<<timeout_
          <<" anchor_stamp_ns="<<anchor_.stamp_ns<<" current_stamp_ns="<<last_.stamp_ns
          <<" translation="<<translation_delta_<<" translation_bound="<<translation_
          <<" rotation="<<rotation_delta_<<" rotation_bound="<<rotation_;
      detail_=out.str();
    }
    return false;
  }
  const double timeout_,translation_,rotation_;
  bool begun_=false,faulted_=false,anchored_=false;
  double start_=0,last_progress_=0,last_source_=0,last_receipt_=0;
  double translation_delta_=0,rotation_delta_=0;
  NavigationProgressPose anchor_,last_;
  std::string detail_;
};
}  // namespace agx_workbench_navigation
