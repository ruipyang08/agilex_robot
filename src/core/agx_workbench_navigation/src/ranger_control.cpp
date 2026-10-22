#include <agx_workbench_navigation/ranger_control.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
namespace agx_workbench_navigation {
namespace {
RangerControlDecision blockedDecision(const std::string& reason) {
  return {PlannerState::BLOCKED,0,0,0,0,reason};
}
RangerControlDecision stoppedDecision(PlannerState state,const std::string& reason) {
  return {state,0,0,0,0,reason};
}
double clampValue(double value,double lower,double upper) {
  return std::max(lower,std::min(upper,value));
}
}
RangerCommandLimiter::RangerCommandLimiter(const RangerPlannerConfig& config):config_(config) {
  for(double value:{config_.max_linear_accel_mps2,config_.max_linear_decel_mps2,
      config_.max_angular_accel_radps2,config_.max_lateral_accel_mps2,config_.rate_limit_max_dt_s,
      config_.min_turn_radius_m,config_.max_linear_speed_mps,config_.max_spin_speed_radps,
      config_.max_curvature_rate_per_m_s})
    if(!std::isfinite(value) || value<=0)throw std::invalid_argument("invalid Ranger command slew limits");
}
void RangerCommandLimiter::reset() {
  initialized_=false;previous_=stoppedDecision(PlannerState::GOAL_REACHED,"reset");previous_time_s_=0;
}
bool RangerCommandLimiter::commandStopped() const {
  return initialized_ && previous_.linear_x_mps==0 && previous_.angular_z_radps==0;
}
RangerControlDecision RangerCommandLimiter::limit(const RangerControlDecision& desired,double now_s,
                                                 const CanonicalMotionProgress& motion) {
  const bool known=desired.state==PlannerState::FOLLOW_ARC || desired.state==PlannerState::SPIN_ALIGN ||
      desired.state==PlannerState::GOAL_ALIGN || desired.state==PlannerState::GOAL_REACHED;
  if(desired.state==PlannerState::BLOCKED || !known || !motion.valid ||
      !std::isfinite(motion.forward_mps) || !std::isfinite(motion.lateral_mps) ||
      !std::isfinite(motion.yaw_radps) || !std::isfinite(motion.linear_lead_mps) ||
      !std::isfinite(motion.angular_lead_radps) || motion.linear_lead_mps<=0 ||
      motion.angular_lead_radps<=0 || !std::isfinite(now_s) || now_s<0 ||
      !std::isfinite(desired.linear_x_mps) || !std::isfinite(desired.angular_z_radps) ||
      desired.linear_y_mps!=0 || desired.linear_x_mps<0 || desired.linear_x_mps>config_.max_linear_speed_mps ||
      (desired.linear_x_mps>0 && std::abs(desired.angular_z_radps)>desired.linear_x_mps/config_.min_turn_radius_m+1e-12) ||
      (desired.linear_x_mps==0 && std::abs(desired.angular_z_radps)>config_.max_spin_speed_radps) ||
      std::abs(desired.linear_x_mps*desired.angular_z_radps)>config_.max_lateral_accel_mps2+1e-12 ||
      (initialized_ && now_s<previous_time_s_)) {
    reset();return blockedDecision(desired.state==PlannerState::BLOCKED?desired.reason:"invalid command slew input");
  }
  if(!initialized_) {
    initialized_=true;previous_time_s_=now_s;
    previous_=stoppedDecision(desired.state,"acceleration startup");return previous_;
  }
  const double elapsed=clampValue(now_s-previous_time_s_,0,config_.rate_limit_max_dt_s);
  previous_time_s_=now_s;
  auto target=desired;
  const bool previous_spin=previous_.linear_x_mps==0 && previous_.angular_z_radps!=0;
  const bool target_spin=target.linear_x_mps==0 && target.angular_z_radps!=0;
  if((previous_spin && target.linear_x_mps>0) || (previous_.linear_x_mps>0 && target_spin))
    target=stoppedDecision(previous_.state,"settling before motion mode change");
  const double p=previous_.linear_x_mps,q=previous_.angular_z_radps;
  const double acceleration=config_.max_linear_accel_mps2*elapsed;
  const double deceleration=config_.max_linear_decel_mps2*elapsed;
  const double angular_change=config_.max_angular_accel_radps2*elapsed;
  const double curvature_change=config_.max_curvature_rate_per_m_s*elapsed;
  const double linear_ceiling=std::max(0.0,motion.forward_mps)+motion.linear_lead_mps;
  const double yaw_floor=std::min(0.0,motion.yaw_radps)-motion.angular_lead_radps;
  const double yaw_ceiling=std::max(0.0,motion.yaw_radps)+motion.angular_lead_radps;
  const bool arc_mode=p>0 || (!previous_spin && target.linear_x_mps>0);
  const bool braking=target.linear_x_mps==0 && target.angular_z_radps==0;
  const double previous_curvature=p>0?q/p:0;
  for(double value:{acceleration,deceleration,angular_change,curvature_change,
      linear_ceiling,yaw_floor,yaw_ceiling,previous_curvature}) {
    if(!std::isfinite(value)) {reset();return blockedDecision("invalid command feasibility bounds");}
  }
  const auto valid=[&](double v,double w) {
    if(!std::isfinite(v) || !std::isfinite(w) || v<0 || v>config_.max_linear_speed_mps ||
        v-p>acceleration || p-v>deceleration || std::abs(w-q)>angular_change ||
        v>linear_ceiling || w<yaw_floor || w>yaw_ceiling ||
        std::abs(v*w)>config_.max_lateral_accel_mps2 ||
        (braking && (v>p || w<std::min(0.0,q) || w>std::max(0.0,q)))) return false;
    if(!arc_mode) return v==0 && std::abs(w)<=config_.max_spin_speed_radps &&
        (w==0 || std::abs(w)>std::numeric_limits<double>::epsilon());
    if(v==0) return w==0;
    // Admission already binds this configured radius to the active vendor
    // minimum. Check the actual dispatch quotient as well as the cone:
    // rounded w=v/R alone can produce v/w<R and select vendor spin.
    return std::abs(w)<=v/config_.min_turn_radius_m &&
        (std::abs(w)<=std::numeric_limits<double>::epsilon() ||
         std::abs(v/w)>=config_.min_turn_radius_m) &&
        (p==0 || std::abs(w/v-previous_curvature)<=curvature_change);
  };
  const auto interpolate=[&](const RangerControlDecision& next,RangerControlDecision* output) {
    const double linear_delta=next.linear_x_mps-previous_.linear_x_mps;
    const double angular_delta=next.angular_z_radps-previous_.angular_z_radps;
    double lower=0,upper=1;
    if(linear_delta!=0) upper=std::min(upper,elapsed*(linear_delta>0?
        config_.max_linear_accel_mps2:config_.max_linear_decel_mps2)/std::abs(linear_delta));
    if(angular_delta!=0) upper=std::min(upper,elapsed*config_.max_angular_accel_radps2/std::abs(angular_delta));
    const auto intersect=[&](double previous,double delta,double minimum,double maximum) {
      if(delta==0) return previous>=minimum && previous<=maximum;
      double a=(minimum-previous)/delta,b=(maximum-previous)/delta;
      if(a>b) std::swap(a,b);
      lower=std::max(lower,a);upper=std::min(upper,b);
      return lower<=upper;
    };
    if(!intersect(previous_.linear_x_mps,linear_delta,0,
                  std::max(0.0,motion.forward_mps)+motion.linear_lead_mps) ||
       !intersect(previous_.angular_z_radps,angular_delta,
                  std::min(0.0,motion.yaw_radps)-motion.angular_lead_radps,
                  std::max(0.0,motion.yaw_radps)+motion.angular_lead_radps)) return false;
    *output=next;
    const double linear_ceiling=std::max(0.0,motion.forward_mps)+motion.linear_lead_mps;
    const double yaw_floor=std::min(0.0,motion.yaw_radps)-motion.angular_lead_radps;
    const double yaw_ceiling=std::max(0.0,motion.yaw_radps)+motion.angular_lead_radps;
    bool bounded=false;
    for(int attempt=0;attempt<4;++attempt) {
      output->linear_x_mps=upper>=1?next.linear_x_mps:previous_.linear_x_mps+upper*linear_delta;
      output->angular_z_radps=upper>=1?next.angular_z_radps:previous_.angular_z_radps+upper*angular_delta;
      bounded=output->linear_x_mps>=0 && output->linear_x_mps<=linear_ceiling &&
          output->angular_z_radps>=yaw_floor && output->angular_z_radps<=yaw_ceiling;
      if(bounded || upper<=lower) break;
      // Round toward the feasible interior, never enlarge the measured lead.
      // Otherwise one ulp outside a ceiling can poison the next steady tick.
      upper=std::nextafter(upper,lower);
    }
    if(!bounded) return false;
    if(output->state==PlannerState::GOAL_REACHED &&
        (output->linear_x_mps!=0 || output->angular_z_radps!=0)) output->state=previous_.state;
    output->command_radius_m=output->linear_x_mps>0 && output->angular_z_radps!=0?
        std::abs(output->linear_x_mps/output->angular_z_radps):0;
    return true;
  };
  RangerControlDecision output;
  bool feasible=interpolate(target,&output);
  feasible=feasible && valid(output.linear_x_mps,output.angular_z_radps);
  if(!feasible) {
    // A failed interpolation line does not prove the 2-D command set empty.
    // Project the fixed-mode yaw interval onto forward speed. Wider arithmetic
    // reduces endpoint cancellation; only the strict emitted-double check above
    // can accept a candidate. No measured lead or rate bound is enlarged.
    using Wide=long double;
    Wide vlo=std::max(Wide(0),Wide(p)-deceleration);
    Wide vhi=std::min({Wide(config_.max_linear_speed_mps),Wide(p)+acceleration,Wide(linear_ceiling)});
    Wide low=std::max(Wide(q)-angular_change,Wide(yaw_floor));
    Wide high=std::min(Wide(q)+angular_change,Wide(yaw_ceiling));
    if(braking) {
      vhi=std::min(vhi,Wide(p));
      low=std::max(low,Wide(std::min(0.0,q)));
      high=std::min(high,Wide(std::max(0.0,q)));
    }
    Wide a=-Wide(1)/config_.min_turn_radius_m,b=-a;
    if(arc_mode && p>0) {
      a=std::max(a,Wide(previous_curvature)-curvature_change);
      b=std::min(b,Wide(previous_curvature)+curvature_change);
    }
    const Wide lateral=config_.max_lateral_accel_mps2;
    bool nonempty=vlo<=vhi && low<=high && a<=b;
    if(arc_mode) {
      // max(low,a*v,-A/v) <= min(high,b*v,A/v). Its six
      // nontrivial pairwise inequalities each restrict one v interval.
      const auto linear_bound=[&](Wide coefficient,Wide right) {
        if(coefficient>0) vhi=std::min(vhi,right/coefficient);
        else if(coefficient<0) vlo=std::max(vlo,right/coefficient);
        else if(right<0) nonempty=false;
      };
      linear_bound(-b,-low);
      linear_bound(a,high);
      if(low>0)vhi=std::min(vhi,lateral/low);
      if(high<0)vhi=std::min(vhi,-lateral/high);
      if(a>0)vhi=std::min(vhi,std::sqrt(lateral/a));
      if(b<0)vhi=std::min(vhi,std::sqrt(-lateral/b));
    } else {
      nonempty=nonempty && vlo<=0 && vhi>=0;
      vlo=vhi=0;
      low=std::max(low,-Wide(config_.max_spin_speed_radps));
      high=std::min(high,Wide(config_.max_spin_speed_radps));
    }
    nonempty=nonempty && vlo<=vhi && low<=high;
    const auto accept=[&](double v,double w) {
      if(!valid(v,w))return false;
      output=target;output.linear_x_mps=v;output.angular_z_radps=w;return true;
    };
    const auto try_yaw=[&](double v) {
      Wide lo=low,hi=high;
      if(arc_mode) {
        if(v==0) {lo=std::max(lo,Wide(0));hi=std::min(hi,Wide(0));}
        else {
          lo=std::max({lo,a*v,-lateral/v});
          hi=std::min({hi,b*v,lateral/v});
        }
      }
      if(lo>hi)return false;
      double w=static_cast<double>(std::max(lo,std::min(hi,Wide(target.angular_z_radps))));
      if(!arc_mode && w!=0 && std::abs(w)<=std::numeric_limits<double>::epsilon()) {
        // The vendor treats this tail as straight. Reach exact zero when its
        // slew permits; otherwise retain the nearest actual spin command.
        const double spin_edge=std::nextafter(std::numeric_limits<double>::epsilon(),
                                             std::numeric_limits<double>::infinity());
        if(accept(v,0) || accept(v,std::copysign(spin_edge,w)))return true;
      }
      const double middle=static_cast<double>(lo+(hi-lo)/2);
      for(int attempt=0;attempt<6;++attempt) {
        if(accept(v,w))return true;
        w=attempt<4?std::nextafter(w,middle):middle;
      }
      return false;
    };
    if(nonempty) {
      double v=static_cast<double>(std::max(vlo,std::min(vhi,Wide(target.linear_x_mps))));
      const double middle=static_cast<double>(vlo+(vhi-vlo)/2);
      // A yaw bound can be pinned at a progress/curvature intersection. In
      // that case repair v inward and recompute yaw, rather than moving yaw
      // outside its measured bound. Midpoints are bounded numerical recovery.
      for(int attempt=0;attempt<6 && !feasible;++attempt) {
        feasible=try_yaw(v);
        v=attempt<4?std::nextafter(v,middle):middle;
      }
    }
    if(!feasible) {
      reset();return blockedDecision(nonempty?"command feasibility numerical boundary":
          "measured progress incompatible with source slew");
    }
  }
  if(output.state==PlannerState::GOAL_REACHED &&
      (output.linear_x_mps!=0 || output.angular_z_radps!=0)) output.state=previous_.state;
  output.command_radius_m=output.linear_x_mps>0 && output.angular_z_radps!=0?
      std::abs(output.linear_x_mps/output.angular_z_radps):0;
  previous_=output;return output;
}


}  // namespace agx_workbench_navigation
