#include <agx_workbench_navigation/motion_search.hpp>
#include <agx_workbench_navigation/motion_speed_profile.hpp>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <queue>

namespace agx_workbench_navigation {
namespace {
using XY=std::array<double,2>;
using Controls=std::vector<XY>;
XY Add(XY a,XY b) {return {{a[0]+b[0],a[1]+b[1]}};}
XY Sub(XY a,XY b) {return {{a[0]-b[0],a[1]-b[1]}};}
XY Scale(XY a,double s) {return {{a[0]*s,a[1]*s}};}
double Norm(XY a) {return std::hypot(a[0],a[1]);}
double Cross(XY a,XY b) {return a[0]*b[1]-a[1]*b[0];}
double Dot(XY a,XY b) {return a[0]*b[0]+a[1]*b[1];}
Controls Derivative(const Controls& p) {
  Controls d;for(std::size_t i=1;i<p.size();++i)d.push_back(Scale(Sub(p[i],p[i-1]),p.size()-1));return d;
}
std::pair<Controls,Controls> Split(Controls p) {
  Controls left,right;left.push_back(p.front());right.push_back(p.back());
  while(p.size()>1) {
    Controls next;for(std::size_t i=1;i<p.size();++i)next.push_back(Scale(Add(p[i-1],p[i]),.5));
    p=std::move(next);left.push_back(p.front());right.push_back(p.back());
  }
  std::reverse(right.begin(),right.end());return {left,right};
}
MotionSample Endpoint(const Controls& p,bool end,double cap) {
  const auto d=Derivative(p),dd=Derivative(d);
  const auto v=end?d.back():d.front(),a=end?dd.back():dd.front(),point=end?p.back():p.front();
  return {{point[0],point[1],std::atan2(v[1],v[0])},Cross(v,a)/std::pow(Norm(v),3),cap,0,0};
}
// Bezier derivative control hulls bound tangent, curvature and curvature rate
// throughout each leaf, not merely at its sampled endpoints.
bool SampleConnector(const Controls& p,const MotionLimits& l,double step,double radius,
                     MotionSegment& out,unsigned depth=0) {
  if(depth>20 || out.samples.size()>19000)return false;
  const auto d=Derivative(p),dd=Derivative(d),ddd=Derivative(dd);
  const auto chord=Sub(p.back(),p.front());const double length=Norm(chord);
  if(length<1e-12)return false;
  const auto axis=Scale(chord,1/length);
  double lower=INFINITY,upper=0,max_angle=0;
  for(auto v:d) {
    lower=std::min(lower,Dot(v,axis));upper=std::max(upper,Norm(v));
    max_angle=std::max(max_angle,std::abs(std::atan2(Cross(axis,v),Dot(axis,v))));
  }
  if(lower<=1e-12 || upper>step || max_angle>.004) {
    const auto halves=Split(p);
    return SampleConnector(halves.first,l,step,radius,out,depth+1) &&
           SampleConnector(halves.second,l,step,radius,out,depth+1);
  }
  double numerator=0,numerator_prime=0,dot_bound=0;
  for(auto v:d)for(auto a:dd) {
    numerator=std::max(numerator,std::abs(Cross(v,a)));
    dot_bound=std::max(dot_bound,std::abs(Dot(v,a)));
  }
  for(auto v:d)for(auto j:ddd)numerator_prime=std::max(numerator_prime,std::abs(Cross(v,j)));
  const double curvature=numerator/std::pow(lower,3);
  const double rate=numerator_prime/std::pow(lower,4)+3*numerator*dot_bound/std::pow(lower,6);
  if(!std::isfinite(curvature) || !std::isfinite(rate) || curvature>1/l.min_turn_radius_m)return false;
  out.curvature_bound_per_m=std::max(out.curvature_bound_per_m,curvature);
  double cap=l.max_speed_mps;
  if(curvature>1e-10)cap=std::min(cap,std::min(std::sqrt(l.lateral_acceleration_mps2/curvature),l.max_yaw_rate_radps/curvature));
  if(rate>1e-10)cap=std::min(cap,std::min(l.curvature_rate_per_m_s/rate,std::sqrt(l.angular_acceleration_radps2/(2*rate))));
  if(cap<1e-6)return false; // Numerical floor, not a claimed measured actuator deadband.
  double deviation=0;
  for(auto v:p)deviation=std::max(deviation,std::abs(Cross(axis,Sub(v,p.front()))));
  out.geometry_error_m=std::max(out.geometry_error_m,deviation+2*radius*max_angle);
  auto a=Endpoint(p,false,cap),b=Endpoint(p,true,cap);
  if(out.samples.empty())out.samples.push_back(a);
  else out.samples.back().speed_ceiling_mps=std::min(out.samples.back().speed_ceiling_mps,cap);
  out.samples.push_back(b);return true;
}
MotionSegment Connector(const MotionPose& start,double curvature,double speed,const MotionPose& goal,
                        double scale,const MotionLimits& limits,double step,double radius) {
  MotionSegment out;
  const double length=std::hypot(goal.x_m-start.x_m,goal.y_m-start.y_m)*scale;
  if(length<.01)return out;
  XY t0{{std::cos(start.yaw_rad),std::sin(start.yaw_rad)}},n0{{-t0[1],t0[0]}};
  XY t1{{std::cos(goal.yaw_rad),std::sin(goal.yaw_rad)}};
  Controls p(6);p[0]={{start.x_m,start.y_m}};p[5]={{goal.x_m,goal.y_m}};
  p[1]=Add(p[0],Scale(t0,length/5));
  p[2]=Add(Sub(Scale(p[1],2),p[0]),Scale(n0,length*length*curvature/20));
  p[4]=Sub(p[5],Scale(t1,length/5));p[3]=Sub(Scale(p[4],2),p[5]);
  if(!SampleConnector(p,limits,step,radius,out) || out.geometry_error_m>.05)return {};
  // Exact endpoints are the authored Bezier endpoints, not snapped search bins.
  out.samples.front().pose=start;out.samples.front().curvature_per_m=curvature;
  out.samples.back().pose=goal;out.samples.back().curvature_per_m=0;
  if(!ProfileForwardMotion(out,limits,speed,0))return {};
  return out;
}
bool ClearSegment(const MotionSweepChecker& checker,MotionGeometryPolicy policy,const MotionSegment& seg) {
  if(seg.samples.empty())return false;
  policy.integration_error_m+=seg.geometry_error_m;
  std::vector<MotionPose> poses;poses.reserve(seg.samples.size());
  for(const auto& sample:seg.samples)poses.push_back(sample.pose);
  return checker.Check(policy,poses).clear();
}
double ClearanceCost(const MotionSweepChecker& checker,MotionGeometryPolicy policy,const MotionSegment& seg,double weight) {
  if(weight==0 || policy.soft_clearance_m==0)return 0;
  policy.hard_padding_m+=policy.soft_clearance_m;
  if(ClearSegment(checker,policy,seg))return 0;
  double length=0;
  for(std::size_t i=1;i<seg.samples.size();++i) {
    const auto& a=seg.samples[i-1].pose;const auto& b=seg.samples[i].pose;
    length+=std::hypot(b.x_m-a.x_m,b.y_m-a.y_m);
  }
  return weight*length; // Soft route cost only; never a feasibility veto.
}
struct Edge {bool spin=false;double curvature=0,hold=0,end_speed=0,ceiling=0,angle=0;};
struct Node {
  MotionPose pose;double curvature=0,speed=0,cost=0;
  std::size_t parent=0;Edge edge;bool previous_spin=false;
};
MotionSegment Rebuild(const Node& parent,const Edge& edge,const MotionLimits& limits,double step) {
  if(edge.spin)return MakeSpinPrimitive(parent.pose,edge.angle,limits);
  if(parent.speed==0)return MakePreparedForwardPrimitive(parent.pose,parent.curvature,
      edge.curvature,edge.hold,edge.end_speed,edge.ceiling,limits,step);
  return MakeForwardPrimitive(parent.pose,parent.curvature,edge.curvature,edge.hold,
                              parent.speed,edge.end_speed,edge.ceiling,limits,step);
}
using Key=std::array<long long,6>;
Key KeyFor(const Node& n,const MotionSearchOptions& o,const MotionLimits& l) {
  return {{std::llround(n.pose.x_m/o.position_bin_m),std::llround(n.pose.y_m/o.position_bin_m),
      std::llround(n.pose.yaw_rad/o.heading_bin_rad),std::llround(n.curvature*l.min_turn_radius_m*2),
      std::llround(n.speed*100),n.previous_spin?1:0}};
}
}
MotionSearchResult SearchMotionRoute(const MotionGrid& grid,const MotionGeometryPolicy& geometry,
    const MotionLimits& limits,const MotionRoute& header,const MotionSearchOptions& options) {
  MotionSearchResult out;const auto began=std::chrono::steady_clock::now();
  auto elapsed=[&]() {return std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();};
  auto finish=[&](RouteStatus status,const std::string& detail) {
    out.result.status=status;out.result.detail=detail;out.elapsed_s=elapsed();return out;
  };
  if(!ValidateMotionLimits(limits).empty() || !ValidMotionPose(header.measured_start) || !ValidMotionPose(header.goal) ||
      !SameMotionPose(header.measured_start,header.coarse_start) || !header.segments.empty() ||
      options.max_expansions==0 || options.max_nodes<2 || options.max_nodes>200000 ||
      options.max_labels_per_bin==0 || options.max_labels_per_bin>64 ||
      !std::isfinite(options.timeout_s) || options.timeout_s<=0 || options.timeout_s>60 ||
      !std::isfinite(options.position_bin_m) || options.position_bin_m<.001 ||
      !std::isfinite(options.heading_bin_rad) || options.heading_bin_rad<.001 ||
      !std::isfinite(options.heuristic_weight) || options.heuristic_weight<1 || options.heuristic_weight>10 ||
      !std::isfinite(options.soft_penalty_s_per_m) || options.soft_penalty_s_per_m<0 ||
      !std::isfinite(geometry.soft_clearance_m) || geometry.soft_clearance_m<0)
    return finish(RouteStatus::InvalidRequest,"invalid search request or bounds");
  if(!ValidMotionGrid(grid))return finish(RouteStatus::EvidenceUnavailable,"invalid map snapshot");
  if(grid.cells.size()>1000000)return finish(RouteStatus::EvidenceUnavailable,"map snapshot exceeds bounded search size");
  const MotionSweepChecker checker(grid);
  auto interrupted=[&]() {return options.cancelled && options.cancelled();};
  if(interrupted())return finish(RouteStatus::RouteInvalidated,"planning cancelled");
  const auto start_check=CheckMotionSweep(grid,geometry,{header.measured_start});
  const auto goal_check=CheckMotionSweep(grid,geometry,{header.goal});
  if(start_check.status==MotionCollision::Invalid || goal_check.status==MotionCollision::Invalid)
    return finish(RouteStatus::InvalidRequest,"invalid geometry policy");
  if(!start_check.clear() || !goal_check.clear()) {
    out.collision_reported=true;out.collision=!start_check.clear()?start_check:goal_check;
    return finish(RouteStatus::NoRoute,!start_check.clear()?"start footprint is blocked":"goal footprint is blocked");
  }
  double radius=0;for(const auto& p:geometry.footprint)radius=std::max(radius,Norm(p));
  const double step=std::min(.01,geometry.translation_step_m);
  if(radius>1.)return finish(RouteStatus::InvalidRequest,"footprint exceeds primitive integration radius");
  // A permissive 2D distance field guides the motion-state search around walls.
  // It never admits a route: every edge still uses the full polygon sweep and
  // kinematic profile below. Diagonal centre-cell connectivity stays permissive.
  using QueueEntry=std::pair<double,std::size_t>;
  std::vector<double> guide(grid.cells.size(),INFINITY);
  std::priority_queue<QueueEntry,std::vector<QueueEntry>,std::greater<QueueEntry>> flood;
  const auto goal_x=static_cast<std::size_t>((header.goal.x_m-grid.origin_x_m)/grid.resolution_m);
  const auto goal_y=static_cast<std::size_t>((header.goal.y_m-grid.origin_y_m)/grid.resolution_m);
  const auto goal_cell=goal_y*grid.width+goal_x;guide[goal_cell]=0;flood.push({0,goal_cell});
  std::size_t visited=0;
  while(!flood.empty()) {
    if((++visited%1024)==0) {
      if(interrupted())return finish(RouteStatus::RouteInvalidated,"planning cancelled");
      if(elapsed()>=options.timeout_s) {
        out.budget_exhausted=true;out.budget_reason="time_limit_guidance";
        return finish(RouteStatus::SearchLimit,"map guidance budget exhausted; no complete route");
      }
    }
    const auto current=flood.top();flood.pop();
    if(current.first!=guide[current.second])continue;
    const auto x=static_cast<long long>(current.second%grid.width),y=static_cast<long long>(current.second/grid.width);
    for(int dy=-1;dy<=1;++dy)for(int dx=-1;dx<=1;++dx) {
      if((dx==0 && dy==0) || x+dx<0 || y+dy<0 ||
         x+dx>=static_cast<long long>(grid.width) || y+dy>=static_cast<long long>(grid.height))continue;
      const auto next=static_cast<std::size_t>(y+dy)*grid.width+static_cast<std::size_t>(x+dx);
      if(grid.cells[next]>=254)continue;
      const double cost=current.first+grid.resolution_m*(dx!=0 && dy!=0?std::sqrt(2.):1.);
      if(cost<guide[next]) {guide[next]=cost;flood.push({cost,next});}
    }
  }
  std::vector<Node> nodes(1);nodes[0].pose=header.measured_start;
  std::priority_queue<QueueEntry,std::vector<QueueEntry>,std::greater<QueueEntry>> queue;
  auto heuristic=[&](const MotionPose& pose) {
    double distance=std::hypot(header.goal.x_m-pose.x_m,header.goal.y_m-pose.y_m);
    const auto x=static_cast<long long>(std::floor((pose.x_m-grid.origin_x_m)/grid.resolution_m));
    const auto y=static_cast<long long>(std::floor((pose.y_m-grid.origin_y_m)/grid.resolution_m));
    if(x>=0 && y>=0 && x<static_cast<long long>(grid.width) && y<static_cast<long long>(grid.height)) {
      const double value=guide[static_cast<std::size_t>(y)*grid.width+static_cast<std::size_t>(x)];
      if(std::isfinite(value))distance=std::max(distance,.9*value-2*grid.resolution_m);
    }
    return distance/limits.max_speed_mps;
  };
  queue.push({options.heuristic_weight*heuristic(nodes[0].pose),0});
  std::map<Key,std::vector<std::size_t>> labels;labels[KeyFor(nodes[0],options,limits)]={0};
  double best=INFINITY;std::size_t best_node=0;std::vector<MotionSegment> best_tail;
  bool pruned=false;
  while(!queue.empty()) {
    if(interrupted())return finish(RouteStatus::RouteInvalidated,"planning cancelled");
    if(elapsed()>=options.timeout_s || out.expansions>=options.max_expansions || nodes.size()>=options.max_nodes) {
      out.budget_exhausted=true;
      out.budget_reason=elapsed()>=options.timeout_s?"time_limit":
          out.expansions>=options.max_expansions?"expansion_limit":"node_limit";
      break;
    }
    const auto entry=queue.top();queue.pop();
    const std::size_t index=entry.second;const Node node=nodes[index];
    // Weighted queue rank is not a lower bound. Only the unweighted bound
    // can discard a candidate after a complete incumbent has been found.
    if(node.cost+heuristic(node.pose)>=best)continue;
    ++out.expansions;
    auto offer=[&](std::vector<MotionSegment> tail) {
      ++out.connector_attempts;
      double cost=node.cost;
      for(const auto& s:tail) {
        if(!ClearSegment(checker,geometry,s)) {++out.collision_rejected;return;}
        cost+=s.estimated_time_s+ClearanceCost(checker,geometry,s,options.soft_penalty_s_per_m);
      }
      if(cost<best) {best=cost;best_node=index;best_tail=std::move(tail);}
    };
    const double distance=std::hypot(header.goal.x_m-node.pose.x_m,header.goal.y_m-node.pose.y_m);
    if(SameMotionPose(node.pose,header.goal) && node.speed==0) {
      MotionSegment stop;stop.kind=MotionKind::VerifyStop;stop.samples.push_back({node.pose,0,0,0,0});offer({stop});
    } else if(distance<1e-6 && node.speed==0) {
      const double angle=MotionAngleDifference(header.goal.yaw_rad,node.pose.yaw_rad);
      offer({MakeSpinPrimitive(node.pose,angle,limits)});
    } else if(distance<=4) {
      // Recover exact constant-curvature connections as well as smooth blends.
      // A stopped node can prepare its steering without needing a moving ramp
      // that would protrude into the inside corner before the arc even begins.
      const double angle=MotionAngleDifference(header.goal.yaw_rad,node.pose.yaw_rad);
      const double bearing=std::atan2(header.goal.y_m-node.pose.y_m,header.goal.x_m-node.pose.x_m);
      const double curvature=2*std::sin(angle/2)/distance;
      if(std::abs(angle)>1e-6 && std::abs(curvature)<=1/limits.min_turn_radius_m+1e-9 &&
         std::abs(MotionAngleDifference(bearing,node.pose.yaw_rad+angle/2))<1e-8) {
        const double k=std::max(-1/limits.min_turn_radius_m,std::min(1/limits.min_turn_radius_m,curvature));
        MotionSegment arc;
        if(node.speed==0)arc=MakePreparedForwardPrimitive(node.pose,node.curvature,k,
            angle/k,0,limits.max_speed_mps,limits,step);
        else if(std::abs(node.curvature-k)<1e-9)arc=MakeForwardPrimitive(node.pose,k,k,
            angle/k,node.speed,0,limits.max_speed_mps,limits,step);
        if(!arc.samples.empty() && SameMotionPose(arc.samples.back().pose,header.goal))offer({arc});
      }
      for(double scale:{.7,1.,1.5}) {
        if(elapsed()>=options.timeout_s || interrupted())break;
        auto s=Connector(node.pose,node.curvature,node.speed,header.goal,scale,limits,step,radius);
        if(!s.samples.empty())offer({s});
      }
      if(node.speed==0) {
        const double ax=std::cos(node.pose.yaw_rad),ay=std::sin(node.pose.yaw_rad);
        const double bx=std::cos(header.goal.yaw_rad),by=std::sin(header.goal.yaw_rad);
        const double determinant=ax*by-ay*bx;
        if(std::abs(determinant)>1e-6) {
          const double dx=header.goal.x_m-node.pose.x_m,dy=header.goal.y_m-node.pose.y_m;
          const double first=(dx*by-dy*bx)/determinant,second=(ax*dy-ay*dx)/determinant;
          if(first>.01 && second>.01 && first+second<=8) {
            // Tangent straight/arc/straight candidates share these exact
            // endpoints with the stopped-turn alternative. Preparation and
            // braking are explicit; no unchecked corner waypoint is inserted.
            for(double radius_scale:{1.,1.5,2.}) {
              const double turn_radius=radius_scale*limits.min_turn_radius_m;
              const double tangent=turn_radius*std::tan(std::abs(angle)/2);
              if(tangent<=0 || first<=tangent+.01 || second<=tangent+.01)continue;
              auto lead=MakePreparedForwardPrimitive(node.pose,node.curvature,0,first-tangent,
                  0,limits.max_speed_mps,limits,step);
              if(lead.samples.empty())continue;
              const double k=(angle>0?1.:-1.)/turn_radius;
              auto curve=MakePreparedForwardPrimitive(lead.samples.back().pose,0,k,
                  std::abs(angle)*turn_radius,0,limits.max_speed_mps,limits,step);
              if(curve.samples.empty())continue;
              auto exit=MakePreparedForwardPrimitive(curve.samples.back().pose,k,0,
                  second-tangent,0,limits.max_speed_mps,limits,step);
              if(!exit.samples.empty() && SameMotionPose(exit.samples.back().pose,header.goal))
                offer({lead,curve,exit});
            }
            auto approach=MakePreparedForwardPrimitive(node.pose,node.curvature,0,first,0,limits.max_speed_mps,limits,step);
            if(!approach.samples.empty()) {
              auto turn=MakeSpinPrimitive(approach.samples.back().pose,angle,limits);
              if(!turn.samples.empty()) {
                auto depart=MakePreparedForwardPrimitive(turn.samples.back().pose,0,0,second,0,limits.max_speed_mps,limits,step);
                if(!depart.samples.empty() && SameMotionPose(depart.samples.back().pose,header.goal))
                  offer({approach,turn,depart});
              }
            }
          }
        }
      }
      // A completely checked stop/turn/straight/turn alternative is useful in
      // tight spaces. It never appends an unchecked sharp corner to a moving path.
      if(node.speed==0) {
        std::vector<MotionSegment> tail;MotionPose pose=node.pose;
        const double bearing=std::atan2(header.goal.y_m-pose.y_m,header.goal.x_m-pose.x_m);
        const double first=MotionAngleDifference(bearing,pose.yaw_rad);
        if(std::abs(first)>1e-6) {auto s=MakeSpinPrimitive(pose,first,limits);pose=s.samples.back().pose;tail.push_back(s);}
        auto forward=MakePreparedForwardPrimitive(pose,tail.empty()?node.curvature:0,0,
            distance,0,limits.max_speed_mps,limits,step);
        if(!forward.samples.empty()) {
          pose=forward.samples.back().pose;tail.push_back(forward);
          const double last=MotionAngleDifference(header.goal.yaw_rad,pose.yaw_rad);
          if(std::abs(last)>1e-6)tail.push_back(MakeSpinPrimitive(pose,last,limits));
          offer(tail);
        }
      }
    }
    auto expand=[&](Edge edge) {
      if(elapsed()>=options.timeout_s || interrupted() || nodes.size()>=options.max_nodes)return;
      auto segment=Rebuild(node,edge,limits,step);++out.generated;
      if(edge.spin)++out.spin_generated;else ++out.forward_generated;
      if(segment.samples.empty())return;
      Node next;next.pose=segment.samples.back().pose;next.curvature=edge.spin?0:edge.curvature;
      next.speed=edge.spin?0:edge.end_speed;next.cost=node.cost+segment.estimated_time_s;
      next.parent=index;next.edge=edge;next.previous_spin=edge.spin;
      if(next.cost+heuristic(next.pose)>=best)return;
      const auto key=KeyFor(next,options,limits);
      const auto found=labels.find(key);
      if(found!=labels.end()) {
        for(auto old:found->second)if(nodes[old].cost<=next.cost && SameMotionPose(nodes[old].pose,next.pose,.005)) {
          ++out.duplicate_rejected;return;
        }
        if(found->second.size()>=options.max_labels_per_bin) {pruned=true;++out.label_pruned;return;}
      }
      if(!ClearSegment(checker,geometry,segment)) {++out.collision_rejected;return;}
      next.cost+=ClearanceCost(checker,geometry,segment,options.soft_penalty_s_per_m);
      const auto id=nodes.size();nodes.push_back(next);labels[key].push_back(id);
      queue.push({next.cost+options.heuristic_weight*heuristic(next.pose),id});
    };
    for(double factor:{0.,-.5,.5,-1.,1.})for(double hold:{.2,.4,.8})for(double speed:{0.,.1,.25,.5}) {
      if(speed>limits.max_speed_mps)continue;
      Edge edge;edge.curvature=factor/limits.min_turn_radius_m;edge.hold=hold;edge.end_speed=speed;
      edge.ceiling=std::min(limits.max_speed_mps,std::max(.1,std::max(node.speed,speed)));
      expand(edge);
    }
    if(node.speed==0 && !node.previous_spin)for(int degrees=5;degrees<=180;degrees+=5)for(double sign:{-1.,1.}) {
      Edge edge;edge.spin=true;edge.angle=sign*degrees*std::acos(-1.)/180;expand(edge);
    }
  }
  if(interrupted())return finish(RouteStatus::RouteInvalidated,"planning cancelled");
  if(!std::isfinite(best))return finish(out.budget_exhausted || pruned?RouteStatus::SearchLimit:RouteStatus::NoRoute,
      out.budget_exhausted || pruned?"search budget or label capacity exhausted; no complete route":"supported search found no complete route");
  MotionRoute route=header;std::vector<MotionSegment> reversed;
  for(std::size_t id=best_node;id!=0;id=nodes[id].parent)reversed.push_back(Rebuild(nodes[nodes[id].parent],nodes[id].edge,limits,step));
  std::reverse(reversed.begin(),reversed.end());route.segments=std::move(reversed);
  route.segments.insert(route.segments.end(),best_tail.begin(),best_tail.end());route.estimated_time_s=0;
  for(const auto& segment:route.segments) {
    if(!ClearSegment(checker,geometry,segment))return finish(RouteStatus::RouteInvalidated,"final swept route validation failed");
    route.estimated_time_s+=segment.estimated_time_s;
  }
  const auto invalid=ValidateMotionRoute(route,limits);
  if(!invalid.empty())return finish(RouteStatus::InvalidRequest,"generated route rejected: "+invalid);
  out.result.route=std::move(route);
  return finish(RouteStatus::Ready,out.budget_exhausted?"complete feasible route; optimization budget exhausted":"complete feasible motion route");
}
}  // namespace agx_workbench_navigation
