#include <agx_workbench_navigation/motion_route_store.hpp>
#include <chrono>
#include <cmath>

namespace agx_workbench_navigation {
MotionRouteStore& SharedMotionRouteStore() {static MotionRouteStore store;return store;}
MotionRouteStore::MotionRouteStore(std::function<double()> clock):clock_(std::move(clock)) {
  if(!clock_)clock_=[] {return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();};
}
bool MotionRouteStore::advance(double now) {
  if(!std::isfinite(now) || now<0 || now<last_now_s) {records_.clear();return false;}
  last_now_s=now;
  for(auto it=records_.begin();it!=records_.end();) {
    if(now>=it->second.expires_s)it=records_.erase(it);else ++it;
  }
  return true;
}
bool MotionRouteStore::put(MotionRoute route,const MotionLimits& limits) {
  if(!ValidateMotionRoute(route,limits).empty())return false;
  std::lock_guard<std::mutex> lock(mutex_);const double now=clock_();
  if(!advance(now) || records_.count(route.route_id))return false;
  std::size_t pending=0;for(const auto& pair:records_)if(!pair.second.pinned)++pending;
  if(pending>=8)return false;
  const std::string id=route.route_id;
  records_.emplace(id,Record{std::make_shared<const MotionRoute>(std::move(route)),now+60,false,{},0,0,{}});
  return true;
}
std::shared_ptr<const MotionRoute> MotionRouteStore::get(const std::string& id,const std::string& invocation) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!advance(clock_()))return {};
  const auto found=records_.find(id);
  if(found==records_.end() || found->second.route->invocation_id!=invocation || !found->second.blocked_reason.empty())return {};
  return found->second.route;
}
bool MotionRouteStore::pin(const std::string& id,const std::string& invocation,double deadline) {
  std::lock_guard<std::mutex> lock(mutex_);const double now=clock_();
  if(!advance(now) || !std::isfinite(deadline) || deadline<=now || deadline-now>86400)return false;
  const auto found=records_.find(id);
  if(found==records_.end() || found->second.route->invocation_id!=invocation || !found->second.blocked_reason.empty())return false;
  if(found->second.pinned)return deadline==found->second.expires_s; // Idempotent, never renews.
  for(const auto& pair:records_)if(pair.second.pinned)return false;
  found->second.pinned=true;found->second.expires_s=deadline;return true;
}
void MotionRouteStore::release(const std::string& id,const std::string& invocation) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found=records_.find(id);
  if(found!=records_.end() && found->second.route->invocation_id==invocation)records_.erase(found);
}
bool MotionRouteStore::bind(const CommandBinding& binding) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!advance(clock_()) || binding.session_id.empty() || binding.grant_id.empty() ||
     binding.source_goal_id.empty() || !CommandSourceGate::Valid(binding.endpoint))return false;
  const auto found=records_.find(binding.route_id);
  if(found==records_.end() || !found->second.blocked_reason.empty() || !found->second.pinned ||
     found->second.route->invocation_id!=binding.invocation_id ||
     binding.task_deadline_s!=found->second.expires_s)return false;
  if(found->second.route->segments.front().kind==MotionKind::DockDeparture &&
     !found->second.departure_completed)return false;
  const auto& endpoint=binding.endpoint;const auto& q=endpoint.orientation;
  if(std::abs(endpoint.position[2])>1e-6 || std::abs(q[0])>1e-6 || std::abs(q[1])>1e-6 ||
     !SameMotionPose(found->second.route->goal,{endpoint.position[0],endpoint.position[1],
       std::atan2(2*q[2]*q[3],1-2*q[2]*q[2])}))return false;
  if(found->second.binding)return CommandSourceGate::SameBinding(*found->second.binding,binding);
  found->second.binding=std::make_shared<const CommandBinding>(binding);
  return true;
}
bool MotionRouteStore::completeDeparture(const std::string& id,const std::string& invocation) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!advance(clock_()))return false;
  const auto found=records_.find(id);
  if(found==records_.end() || !found->second.pinned || found->second.binding ||
     !found->second.blocked_reason.empty() || found->second.route->invocation_id!=invocation ||
     found->second.route->segments.size()<2 ||
     found->second.route->segments.front().kind!=MotionKind::DockDeparture)return false;
  found->second.departure_completed=true;found->second.segment=1;found->second.sample=0;
  return true;
}
std::shared_ptr<const MotionRoute> MotionRouteStore::bound(const CommandBinding& binding) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!advance(clock_()))return {};
  const auto found=records_.find(binding.route_id);
  if(found==records_.end() || !found->second.blocked_reason.empty() || !found->second.binding ||
     !CommandSourceGate::SameBinding(*found->second.binding,binding))return {};
  return found->second.route;
}
std::shared_ptr<const MotionRoute> MotionRouteStore::boundGoal(const CommandGoalEndpoint& goal,
    std::size_t* segment,std::size_t* sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!advance(clock_()) || !CommandSourceGate::Valid(goal))return {};
  for(const auto& pair:records_)if(pair.second.blocked_reason.empty() && pair.second.binding &&
      CommandSourceGate::Same(pair.second.binding->endpoint,goal)) {
    if(segment)*segment=pair.second.segment;if(sample)*sample=pair.second.sample;
    return pair.second.route;
  }
  return {};
}
void MotionRouteStore::block(const std::string& id,const std::string& invocation,const std::string& reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!advance(clock_()) || reason.empty())return;
  const auto found=records_.find(id);
  if(found!=records_.end() && found->second.route->invocation_id==invocation && found->second.blocked_reason.empty())
    found->second.blocked_reason=reason.substr(0,256);
}
std::string MotionRouteStore::blocked(const CommandBinding& binding) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!advance(clock_()))return {};
  const auto found=records_.find(binding.route_id);
  if(found==records_.end() || !found->second.binding ||
      !CommandSourceGate::SameBinding(*found->second.binding,binding))return {};
  return found->second.blocked_reason;
}
bool MotionRouteStore::progress(const CommandBinding& binding,std::size_t segment,std::size_t sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  if(!advance(clock_()))return false;
  const auto found=records_.find(binding.route_id);
  if(found==records_.end() || !found->second.blocked_reason.empty() || !found->second.binding ||
     !CommandSourceGate::SameBinding(*found->second.binding,binding))return false;
  auto& record=found->second;
  if(segment<record.segment || (segment==record.segment && sample<record.sample) ||
     segment>record.route->segments.size() ||
     (segment==record.route->segments.size()?sample!=0:sample>=record.route->segments[segment].samples.size()))return false;
  record.segment=segment;record.sample=sample;return true;
}
}  // namespace agx_workbench_navigation
