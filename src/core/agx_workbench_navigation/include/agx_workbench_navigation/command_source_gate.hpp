#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <memory>
#include <set>
#include <string>
#include <utility>

namespace agx_workbench_navigation {
struct CommandGoalEndpoint {
  std::string frame;
  std::uint64_t stamp_ns=0;
  std::array<double,3> position{};
  std::array<double,4> orientation{};
};
struct CommandBinding {
  std::string session_id,grant_id,source_goal_id;
  CommandGoalEndpoint endpoint;
  std::string invocation_id,route_id;
  double task_deadline_s=0;
  CommandBinding()=default;
  CommandBinding(std::string session,std::string grant,std::string source,CommandGoalEndpoint goal,
      std::string invocation={},std::string route={},double deadline=0)
      :session_id(std::move(session)),grant_id(std::move(grant)),source_goal_id(std::move(source)),
       endpoint(std::move(goal)),invocation_id(std::move(invocation)),route_id(std::move(route)),
       task_deadline_s(deadline) {}
};
enum class PlanDisposition {Installed, SupersededDelivery, Rejected};

// The goal's original nonzero map-frame ROS timestamp is its plan nonce. The
// global planner must retain it on the last point. Never mint identity from a
// current authority topic or a plan's computation/receipt time.
// Callbacks execute under the same lock and must not re-enter this gate.
class CommandSourceGate {
 public:
  template<class Reset>
  bool Bind(const CommandBinding& binding,Reset reset) {
    return BindValidated(binding,[]{return true;},std::move(reset));
  }
  template<class Validate,class Reset>
  bool BindValidated(const CommandBinding& binding,Validate validate,Reset reset) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (binding.session_id.empty() || binding.grant_id.empty() || binding.source_goal_id.empty() ||
        !Valid(binding.endpoint) || binding.route_id.empty()!=binding.invocation_id.empty() ||
        (!binding.route_id.empty() && (!std::isfinite(binding.task_deadline_s) || binding.task_deadline_s<=0)) ||
        (binding.route_id.empty() && binding.task_deadline_s!=0)) return false;
    if (binding_ && SameBinding(binding,*binding_)) return validate();
    const auto grant=std::make_pair(binding.session_id,binding.grant_id);
    const auto operation=std::make_pair(binding.session_id,binding.source_goal_id);
    if (used_grants_.count(grant) || used_operations_.count(operation)) return false;
    if (binding.endpoint.stamp_ns<=last_goal_stamp_) return false;
    if (!validate()) return false;
    // A rebind waits for in-flight computation/publication, then destroys the
    // previous plan before granting the new operation permission to compute.
    last_goal_stamp_=binding.endpoint.stamp_ns;
    used_grants_.insert(grant);used_operations_.insert(operation);
    binding_.reset();plan_ready_=false;sequence_=0;
    reset();
    binding_=std::make_unique<CommandBinding>(binding);
    return true;
  }
  template<class Install>
  PlanDisposition DeliverPlan(const CommandGoalEndpoint& endpoint,Install install) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!binding_ || !Valid(endpoint)) return PlanDisposition::Rejected;
    // A stock move_base pending buffer may outlive its operation. Discarding
    // valid older delivery grants no readiness, installation or command identity.
    if (endpoint.stamp_ns<binding_->endpoint.stamp_ns) return PlanDisposition::SupersededDelivery;
    if (!Same(endpoint,binding_->endpoint)) return PlanDisposition::Rejected;
    install();plan_ready_=true;return PlanDisposition::Installed;
  }
  // This bool retains actual-install semantics for existing callers.
  template<class Install>
  bool AcceptPlan(const CommandGoalEndpoint& endpoint,Install install) {
    return DeliverPlan(endpoint,std::move(install))==PlanDisposition::Installed;
  }
  template<class ComputeCommand>
  bool Compute(ComputeCommand compute) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!binding_ || sequence_==std::numeric_limits<std::uint64_t>::max()) return false;
    return compute(*binding_,++sequence_,plan_ready_);
  }
  template<class Reset>
  void InvalidatePlan(Reset reset) {
    std::lock_guard<std::mutex> lock(mutex_);
    plan_ready_=false;reset();
  }
  template<class Read>
  bool Inspect(Read read) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return binding_ && plan_ready_ && read();
  }
  static bool Valid(const CommandGoalEndpoint& goal) {
    if (goal.frame!="map" || goal.stamp_ns==0) return false;
    for (double value:goal.position) if (!std::isfinite(value)) return false;
    double norm=0;
    for (double value:goal.orientation) {if(!std::isfinite(value))return false;norm+=value*value;}
    return std::isfinite(norm) && std::abs(norm-1.0)<=1e-6;
  }
  static bool Same(const CommandGoalEndpoint& a,const CommandGoalEndpoint& b) {
    if(a.frame!=b.frame || a.stamp_ns!=b.stamp_ns)return false;
    for(std::size_t i=0;i<3;++i)if(std::abs(a.position[i]-b.position[i])>1e-6)return false;
    double dot=0;
    for(std::size_t i=0;i<4;++i)dot+=a.orientation[i]*b.orientation[i];
    return std::isfinite(dot) && std::abs(std::abs(dot)-1.0)<=1e-6;
  }
  static bool SameBinding(const CommandBinding& a,const CommandBinding& b) {
    return a.session_id==b.session_id && a.grant_id==b.grant_id &&
        a.source_goal_id==b.source_goal_id && Same(a.endpoint,b.endpoint) &&
        a.invocation_id==b.invocation_id && a.route_id==b.route_id &&
        a.task_deadline_s==b.task_deadline_s;
  }
 private:
  mutable std::mutex mutex_;
  std::unique_ptr<CommandBinding> binding_;
  std::set<std::pair<std::string,std::string>> used_grants_,used_operations_;
  std::uint64_t sequence_=0,last_goal_stamp_=0;
  bool plan_ready_=false;
};
}
