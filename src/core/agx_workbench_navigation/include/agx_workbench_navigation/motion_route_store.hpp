#pragma once
#include <agx_workbench_navigation/motion_route.hpp>
#include <agx_workbench_navigation/command_source_gate.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <functional>

namespace agx_workbench_navigation {
// This store carries no command authority. Every execution boundary must also
// validate its live session/grant. Clock reads occur under the store mutex.
class MotionRouteStore {
 public:
  explicit MotionRouteStore(std::function<double()> clock={});
  bool put(MotionRoute route,const MotionLimits& limits=MotionLimits{});
  std::shared_ptr<const MotionRoute> get(const std::string& route_id,
      const std::string& invocation_id);
  bool pin(const std::string& route_id, const std::string& invocation_id,
           double task_deadline_s);
  void release(const std::string& route_id, const std::string& invocation_id);
  bool bind(const CommandBinding& binding);
  // Called only after the phase owner checks fresh stopped endpoint evidence.
  // An external management request alone is not completion evidence.
  bool completeDeparture(const std::string& route_id,const std::string& invocation_id);
  std::shared_ptr<const MotionRoute> bound(const CommandBinding& binding);
  std::shared_ptr<const MotionRoute> boundGoal(const CommandGoalEndpoint& goal,
      std::size_t* segment=nullptr,std::size_t* sample=nullptr);
  void block(const std::string& route_id,const std::string& invocation_id,const std::string& reason);
  std::string blocked(const CommandBinding&);
  bool progress(const CommandBinding&,std::size_t segment,std::size_t sample);
 private:
  struct Record {
    std::shared_ptr<const MotionRoute> route;
    double expires_s;bool pinned=false;
    std::shared_ptr<const CommandBinding> binding;
    std::size_t segment=0,sample=0;
    std::string blocked_reason;
    bool departure_completed=false;
  };
  bool advance(double now_s);
  std::mutex mutex_;
  std::map<std::string,Record> records_;
  double last_now_s=-1;
  std::function<double()> clock_;
};
MotionRouteStore& SharedMotionRouteStore();
}  // namespace agx_workbench_navigation
