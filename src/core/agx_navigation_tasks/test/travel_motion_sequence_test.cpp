#include <cassert>
#include <limits>
#include <mutex>
#include "travel_test_support.hpp"
#include "agx_navigation_tasks/travel_motion_sequence.hpp"
using namespace travel_test;
int main() {
  const auto profile=Profile();
  auto start=StartClearance{true,"home","home-to-default","measured"};
  const auto load=Sample().robot.transport;
  auto selected=ResolveTravelMotionStart(profile,start,load);
  assert(selected && selected->id=="home");
  start.path_id="other";assert(!ResolveTravelMotionStart(profile,start,load));
  start.path_id="home-to-default";start.ok=false;assert(!ResolveTravelMotionStart(profile,start,load));
  start.ok=true;
  assert(!ResolveTravelMotionStart(profile,start,Sample(100,200,true).robot.transport));
  const std::vector<std::string> names{"joint1","joint2"};
  assert(InTravelJointCorridor(profile,*selected,names,{.25,-.25}));
  assert(!InTravelJointCorridor(profile,*selected,names,{-.2,-.25}));
  assert(!InTravelJointCorridor(profile,*selected,{"joint1","joint1"},{.25,-.25}));
  assert(!InTravelJointCorridor(profile,*selected,names,{.25,std::numeric_limits<double>::quiet_NaN()}));
  auto limits=profile;limits.lift_motion_velocity_mps=.0016;
  auto wire=TravelLiftWireValues(limits);assert(wire && wire->speed_mm_s==1 && wire->position_mm==400);
  limits.lift_motion_velocity_mps=.0009;assert(!TravelLiftWireValues(limits));
  limits=profile;limits.lift_position_m=1e300;assert(!TravelLiftWireValues(limits));
  limits=profile;limits.lift_position_m=.4004;limits.lift_tolerance_m=.0001;
  assert(!TravelLiftWireValues(limits));
  auto geometry=profile;geometry.held_attachment_link="dh_robotics_ag95_base_link";
  geometry.held_touch_links={"finger"};
  const std::map<std::string,std::string> links{{"wrist",""},{"dh_robotics_ag95_base_link","wrist"},{"finger","dh_robotics_ag95_base_link"}};
  assert(TravelPayloadLinksAllowed(geometry,links));
  geometry.held_touch_links={"wrist"};assert(!TravelPayloadLinksAllowed(geometry,links));
  geometry.held_touch_links={"missing"};assert(!TravelPayloadLinksAllowed(geometry,links));
  geometry.held_touch_links={"finger"};geometry.held_attachment_link="wrist";
  assert(!TravelPayloadLinksAllowed(geometry,links));
  agxcore::TubeLiveSnapshot observed;
  observed.arm.valid=observed.arm.fresh=observed.arm.stationary=true;
  observed.lift.valid=observed.lift.fresh=observed.lift.stationary=true;
  observed.arm.positions[0]=.5;observed.arm.positions[1]=-.5;observed.lift.position_m=.4;
  std::map<std::string,double> model_positions{{"joint1",.5},{"joint2",-.5},{"lifting_joint",.4}};
  assert(TravelModelMatchesMeasured(profile,model_positions,observed));
  model_positions["lifting_joint"]=.3;assert(!TravelModelMatchesMeasured(profile,model_positions,observed));
  model_positions["lifting_joint"]=.4;model_positions["joint1"]=std::numeric_limits<double>::quiet_NaN();
  assert(!TravelModelMatchesMeasured(profile,model_positions,observed));

  for(int interrupt=-1;interrupt<4;++interrupt) {
    auto token=std::make_shared<std::atomic<bool>>(false);
    std::vector<std::string> trace;
    auto visit=[&](int index,const std::string& name) {
      trace.push_back(name);if(interrupt==index)token->store(true);
    };
    TravelMotionSequencePorts ports;
    ports.arm=[&]{visit(0,"arm");return agx_task_workflow::StepResult{true,"arm complete"};};
    ports.lift_needed=[&]()->std::optional<bool>{visit(1,"measured lift");return true;};
    ports.lift_speed=[&]{visit(2,"speed");return agx_task_workflow::StepResult{true,"speed accepted"};};
    ports.lift_position=[&]{visit(3,"position");return agx_task_workflow::StepResult{true,"position accepted"};};
    const auto result=RunTravelMotionSequence(token,ports);
    const std::vector<std::string> success{"arm","measured lift","speed","position"};
    if(interrupt<0)assert(result.ok && trace==success);
    else {
      assert(!result.ok);
      assert(trace==std::vector<std::string>(success.begin(),success.begin()+interrupt+1));
    }
  }
  auto token=std::make_shared<std::atomic<bool>>(false);
  std::vector<std::string> trace;
  TravelMotionSequencePorts ports;
  ports.arm=[] {return agx_task_workflow::StepResult{true,"already default"};};
  ports.lift_needed=[]()->std::optional<bool>{return true;};
  ports.lift_speed=[&]{trace.push_back("failed speed response");return agx_task_workflow::StepResult{false,"RPC failed"};};
  ports.lift_position=[&]{trace.push_back("unexpected position");return agx_task_workflow::StepResult{true,"bad"};};
  assert(!RunTravelMotionSequence(token,ports).ok && trace.size()==1);
  trace.clear();ports.lift_needed=[]()->std::optional<bool>{return false;};
  assert(RunTravelMotionSequence(token,ports).ok && trace.empty());
  ports.lift_needed=[]()->std::optional<bool>{return {};};
  assert(!RunTravelMotionSequence(token,ports).ok && trace.empty());

  // Exercise the production sequence inside the real bounded supervisor: the
  // lift speed RPC returns only after cancellation and the first stop request.
  for(bool rpc_accepted:{true,false}) {
    std::atomic<bool> cancelled{false};std::mutex mutex;
    std::promise<void> stopped;auto stop_done=stopped.get_future().share();
    std::vector<std::string> events;int stop_count=0;
    auto record=[&](const std::string& event){std::lock_guard<std::mutex> lock(mutex);events.push_back(event);};
    auto bounded=RunBoundedMotion([&](const MotionStopToken& current) {
      TravelMotionSequencePorts delayed;
      delayed.arm=[&]{record("arm complete");return agx_task_workflow::StepResult{true,"arm complete"};};
      delayed.lift_needed=[]()->std::optional<bool>{return true;};
      delayed.lift_speed=[&]{
        record("speed request");cancelled=true;stop_done.wait();record("late speed reply");
        return agx_task_workflow::StepResult{rpc_accepted,"speed RPC returned"};
      };
      delayed.lift_position=[&]{record("position request");return agx_task_workflow::StepResult{true,"position accepted"};};
      return RunTravelMotionSequence(current,delayed);
    },[&]{return cancelled?agx_task_workflow::FailureKind::Cancelled:agx_task_workflow::FailureKind::None;},[&]{
      record("stop");if(stop_count++==0)stopped.set_value();
    },1,.1,1);
    assert(!bounded.step.ok && bounded.step.failure==agx_task_workflow::FailureKind::Cancelled && !bounded.exit_required);
    assert(bounded.step.stop==agx_task_workflow::StopState::NotConfirmed && stop_count>=2);
    assert(std::find(events.begin(),events.end(),"position request")==events.end());
    assert(events.back()=="stop");
  }
}
