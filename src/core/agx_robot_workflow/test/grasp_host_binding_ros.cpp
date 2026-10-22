#include "agx_robot_task_runtime/task_metadata.hpp"
// Actual Workflow/monitor/adapter/store with labeled out-of-line mission leaves.
// Private friend access exercises the production host methods directly.
#include <gtest/gtest.h>
#include <boost/make_shared.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <set>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#define main workflow_node_entry_not_called_by_fixture
#include "../src/workflow_node.cpp"
#undef main

namespace {
agx_task_workflow::PreparedTask ActuatingTask(const std::string& id) {
  agx_task_workflow::PreparedTask task;
  task.id=id;task.type="grasp_test_tube";
  task.resources={agx_robot_task_runtime::resources::Base,agx_robot_task_runtime::resources::Arm,agx_robot_task_runtime::resources::Lift,
                  agx_robot_task_runtime::resources::Gripper,agx_robot_task_runtime::resources::Camera};
  return task;
}

struct LeafProbe {
  std::mutex mutex;
  std::vector<std::string> calls;
  std::thread::id worker;
  bool joined_callback_on_caller = false;
  bool stop_quiescent = true;
  bool throw_after_constructor_effect = false;
  bool block_run = false; // Child-only negative leaf; never released in-process.
  std::atomic<bool> run_entered{false},run_returned{false};
  std::mutex run_mutex;
  std::condition_variable run_condition;
  std::string constructor_source="leaf-constructor";
  std::function<void(const std::string&)> after_effect;
  void Call(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex); calls.push_back(name);
  }
};
LeafProbe* probe = nullptr;  // One test; initialized before any real monitor worker.
agxcore::TubeEvidence Evidence(const std::string& source) {
  return {source,ros::Time::now().toSec(),Now()};
}
class EmptyObservation final : public agxcore::TubeLiveObserver {
 public:
  agxcore::TubeLiveSnapshot Snapshot() const override {
    agxcore::TubeLiveSnapshot s;
    s.gripper.valid=s.gripper.fresh=s.gripper.known_empty=true;
    s.gripper.evidence=Evidence("fixture-empty-observation");
    return s;
  }
};
}

// Necessary out-of-line leaf definitions only. These use the real class layout
// and real TubeEffectJournal. The production inline monitor is NOT replaced.
// Native link-map review must prove tube_grasp_mission.cpp.o is not selected.
namespace agxcore {
TubeGraspMission::TubeGraspMission(ros::NodeHandle* node,
    std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig> config,
    const std::string&, TubeEffectSink sink, std::function<bool()> held)
    : node_(node),resolved_config_(std::move(config)),effects_(std::move(sink)),held_base_(std::move(held)) {
  probe->worker=std::this_thread::get_id();
  probe->Call("constructor");
  if (!effects_.Record(TubeEffectKind::KnownEmpty,Evidence(probe->constructor_source)))
    throw std::runtime_error("fixture constructor effect rejected");
  if (probe->after_effect) probe->after_effect("constructor");
  if (probe->throw_after_constructor_effect) throw std::runtime_error("fixture leaf setup failure");
}
TubeGraspMission::~TubeGraspMission() = default;
TubeMissionStatus TubeGraspMission::Run(const std::string&, const std::string&) {
  probe->Call("run");
  probe->run_entered=true;
  if(probe->block_run) {
    std::unique_lock<std::mutex> lock(probe->run_mutex);
    probe->run_condition.wait(lock,[] {return false;});
  }
  probe->run_returned=true;
  return TubeMissionStatus::kTargetAbsent;
}
void TubeGraspMission::RequestStop() { stop_requested_=true; probe->Call("request_stop"); }
bool TubeGraspMission::FinishStopRequests(double timeout_s) {
  if (!(timeout_s>=0)) throw std::runtime_error("invalid fixture stop budget");
  probe->Call("finish_stop");
  probe->joined_callback_on_caller=probe->worker!=std::this_thread::get_id();
  return probe->stop_quiescent;
}
void TubeGraspMission::RefreshPayloadEvidence() {}
bool TubeGraspMission::ObserveStopped(const TubeEvidence&) {
  probe->Call("observe_stopped");
  return effects_.Record(TubeEffectKind::MeasuredStop,Evidence("leaf-measured-stop"));
}
TubeMissionOutcome TubeGraspMission::Outcome(TubeMissionStatus status) {
  probe->Call("outcome");
  // Real monitor asks Outcome after join/stop observation. This late effect must
  // still reach the actual Grasp-created adapter before its actual Finish.
  effects_.Record(TubeEffectKind::KnownEmpty,Evidence("leaf-post-join-outcome"));
  if (probe->after_effect) probe->after_effect("outcome");
  return effects_.Outcome(status);
}
}

class WorkflowGraspBindingTestAccess {
 public:
  static void InstallHeldEmptyFixture(Workflow& host) {
    host.base_guard_=std::make_unique<agx_robot_task_runtime::BaseGuard>(agx_robot_task_runtime::BaseGuardConfig{
        .25,.35,.05,.02,.01,.5,.015,.035,1.0,.5});
    auto authority=boost::make_shared<Authority>();
    authority->header.stamp=ros::Time::now(); authority->state=Authority::NONE;
    authority->manipulation_hold_active=true; authority->session_id="isolated-base";
    authority->hold_epoch=7; host.AuthorityState(authority);
    auto odom=boost::make_shared<nav_msgs::Odometry>();
    odom->header.stamp=ros::Time::now(); odom->pose.pose.orientation.w=1;
    host.Odom(odom); host.Command(boost::make_shared<geometry_msgs::Twist>());
    auto status=boost::make_shared<actionlib_msgs::GoalStatusArray>();
    status->header.stamp=ros::Time::now(); host.Status(status);
    host.watchdog_ready_=true; host.watchdog_seen_=Now();
    host.live_observer_=std::make_shared<EmptyObservation>();
    const auto captured=host.base_guard_->CaptureHold(host.BaseSnapshot(),Now(),ros::Time::now().toSec());
    if (!captured.ok) throw std::runtime_error(captured.detail);
  }
  // Child-only fresh measured input delivery. Guard/observer/hold ownership was
  // installed once before this callback and is never replaced while Grasp runs.
  static void RefreshHeldInputs(Workflow& host) {
    auto authority=boost::make_shared<Authority>();
    authority->header.stamp=ros::Time::now();authority->state=Authority::NONE;
    authority->manipulation_hold_active=true;authority->session_id="isolated-base";
    authority->hold_epoch=7;host.AuthorityState(authority);
    auto odom=boost::make_shared<nav_msgs::Odometry>();
    odom->header.stamp=ros::Time::now();odom->pose.pose.orientation.w=1;
    host.Odom(odom);host.Command(boost::make_shared<geometry_msgs::Twist>());
    auto status=boost::make_shared<actionlib_msgs::GoalStatusArray>();
    status->header.stamp=ros::Time::now();host.Status(status);
    host.watchdog_ready_=true;host.watchdog_seen_=Now();
  }
  static bool StopInputObserved(Workflow& host) {return host.external_stop_.load();}
  static std::string SummaryJson(Workflow& host) {return host.GraspObservationSummary();}
  static void ReplaceState(Workflow& host,std::shared_ptr<agx_robot_task_runtime::RobotStateStore> state) {
    host.state_=std::move(state);
  }
  static agx_robot_task_runtime::RobotStateSnapshot State(Workflow& host) { return host.state_->Snapshot(); }
  static bool Held(Workflow& host) { return host.BaseReady(true) && host.GraspMayContinue(); }
  static bool EventSubscriberConnected(Workflow& host) {
    return host.event_pub_.getNumSubscribers()>0;
  }
  static void Prepare(Workflow& host,const std::vector<agx_task_workflow::PreparedTask>& tasks) {
    host.PrepareGraspObservations(tasks);
  }
  static void RunReturned(Workflow& host) {
    host.MarkGraspObservationRunReturned();
  }
  static void PauseObservationTimer(Workflow& host) {
    host.grasp_observation_timer_.stop();
  }
  template<class Publish> static void Pump(Workflow& host,Publish&& publish) {
    host.PumpGraspObservations(std::forward<Publish>(publish));
  }
  static YAML::Node Summary(Workflow& host) {
    return YAML::Load(host.GraspObservationSummary());
  }
  static std::shared_ptr<agx_tube_tasks::GraspEffectAdapter> Adapter(Workflow& host,std::size_t index) {
    return host.grasp_observation_slots_.at(index)->adapter;
  }
};

class GraspHostBinding : public ::testing::Test {
 protected:
  LeafProbe leaf;
  std::shared_ptr<Workflow> host;
  agx_tube_tasks::GraspParameters request;
  void SetUp() override {
    probe=&leaf; interrupted=0;
    ros::NodeHandle private_node("~"); private_node.setParam("run_id","binding-run");
    private_node.deleteParam("publish_grasp_effect_observations");
    host=std::make_shared<Workflow>(false,30.0);
    WorkflowGraspBindingTestAccess::InstallHeldEmptyFixture(*host);
    host->BeginTask(ActuatingTask("A"));
    auto config=std::make_shared<agx_xarm_pick::ResolvedTubeTaskConfig>();
    config->container.id="fixture-container";config->fixture_profile_id="fixture-profile";
    request={"fixture-profile","green",config,""};
    ASSERT_TRUE(WorkflowGraspBindingTestAccess::Held(*host));
  }
  void TearDown() override {
    host.reset();probe=nullptr;
    ros::NodeHandle("~").deleteParam("publish_grasp_effect_observations");
  }
  std::vector<agx_robot_task_runtime::EffectRecord> BoundEffects() {
    auto snapshot=WorkflowGraspBindingTestAccess::State(*host);
    std::vector<agx_robot_task_runtime::EffectRecord> bound;
    for (const auto& r:snapshot.effects)
      if (r.operation_id=="binding-run/A/grasp") bound.push_back(r);
    return bound;
  }
};

TEST_F(GraspHostBinding, ConstructorAndPostJoinEffectsUseActualHostBindingAndFinish) {
  std::vector<std::string> seen;
  leaf.after_effect=[&](const std::string& stage) {
    const auto rows=BoundEffects();
    if (rows.empty()) throw std::runtime_error("actual host adapter did not apply leaf effect");
    for (const auto& row:rows) {
      if (row.evidence.run_id!="binding-run" || row.evidence.task_id!="A" ||
          row.evidence.operation_id!="binding-run/A/grasp")
        throw std::runtime_error("actual host effect binding changed");
    }
    seen.push_back(stage);
  };
  const auto result=host->Grasp(request);
  EXPECT_FALSE(result.ok);EXPECT_EQ(result.failure,agx_task_workflow::FailureKind::Ordinary);
  EXPECT_EQ(result.stop,agx_task_workflow::StopState::Confirmed);EXPECT_FALSE(host->ExitRequired());
  EXPECT_TRUE(leaf.joined_callback_on_caller);
  EXPECT_EQ(seen,(std::vector<std::string>{"constructor","outcome"}));
  EXPECT_EQ(leaf.calls,(std::vector<std::string>{"constructor","run","finish_stop","observe_stopped","outcome"}));
  const auto rows=BoundEffects();ASSERT_EQ(rows.size(),2u);
  EXPECT_EQ(rows[0].effect.id,"binding-run/A/grasp/effect/1");
  EXPECT_EQ(rows[1].effect.id,"binding-run/A/grasp/effect/3");
  EXPECT_EQ(result.physical_effect_ids,(std::vector<std::string>{rows[0].effect.id,rows[1].effect.id}));
  EXPECT_EQ(result.state_revision,rows.back().resulting_revision);
  host->BeginTask(ActuatingTask("B"));
  EXPECT_EQ(BoundEffects().size(),2u);  // Original A state evidence survives active B.
}

TEST_F(GraspHostBinding, JoinedMissionAndUnfinishedStopRemainIndependent) {
  leaf.stop_quiescent=false;
  const auto result=host->Grasp(request);
  EXPECT_TRUE(leaf.joined_callback_on_caller);
  EXPECT_TRUE(host->ExitRequired());
  EXPECT_FALSE(result.ok);EXPECT_EQ(result.failure,agx_task_workflow::FailureKind::UnconfirmedStop);
  EXPECT_EQ(result.stop,agx_task_workflow::StopState::NotConfirmed);
  EXPECT_EQ(leaf.calls,(std::vector<std::string>{"constructor","run","finish_stop","outcome"}));
  EXPECT_EQ(BoundEffects().size(),2u);
  EXPECT_THROW(host->BeginTask(ActuatingTask("B")),std::runtime_error);
}

TEST_F(GraspHostBinding, ConstructorExceptionPreservesAppliedEffectWithoutInventingMissionOutcome) {
  leaf.throw_after_constructor_effect=true;
  const auto result=host->Grasp(request);
  EXPECT_FALSE(result.ok);EXPECT_EQ(result.failure,agx_task_workflow::FailureKind::Fault);
  EXPECT_EQ(result.stop,agx_task_workflow::StopState::NotConfirmed);EXPECT_FALSE(host->ExitRequired());
  EXPECT_EQ(leaf.calls,(std::vector<std::string>{"constructor"}));
  ASSERT_EQ(BoundEffects().size(),1u);
  EXPECT_EQ(result.physical_effect_ids,(std::vector<std::string>{"binding-run/A/grasp/effect/1"}));
}

TEST_F(GraspHostBinding, RealRunnerTerminalNotificationFollowsStopAndCleanup) {
  std::vector<std::string> order;
  agx_task_workflow::PreparedTask task{"A","grasp_test_tube","GRASP",[&](const agx_task_workflow::PhaseSink&) {
    order.push_back("grasp");return host->Grasp(request);
  }};
  task.metadata_complete=true; task.stop_timeout_s=1;
  task.resources=ActuatingTask("A").resources;
  task.entry_check=[&] {return host->GraspEntryCheck();};
  task.stop=[&](double) {order.push_back("fake-stop-hook");agx_task_workflow::StepResult r{true,"isolated stop boundary"};r.stop=agx_task_workflow::StopState::Confirmed;return r;};
  task.cleanup=[&](double t) {order.push_back("cleanup");return host->CleanupTask(t);};
  agx_task_workflow::Ports ports;
  ports.phase=[](const std::string&) {};
  ports.cancelled=[] {return false;}; ports.stop=[&] {order.push_back("fallback");};
  ports.runtime_guard=[&] {return host->RuntimeFailure();};
  ports.task_begin=[&](const agx_task_workflow::PreparedTask& t) {host->BeginTask(t);};
  ports.task_event=[&](const agx_task_workflow::TaskResult& t) {order.push_back(t.status);host->TaskEvent(t);};
  const auto result=agx_task_workflow::Run({task},ports);
  EXPECT_FALSE(result.success);ASSERT_EQ(result.tasks.size(),1u);
  EXPECT_EQ(result.tasks[0].status,"failed");
  ASSERT_TRUE(result.tasks[0].dispatched);
  ASSERT_EQ(std::count(order.begin(),order.end(),"grasp"),1);
  ASSERT_EQ(std::count(leaf.calls.begin(),leaf.calls.end(),"run"),1);
  const auto rows=BoundEffects();ASSERT_EQ(rows.size(),2u);
  EXPECT_EQ(rows.front().effect.id,"binding-run/A/grasp/effect/1");
  EXPECT_EQ(rows.back().effect.id,"binding-run/A/grasp/effect/3");
  EXPECT_EQ(result.tasks[0].physical_effect_ids,
      (std::vector<std::string>{rows.front().effect.id,rows.back().effect.id}));
  ASSERT_EQ(std::count(order.begin(),order.end(),"fake-stop-hook"),1);
  ASSERT_EQ(std::count(order.begin(),order.end(),"cleanup"),1);
  const auto grasp=std::find(order.begin(),order.end(),"grasp");
  const auto stop=std::find(order.begin(),order.end(),"fake-stop-hook");
  const auto cleanup=std::find(order.begin(),order.end(),"cleanup");
  const auto terminal=std::find(order.begin(),order.end(),"failed");
  ASSERT_NE(terminal,order.end());
  EXPECT_LT(grasp,stop);EXPECT_LT(stop,cleanup);EXPECT_LT(cleanup,terminal);
}

TEST_F(GraspHostBinding, ExplicitObservationOptInRequiresBoolean) {
  host.reset();
  ros::NodeHandle private_node("~");
  private_node.setParam("publish_grasp_effect_observations",1);
  EXPECT_THROW(std::make_shared<Workflow>(false,30.0),std::invalid_argument);
  private_node.setParam("publish_grasp_effect_observations",std::string("true"));
  EXPECT_THROW(std::make_shared<Workflow>(false,30.0),std::invalid_argument);
}

TEST_F(GraspHostBinding, OptInProducesActualConstructorAndPostJoinRows) {
  host.reset();
  ros::NodeHandle private_node("~"), node;
  private_node.setParam("publish_grasp_effect_observations",true);
  std::mutex received_mutex;
  std::vector<std::string> received;
  auto subscriber=node.subscribe<std_msgs::String>("/workbench_workflow/events",100,
      [&](const std_msgs::String::ConstPtr& message) {
        std::lock_guard<std::mutex> lock(received_mutex);
        received.push_back(message->data);
      });
  ros::AsyncSpinner spinner(2);spinner.start();
  host=std::make_shared<Workflow>(true,30.0);
  const auto connection_deadline=ros::WallTime::now()+ros::WallDuration(2.0);
  while(!WorkflowGraspBindingTestAccess::EventSubscriberConnected(*host) &&
        ros::WallTime::now()<connection_deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  ASSERT_TRUE(WorkflowGraspBindingTestAccess::EventSubscriberConnected(*host));
  // Expanded implementation-stage fixture now performs the actual post-admission
  // preparation. The preserved baseline witness did not claim this absent API.
  WorkflowGraspBindingTestAccess::Prepare(*host,{{"A","grasp_test_tube"}});
  WorkflowGraspBindingTestAccess::InstallHeldEmptyFixture(*host);
  host->BeginTask(ActuatingTask("A"));
  const auto result=host->Grasp(request);
  ASSERT_FALSE(result.ok);
  ASSERT_EQ(result.failure,agx_task_workflow::FailureKind::Ordinary);
  ASSERT_EQ(result.stop,agx_task_workflow::StopState::Confirmed);
  ASSERT_EQ(leaf.calls,(std::vector<std::string>{"constructor","run","finish_stop","observe_stopped","outcome"}));
  const auto effects=BoundEffects();ASSERT_EQ(effects.size(),2u);
  ASSERT_EQ(result.physical_effect_ids,(std::vector<std::string>{effects[0].effect.id,effects[1].effect.id}));
  const auto count_rows=[&] {
    std::lock_guard<std::mutex> lock(received_mutex);
    return std::count_if(received.begin(),received.end(),[](const std::string& text) {
      return text.find("\"kind\":\"grasp_effect_observation\"")!=std::string::npos;
    });
  };
  const auto rows_deadline=ros::WallTime::now()+ros::WallDuration(2.0);
  while(count_rows()<3 && ros::WallTime::now()<rows_deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  EXPECT_GE(count_rows(),3);
}

namespace {
struct Gate {
  std::mutex mutex;
  std::condition_variable condition;
  bool entered=false,released=false;
  void Hold() {
    std::unique_lock<std::mutex> lock(mutex);
    entered=true;condition.notify_all();
    condition.wait(lock,[&] {return released;});
  }
  bool Wait() {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock,std::chrono::seconds(2),[&] {return entered;});
  }
  void Release() {
    std::lock_guard<std::mutex> lock(mutex);
    released=true;condition.notify_all();
  }
};
struct JoinedGate {
  Gate& gate;
  std::thread& thread;
  ~JoinedGate() {gate.Release();if(thread.joinable())thread.join();}
  void Join() {gate.Release();if(thread.joinable())thread.join();}
};
}

class GraspObservationTransport : public GraspHostBinding {
 protected:
  std::vector<std::string> published;
  void Enabled(const std::vector<std::string>& ids={"A"}) {
    host.reset();
    ros::NodeHandle("~").setParam("publish_grasp_effect_observations",true);
    host=std::make_shared<Workflow>(true,30.0);
    std::vector<agx_task_workflow::PreparedTask> tasks;
    for(const auto& id:ids)tasks.push_back({id,"grasp_test_tube"});
    WorkflowGraspBindingTestAccess::Prepare(*host,tasks);
    WorkflowGraspBindingTestAccess::PauseObservationTimer(*host);
    WorkflowGraspBindingTestAccess::InstallHeldEmptyFixture(*host);
    host->BeginTask(ActuatingTask(ids.front()));
  }
  void Pump() {
    WorkflowGraspBindingTestAccess::Pump(*host,[&](const std::string& text) {published.push_back(text);});
  }
  template<class Publish> bool UntilWith(const std::function<bool()>& done,Publish&& publish) {
    const auto deadline=ros::WallTime::now()+ros::WallDuration(2.0);
    do {
      WorkflowGraspBindingTestAccess::Pump(*host,publish);
      if(done())return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    } while(ros::WallTime::now()<deadline);
    return false;
  }
  bool Until(const std::function<bool()>& done) {
    return UntilWith(done,[&](const std::string& text) {published.push_back(text);});
  }
  YAML::Node Slot(std::size_t index=0) {
    auto summary=WorkflowGraspBindingTestAccess::Summary(*host);
    if(!summary["snapshot_available"].as<bool>())throw std::runtime_error("expected quiescent summary");
    return summary["slots"][index];
  }
  YAML::Node Status(std::size_t index=0) {return Slot(index)["status"];}
  void Terminal(const std::string& id="A") {
    agx_task_workflow::TaskResult task;task.id=id;task.type="grasp_test_tube";task.status="failed";
    task.dispatched=true;host->TaskEvent(task);
  }
  std::vector<YAML::Node> Rows(const std::string& kind) {
    std::vector<YAML::Node> rows;
    for(const auto& text:published) {
      auto value=YAML::Load(text);
      if(value["kind"].as<std::string>()==kind)rows.push_back(value);
    }
    return rows;
  }
};

TEST_F(GraspObservationTransport, DisabledAndCheckOnlyDoNotCapture) {
  const auto absent=WorkflowGraspBindingTestAccess::Summary(*host);
  EXPECT_FALSE(absent["requested"].as<bool>());
  EXPECT_FALSE(absent["active"].as<bool>());
  EXPECT_EQ(absent["slots"].size(),0u);
  const std::pair<bool,bool> selections[]={{false,false},{true,false},{false,true}};
  for(const auto& flags:selections) {
    const bool requested=flags.first,execute=flags.second;
    host.reset();ros::NodeHandle("~").setParam("publish_grasp_effect_observations",requested);
    host=std::make_shared<Workflow>(execute,30.0);
    WorkflowGraspBindingTestAccess::Prepare(*host,{{"A","grasp_test_tube"}});
    WorkflowGraspBindingTestAccess::InstallHeldEmptyFixture(*host);host->BeginTask(ActuatingTask("A"));
    const auto result=host->Grasp(request);
    EXPECT_EQ(result.failure,agx_task_workflow::FailureKind::Ordinary);EXPECT_EQ(result.stop,agx_task_workflow::StopState::Confirmed);
    Pump();EXPECT_TRUE(published.empty());
    const auto summary=WorkflowGraspBindingTestAccess::Summary(*host);
    EXPECT_EQ(summary["requested"].as<bool>(),requested);
    EXPECT_FALSE(summary["active"].as<bool>());EXPECT_EQ(summary["slots"].size(),0u);
  }
}

TEST_F(GraspObservationTransport, ActualRowsRemainBoundToAAfterBAndFinalDrainClosesOnce) {
  Enabled({"A","B"});
  const auto a=host->Grasp(request);ASSERT_EQ(a.stop,agx_task_workflow::StopState::Confirmed);
  Terminal();
  WorkflowGraspBindingTestAccess::InstallHeldEmptyFixture(*host);host->BeginTask(ActuatingTask("B"));
  const auto b=host->Grasp(request);ASSERT_EQ(b.stop,agx_task_workflow::StopState::Confirmed);
  Terminal("B");
  ASSERT_TRUE(Until([&] {return Slot(0)["terminal_status_publish_returned"].as<bool>() &&
                              Slot(1)["terminal_status_publish_returned"].as<bool>();}));
  const auto rows=Rows("grasp_effect_observation");ASSERT_EQ(rows.size(),6u);
  for(const auto& id:{std::string("A"),std::string("B")}) {
    std::vector<std::uint64_t> sequence;
    for(const auto& row:rows)if(row["task_id"].as<std::string>()==id) {
      EXPECT_EQ(row["run_id"].as<std::string>(),"binding-run");
      EXPECT_EQ(row["invocation_id"].as<std::string>(),"binding-run/"+id);
      EXPECT_EQ(row["operation_id"].as<std::string>(),"binding-run/"+id+"/grasp");
      sequence.push_back(row["observation_sequence"].as<std::uint64_t>());
      if(sequence.back()==2) {
        EXPECT_TRUE(row["application"].IsNull());EXPECT_TRUE(row["matched_effect"].IsNull());
      } else {
        EXPECT_EQ(row["matched_effect"]["effect"]["id"].as<std::string>(),
                  "binding-run/"+id+"/grasp/effect/"+std::to_string(sequence.back()));
      }
    }
    EXPECT_EQ(sequence,(std::vector<std::uint64_t>{1,2,3}));
  }
  EXPECT_EQ(Rows("grasp_effect_journal_status").size(),2u);
  const auto before=published.size();Terminal("A");Terminal("B");Pump();Pump();
  EXPECT_EQ(published.size(),before);
}

TEST_F(GraspObservationTransport, FutureTaskIsNotTerminalUntilRunReturnAndNeverInventsAProducer) {
  Enabled({"A","B"});
  ASSERT_EQ(host->Grasp(request).stop,agx_task_workflow::StopState::Confirmed);Terminal();
  ASSERT_TRUE(Until([&] {return Slot(0)["terminal_status_publish_returned"].as<bool>();}));
  EXPECT_FALSE(WorkflowGraspBindingTestAccess::Summary(*host)["run_returned"].as<bool>());
  EXPECT_NE(Status(1)["stage"].as<std::string>(),"not_started");
  EXPECT_FALSE(Status(1)["producer"]["entered"].as<bool>());
  for(const auto& row:Rows("grasp_effect_journal_status"))
    EXPECT_NE(row["task_id"].as<std::string>(),"B");
  WorkflowGraspBindingTestAccess::RunReturned(*host);
  ASSERT_TRUE(Until([&] {
    for(const auto& row:Rows("grasp_effect_journal_status"))
      if(row["task_id"].as<std::string>()=="B" && row["stage"].as<std::string>()=="not_started")
        return true;
    return false;
  }));
  EXPECT_TRUE(WorkflowGraspBindingTestAccess::Summary(*host)["run_returned"].as<bool>());
  const auto future=Status(1);
  EXPECT_EQ(future["stage"].as<std::string>(),"not_started");
  EXPECT_FALSE(future["producer"]["entered"].as<bool>());
  EXPECT_FALSE(future["producer"]["adapter_bound"].as<bool>());
  EXPECT_FALSE(future["producer"]["monitor_returned"].as<bool>());
  EXPECT_TRUE(future["producer"]["worker_joined"].IsNull());
  EXPECT_TRUE(future["producer"]["exit_required"].IsNull());
  EXPECT_FALSE(future["producer"]["finish_completed"].as<bool>());
  EXPECT_FALSE(future["producer"]["runner_terminal_seen"].as<bool>());
  EXPECT_FALSE(future["drain"]["final_acquired"].as<bool>());
  EXPECT_TRUE(future["drain"]["last_sequence"].IsNull());
  EXPECT_TRUE(future["drain"]["capture_gap"].IsNull());
  const auto before=published.size();
  WorkflowGraspBindingTestAccess::RunReturned(*host);Pump();Pump();
  EXPECT_EQ(published.size(),before);
}

TEST_F(GraspObservationTransport, NormalBatchRacingTerminalNeedsLaterFinalAcquisition) {
  Enabled();ASSERT_EQ(host->Grasp(request).stop,agx_task_workflow::StopState::Confirmed);
  agx_task_workflow::TaskResult running;running.id="A";running.type="grasp_test_tube";running.status="running";
  host->TaskEvent(running);
  bool marked=false;
  const bool acquired=UntilWith([&] {return marked;},[&](const std::string& text) {
    published.push_back(text);
    const auto value=YAML::Load(text);
    if(!marked && value["kind"].as<std::string>()=="grasp_effect_observation") {
      marked=true;Terminal();
      // Reentrant store and adapter access prove those locks are released.
      (void)WorkflowGraspBindingTestAccess::State(*host);
      auto empty=WorkflowGraspBindingTestAccess::Adapter(*host,0)->TakeObservations();
      EXPECT_EQ(empty.count,0u);
    }
  });
  ASSERT_TRUE(acquired);ASSERT_TRUE(marked);
  EXPECT_FALSE(Status()["drain"]["final_acquired"].as<bool>());
  ASSERT_TRUE(Until([&] {return Status()["drain"]["final_acquired"].as<bool>();}));
  EXPECT_EQ(Rows("grasp_effect_observation").size(),3u);
  EXPECT_TRUE(Slot()["terminal_status_publish_returned"].as<bool>());
}

TEST_F(GraspObservationTransport, MonitorMustReturnAfterPostJoinEffectsBeforeFinalDrain) {
  Enabled();Gate gate;std::exception_ptr failure;agx_task_workflow::StepResult result;
  leaf.after_effect=[&](const std::string& stage) {if(stage=="outcome")gate.Hold();};
  std::thread worker([&] {try {result=host->Grasp(request);}catch(...) {failure=std::current_exception();}});
  JoinedGate cleanup{gate,worker};
  const bool entered=gate.Wait();
  bool early_closed=true,early_returned=true;
  if(entered) {
    // Synthetic run-return notification must not invent missing monitor facts.
    WorkflowGraspBindingTestAccess::RunReturned(*host);
    Terminal();Pump();
    early_closed=Status()["drain"]["final_acquired"].as<bool>();
    early_returned=Status()["producer"]["monitor_returned"].as<bool>();
  }
  cleanup.Join();if(failure)std::rethrow_exception(failure);
  ASSERT_TRUE(entered);EXPECT_TRUE(leaf.joined_callback_on_caller);
  EXPECT_FALSE(early_closed);EXPECT_FALSE(early_returned);
  EXPECT_EQ(result.stop,agx_task_workflow::StopState::Confirmed);
  ASSERT_TRUE(Until([&] {return Status()["drain"]["final_acquired"].as<bool>();}));
  EXPECT_EQ(Rows("grasp_effect_observation").size(),3u);
}

TEST_F(GraspObservationTransport, JoinedExitRequiredClosesEvidenceWithoutClaimingStop) {
  Enabled();leaf.stop_quiescent=false;
  const auto result=host->Grasp(request);Terminal();
  EXPECT_EQ(result.failure,agx_task_workflow::FailureKind::UnconfirmedStop);EXPECT_TRUE(host->ExitRequired());
  ASSERT_TRUE(Until([&] {return Status()["drain"]["final_acquired"].as<bool>();}));
  const auto status=Status();
  EXPECT_TRUE(status["producer"]["worker_joined"].as<bool>());
  EXPECT_TRUE(status["producer"]["exit_required"].as<bool>());
  EXPECT_TRUE(status["producer"]["finish_completed"].as<bool>());
  EXPECT_EQ(Rows("grasp_effect_observation").size(),2u);
}

TEST_F(GraspObservationTransport, InvalidUtf8RowDoesNotChangeEffectsAndIsNotReplayed) {
  Enabled();leaf.constructor_source=std::string("\x80",1);
  const auto result=host->Grasp(request);Terminal();
  EXPECT_EQ(result.failure,agx_task_workflow::FailureKind::Ordinary);EXPECT_EQ(BoundEffects().size(),2u);
  ASSERT_TRUE(Until([&] {return Status()["drain"]["final_acquired"].as<bool>();}));
  const auto transport=Status()["transport"];
  EXPECT_TRUE(transport["transport_gap"].as<bool>());
  // MeasuredStop performs no application and retains the prior invalid gripper
  // evidence. Outcome replaces it; only that third actual row is encodable.
  EXPECT_EQ(transport["failed_rows"].as<std::uint64_t>(),2u);
  EXPECT_EQ(transport["first_failed_sequence"].as<std::uint64_t>(),1u);
  EXPECT_EQ(transport["last_failed_sequence"].as<std::uint64_t>(),2u);
  EXPECT_EQ(transport["rows_publish_returned"].as<std::uint64_t>(),1u);
  const auto rows=Rows("grasp_effect_observation");ASSERT_EQ(rows.size(),1u);
  EXPECT_EQ(rows[0]["observation_sequence"].as<std::uint64_t>(),3u);
  const auto before=published.size();Pump();EXPECT_EQ(published.size(),before);
}

TEST_F(GraspObservationTransport, RowAndFinalStatusPublishFailuresStaySeparate) {
  Enabled();ASSERT_EQ(host->Grasp(request).stop,agx_task_workflow::StopState::Confirmed);Terminal();
  unsigned failed_rows=0,failed_status=0;
  const bool attempted=UntilWith([&] {return failed_status!=0;},[&](const std::string& text) {
    auto value=YAML::Load(text);
    if(value["kind"].as<std::string>()=="grasp_effect_observation" &&
        value["observation_sequence"].as<unsigned>()==1) {
      ++failed_rows;throw std::runtime_error("test row publication failed");
    }
    if(value["kind"].as<std::string>()=="grasp_effect_journal_status") {
      ++failed_status;throw std::runtime_error("test final status publication failed");
    }
    published.push_back(text);
  });
  ASSERT_TRUE(attempted);EXPECT_EQ(failed_rows,1u);EXPECT_EQ(failed_status,1u);
  EXPECT_TRUE(Slot()["terminal_status_attempted"].as<bool>());
  EXPECT_FALSE(Slot()["terminal_status_publish_returned"].as<bool>());
  EXPECT_TRUE(Status()["drain"]["final_acquired"].as<bool>());
  EXPECT_EQ(Status()["transport"]["rows_publish_returned"].as<unsigned>(),2u);
  EXPECT_TRUE(Status()["transport"]["transport_gap"].as<bool>());
  const auto before=published.size();Pump();Pump();EXPECT_EQ(published.size(),before);
}

TEST_F(GraspObservationTransport, InFlightPumpDoesNotBlockSummaryOtherPumpOrCleanup) {
  Enabled();ASSERT_EQ(host->Grasp(request).stop,agx_task_workflow::StopState::Confirmed);
  Gate gate;std::exception_ptr failure;bool held=false;
  std::thread worker([&] {
    try {
      UntilWith([&] {return held;},[&](const std::string&) {
        if(!held) {held=true;gate.Hold();}
      });
    } catch(...) {failure=std::current_exception();}
  });
  JoinedGate cleanup{gate,worker};
  const bool entered=gate.Wait();bool summary_available=true;unsigned competing_calls=0;
  bool cleanup_returned=false;
  if(entered) {
    summary_available=WorkflowGraspBindingTestAccess::Summary(*host)["snapshot_available"].as<bool>();
    WorkflowGraspBindingTestAccess::Pump(*host,[&](const std::string&) {++competing_calls;});
    (void)host->RuntimeFailure();(void)host->StopTask(.1);(void)host->CleanupTask(.1);
    cleanup_returned=true;
  }
  cleanup.Join();if(failure)std::rethrow_exception(failure);
  EXPECT_TRUE(entered);EXPECT_FALSE(summary_available);EXPECT_EQ(competing_calls,0u);
  EXPECT_TRUE(cleanup_returned);
}

TEST_F(GraspObservationTransport, UnrepresentableIdentityOnlyCreatesIndexedUnavailableSummary) {
  const std::string id(505,'t');Enabled({id});
  const auto result=host->Grasp(request);Terminal(id);
  EXPECT_EQ(result.failure,agx_task_workflow::FailureKind::Ordinary);EXPECT_EQ(result.stop,agx_task_workflow::StopState::Confirmed);
  Pump();EXPECT_TRUE(published.empty());
  auto slot=Slot();EXPECT_TRUE(slot["status"].IsNull());
  EXPECT_EQ(slot["recipe_index"].as<unsigned>(),0u);
  EXPECT_EQ(slot["unavailable_reason"].as<std::string>(),"identity_invalid");
}

namespace {
struct ArmedSnapshotGate {
  Gate gate;
  std::atomic<bool> armed{false};
};
}

TEST_F(GraspObservationTransport, BusyBoundAdapterDoesNotInventAcquisitionFacts) {
  for(bool prior_gap:{false,true}) {
    published.clear();Enabled();
    auto clock=std::make_shared<ArmedSnapshotGate>();
    auto state=std::make_shared<agx_robot_task_runtime::RobotStateStore>([clock] {
      if(clock->armed.exchange(false))clock->gate.Hold();
      return ros::Time::now().toSec();
    },Now);
    WorkflowGraspBindingTestAccess::ReplaceState(*host,state);
    ASSERT_EQ(host->Grasp(request).stop,agx_task_workflow::StopState::Confirmed);
    const auto before=state->Snapshot();
    auto adapter=WorkflowGraspBindingTestAccess::Adapter(*host,0);
    ASSERT_TRUE(adapter);
    unsigned next=4;
    if(prior_gap) {
      // Capture rejects this overlength diagnostic source before Snapshot but
      // the real no-application stop remains accepted. Queue space stays free.
      ASSERT_TRUE(adapter->Apply({next++,agxcore::TubeEffectKind::MeasuredStop,
                                 Evidence(std::string(513,'x'))}));
    }
    // An explicit test delivery on the actual host-created adapter, after Grasp
    // returned but before its terminal TaskEvent. This is real lock contention,
    // not a claim that the monitor normally emits after its completed return.
    // MeasuredStop snapshots without applying another physical effect.
    bool applied=false;std::exception_ptr failure;
    clock->armed=true;
    std::thread producer([&] {
      try {
        applied=adapter->Apply({next,agxcore::TubeEffectKind::MeasuredStop,
                               Evidence("test-contended-stop")});
      } catch(...) {failure=std::current_exception();}
    });
    JoinedGate cleanup{clock->gate,producer};
    const bool entered=clock->gate.Wait();
    bool sequence_unknown=false,gap_unknown=false,final_acquired=true;
    std::size_t rows_while_busy=999;
    if(entered) {
      Pump();
      const auto status=Status();
      sequence_unknown=status["drain"]["last_sequence"].IsNull();
      gap_unknown=status["drain"]["capture_gap"].IsNull();
      final_acquired=status["drain"]["final_acquired"].as<bool>();
      rows_while_busy=Rows("grasp_effect_observation").size();
    }
    cleanup.Join();if(failure)std::rethrow_exception(failure);
    ASSERT_TRUE(entered);ASSERT_TRUE(applied);
    EXPECT_TRUE(sequence_unknown);EXPECT_TRUE(gap_unknown);
    EXPECT_FALSE(final_acquired);EXPECT_EQ(rows_while_busy,0u);
    Terminal();
    ASSERT_TRUE(Until([&] {return Status()["drain"]["final_acquired"].as<bool>();}));
    const auto status=Status();
    EXPECT_EQ(status["drain"]["last_sequence"].as<unsigned>(),next);
    EXPECT_EQ(status["drain"]["capture_gap"].as<bool>(),prior_gap);
    const auto rows=Rows("grasp_effect_observation");
    ASSERT_EQ(rows.size(),4u);
    for(std::size_t i=0;i<rows.size();++i)
      EXPECT_EQ(rows[i]["observation_sequence"].as<std::uint64_t>(),i==3?next:i+1);
    const auto after=state->Snapshot();
    EXPECT_EQ(after.revision,before.revision);
    EXPECT_EQ(after.effects.size(),before.effects.size());
    EXPECT_EQ(after.gripper.occupancy,before.gripper.occupancy);
  }
}

TEST_F(GraspObservationTransport, FullEligibleSlotsTakeAtMostOneBatchWithoutStarvation) {
  Enabled({"A","B"});
  ASSERT_EQ(host->Grasp(request).stop,agx_task_workflow::StopState::Confirmed);
  WorkflowGraspBindingTestAccess::InstallHeldEmptyFixture(*host);host->BeginTask(ActuatingTask("B"));
  ASSERT_EQ(host->Grasp(request).stop,agx_task_workflow::StopState::Confirmed);
  auto a=WorkflowGraspBindingTestAccess::Adapter(*host,0);
  auto b=WorkflowGraspBindingTestAccess::Adapter(*host,1);
  ASSERT_TRUE(a);ASSERT_TRUE(b);
  unsigned next_a=4,next_b=4;
  const auto fill=[](const std::shared_ptr<agx_tube_tasks::GraspEffectAdapter>& adapter,
                     unsigned& sequence,unsigned count) {
    for(unsigned i=0;i<count;++i) {
      if(!adapter->Apply({sequence++,agxcore::TubeEffectKind::MeasuredStop,
                         Evidence("test-fairness-stop")}))
        throw std::runtime_error("test stop observation rejected");
    }
  };
  // Both slots stay eligible for nonfinal draining (no terminal notification), and every
  // successful drain is refilled before the next tick. These are actual adapter
  // rows, not a substitute queue or a counter manually injected into Workflow.
  fill(a,next_a,61);fill(b,next_b,61);
  std::set<std::string> served;
  const auto deadline=ros::WallTime::now()+ros::WallDuration(2.0);
  do {
    const auto first=published.size();Pump();
    unsigned count=0;std::set<std::string> tick_tasks;
    for(std::size_t i=first;i<published.size();++i) {
      const auto row=YAML::Load(published[i]);
      if(row["kind"].as<std::string>()=="grasp_effect_observation") {
        ++count;tick_tasks.insert(row["task_id"].as<std::string>());
      }
    }
    EXPECT_LE(count,64u);EXPECT_LE(tick_tasks.size(),1u);
    if(count) {
      EXPECT_EQ(count,64u);
      for(const auto& id:tick_tasks) {
        served.insert(id);
        if(id=="A")fill(a,next_a,64);
        else if(id=="B")fill(b,next_b,64);
        else ADD_FAILURE()<<"Unexpected original task identity: "<<id;
      }
    }
    if(served.size()==2)break;
    // Test-only retry tolerates spurious adapter try-lock failures.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  } while(ros::WallTime::now()<deadline);
  EXPECT_EQ(served,(std::set<std::string>{"A","B"}));
  EXPECT_FALSE(Status(0)["drain"]["final_acquired"].as<bool>());
  EXPECT_FALSE(Status(1)["drain"]["final_acquired"].as<bool>());
}

#include "grasp_unjoined_process_fixture.hpp"

int main(int argc,char** argv) {
  int child_fd=-1;pid_t parent_pid=-1;
  try {
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      for(const auto& prefix:{std::string("--journal-child-fd="),std::string("--journal-parent-pid=")}) {
        if(arg.compare(0,prefix.size(),prefix)!=0)continue;
        std::size_t used=0;const auto value=std::stol(arg.substr(prefix.size()),&used);
        if(used!=arg.size()-prefix.size() || value<=0 || value>std::numeric_limits<int>::max())
          std::_Exit(96);
        if(prefix=="--journal-child-fd=")child_fd=static_cast<int>(value);
        else parent_pid=static_cast<pid_t>(value);
      }
    }
  } catch(...) {std::_Exit(96);}
  if(child_fd>=0) {
    // Parent death during an outer watchdog must not orphan the blocked worker.
    if(parent_pid<=0 || ::prctl(PR_SET_PDEATHSIG,SIGKILL)!=0 || ::getppid()!=parent_pid)
      std::_Exit(96);
    try {
      ros::init(argc,argv,"grasp_journal_unjoined_child");
      (void)new ros::NodeHandle; // Retained through child _Exit; no ROS teardown.
      RunUnjoinedChild(child_fd);
    } catch(...) {
      constexpr char failed[]="{\"schema_version\":1,\"ok\":false,\"child_setup_exception\":true}\n";
      ChildReportExit(child_fd,failed,sizeof(failed)-1,92);
    }
  }
  ros::init(argc,argv,"grasp_host_binding_fixture");
  ros::NodeHandle root_node;  // Retain ROS startup through all test owners.
  ::testing::InitGoogleTest(&argc,argv);
  // Most cases invoke host callbacks directly; the actual ROS publication case
  // owns its local AsyncSpinner. Workflow retains the action client's own thread.
  return RUN_ALL_TESTS();
}
