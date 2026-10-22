#include "agx_robot_task_runtime/robot_task_ports.hpp"
#include "agx_robot_task_runtime/task_metadata.hpp"
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#define main workflow_node_entry_not_called_by_task_extension_fixture
#include "../src/workflow_node.cpp"
#undef main

namespace example = agx_task_workflow::examples::record_value;

class WorkflowGraspBindingTestAccess {
 public:
  static void Fault(Workflow& host) {host.runtime_fault_=true;}
  static void PriorNavigation(Workflow& host) {
    host.active_profile_=std::make_shared<agx_navigation_tasks::TravelProfile>();
    host.qualified_path_=true;
  }
  static bool HasTravelGuard(const Workflow& host) {
    return host.active_profile_ || host.active_envelope_ || host.travel_policy_ || host.qualified_path_;
  }
  static void NavigationSelections(Workflow& host,
      const std::shared_ptr<const agx_navigation_tasks::TravelProfile>& first,
      const std::shared_ptr<const agx_navigation_tasks::TravelProfile>& second) {
    host.admitted_travel_={{"first",first},{"second",second}};
  }
};

namespace {

agx_task_workflow::Result RunHosted(Workflow& host, std::vector<agx_task_workflow::PreparedTask> tasks) {
  for (auto& task : tasks) {
    const auto execute = task.execute;
    const bool shared_actuators = agx_robot_task_runtime::UsesSharedRobotActuators(task.resources);
    task.execute = [&host, execute, shared_actuators](const agx_task_workflow::PhaseSink& phase) {
      host.DispatchStarted(shared_actuators);
      return execute(phase);
    };
  }
  agx_task_workflow::Ports ports;
  ports.task_begin = [&](const agx_task_workflow::PreparedTask& task) { host.BeginTask(task); };
  ports.cancelled = [&] { return host.RuntimeFailure() == agx_task_workflow::FailureKind::Cancelled; };
  ports.stop = [&] { host.Stop(); };
  ports.phase = [](const std::string&) {};
  ports.task_event = [](const agx_task_workflow::TaskResult&) {};
  ports.runtime_guard = [&] { return host.RuntimeFailure(); };
  ports.continuation_allowed = [&](const agx_task_workflow::PreparedTask& task,
                                   const agx_task_workflow::StepResult& result) {
    return host.ContinuationAllowed(task, result);
  };
  auto result = agx_task_workflow::Run(tasks, ports);
  host.ExpireRuntimePorts();
  return result;
}

agx_task_workflow::TaskDefinition MeasuredObservationTask(
    const agx_task_workflow::TaskRuntimePortsProvider& runtime, ros::ServiceClient client,
    std::function<void()> wait_entered = {}) {
  agx_task_workflow::TaskDefinition definition;
  definition.type = "measured_observation";
  definition.description = "Read one measured observation through a bounded ROS service";
  definition.inputs = {{"expected", agx_task_workflow::InputType::String, false, "",
                        "Expected measured response"}};
  definition.interfaces = {{agx_task_workflow::InterfaceKind::Service, "measured_observation",
                            "/task_extension/measured", "std_srvs/Trigger"}};
  definition.effects = {{agx_task_workflow::effects::ObservationOnly,
                         agx_task_workflow::EffectCondition::TaskSucceeded,
                         "Measured response verified"}};
  definition.prepare = [runtime, client, wait_entered](
      const agx_task_workflow::TaskSpec& spec, agx_task_workflow::PreparationState&) mutable {
    YAML::Node values=YAML::Clone(spec.parameters);
    if (spec.config_path) {
      if(spec.parameters["expected"])throw std::invalid_argument("config and expected are exclusive");
      values=YAML::LoadFile(spec.config_path->string());
    }
    agx_task_workflow::RequireKeys(values,{"expected"},"measurement configuration");
    const auto expected = agx_task_workflow::RequireString(values["expected"], "expected");
    const auto task_id = spec.id;
    agx_task_workflow::PreparedTask task{spec.id, spec.type, "TASK_RUNNING",
        [runtime, client, wait_entered, expected, task_id](const agx_task_workflow::PhaseSink& phase) mutable {
          const auto ports = runtime(task_id);
          phase("MEASURE_VALUE");
          if (wait_entered) wait_entered();
          while (!client.exists()) {
            const auto failure = ports.check();
            if (failure != agx_task_workflow::FailureKind::None) {
              agx_task_workflow::StepResult result{false, "measurement service lost while waiting"};
              result.failure = failure;
              return result;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }
          if (ports.check() != agx_task_workflow::FailureKind::None) {
            agx_task_workflow::StepResult result{false, "measurement interrupted before request"};
            result.failure = ports.check();
            return result;
          }
          std_srvs::Trigger request;
          if (!client.call(request))
            return agx_task_workflow::StepResult{false, "measurement service call failed"};
          const auto failure = ports.check();
          if (failure != agx_task_workflow::FailureKind::None) {
            agx_task_workflow::StepResult result{false, "measurement interrupted after response"};
            result.failure = failure;
            return result;
          }
          if (!request.response.success || request.response.message != expected) {
            agx_task_workflow::StepResult result{false, "measured response did not match the admitted expectation"};
            result.failure = agx_task_workflow::FailureKind::Ordinary;
            result.stop = agx_task_workflow::StopState::Confirmed;
            return result;
          }
          return agx_task_workflow::StepResult{true, "measured response verified"};
        }};
    task.entry_check = [runtime, task_id] {
      const auto failure = runtime(task_id).check();
      if (failure == agx_task_workflow::FailureKind::None)
        return agx_task_workflow::StepResult{true, "observation runtime active"};
      agx_task_workflow::StepResult result{false, "observation runtime interrupted"};
      result.failure = failure;
      return result;
    };
    task.stop = [](double) {
      agx_task_workflow::StepResult result{true, "No actuator owned; no asynchronous request remains"};
      result.stop = agx_task_workflow::StopState::Confirmed;
      return result;
    };
    task.cleanup = [](double) {
      return agx_task_workflow::StepResult{true, "Observation callback is quiescent"};
    };
    return task;
  };
  return definition;
}

class TaskExtensionRos : public ::testing::Test {
 protected:
  ros::NodeHandle node;
  ros::AsyncSpinner spinner{3};
  std::atomic<int> hold_calls{0};
  std::atomic<int> arm_stop_messages{0};
  ros::ServiceServer hold;
  ros::Subscriber arm_stop;

  void SetUp() override {
    interrupted = 0;
    hold = node.advertiseService<std_srvs::Trigger::Request,
                                 std_srvs::Trigger::Response>(
        "/workbench_nav/hold_for_manipulation",
        [this](auto&, auto& response) {
          ++hold_calls;
          response.success = true;
          response.message = "fixture hold";
          return true;
        });
    arm_stop = node.subscribe<std_msgs::String>(
        "/trajectory_execution_event", 10,
        [this](const std_msgs::String::ConstPtr&) { ++arm_stop_messages; });
    spinner.start();
  }
  void TearDown() override {
    interrupted = 0;
    spinner.stop();
  }

  void ExpectNoSharedActuatorStop() {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(hold_calls.load(), 0);
    EXPECT_EQ(arm_stop_messages.load(), 0);
  }
};

TEST_F(TaskExtensionRos, ProductionRecordTaskRunsAndFailsWithoutSharedActuatorCalls) {
  auto host = std::make_shared<Workflow>(false, 1.0);
  agx_task_workflow::TaskRegistry registry;
  bool fail = false;
  example::RegisterRecordValueTask(
      registry, [&](const std::string& id) { return host->RuntimePorts(id); },
      [&](const std::string& value, const agx_task_workflow::TaskRuntimePorts&) {
        agx_task_workflow::StepResult result{!fail, value};
        result.failure = fail ? agx_task_workflow::FailureKind::Ordinary : agx_task_workflow::FailureKind::None;
        return result;
      });
  const auto config = agx_task_workflow::ParseConfig(
      "schema_version: 2\nworkflow_id: record\ntasks: "
      "[{id: record,type: record_value,value: sample}]\n");
  auto recipe = agx_task_workflow::PrepareRecipe(config, registry);
  EXPECT_TRUE(recipe.resources.empty());
  EXPECT_TRUE(recipe.required_interfaces.empty());
  EXPECT_TRUE(RunHosted(*host, recipe.tasks).success);
  host->Stop();
  ExpectNoSharedActuatorStop();

  fail = true;
  recipe = agx_task_workflow::PrepareRecipe(config, registry);
  const auto failed = RunHosted(*host, recipe.tasks);
  EXPECT_FALSE(failed.success);
  EXPECT_EQ(failed.failure, agx_task_workflow::FailureKind::Ordinary);
  EXPECT_EQ(failed.stop, agx_task_workflow::StopState::Confirmed);
  ExpectNoSharedActuatorStop();
}

TEST_F(TaskExtensionRos, ProductionHostExposesHeldArrivalOnlyWhenDeclared) {
  auto host = std::make_shared<Workflow>(false, 1.0);
  agx_task_workflow::PreparedTask plain;
  plain.id = "plain";
  plain.type = "observation";
  host->BeginTask(plain);
  EXPECT_FALSE(host->RuntimePorts("plain").held_arrival);
  host->ExpireRuntimePorts();

  auto held = plain;
  held.id = "held";
  held.requirements = {{agx_robot_task_runtime::requirements::HeldArrival,
                        agx_task_workflow::RequirementTiming::Entry,
                        "fresh measured station hold"}};
  host->BeginTask(held);
  const auto ports = host->RuntimePorts("held");
  ASSERT_TRUE(ports.held_arrival);
  const auto absent = ports.held_arrival("wb-100");
  EXPECT_FALSE(absent.ok);
  EXPECT_NE(absent.detail.find("absent"), std::string::npos);
  host->ExpireRuntimePorts();
  ExpectNoSharedActuatorStop();
}

TEST_F(TaskExtensionRos, QuiescentObservationFailureCanContinueWithoutRobotReadiness) {
  auto host = std::make_shared<Workflow>(false, 1.0);
  agx_task_workflow::TaskRegistry registry;
  int calls = 0;
  example::RegisterRecordValueTask(
      registry, [&](const std::string& id) { return host->RuntimePorts(id); },
      [&](const std::string& value, const agx_task_workflow::TaskRuntimePorts&) {
        ++calls;
        agx_task_workflow::StepResult result{value == "second", value};
        result.failure = result.ok ? agx_task_workflow::FailureKind::None
                                   : agx_task_workflow::FailureKind::Ordinary;
        return result;
      });
  const auto recipe = agx_task_workflow::PrepareRecipe(agx_task_workflow::ParseConfig(
      "schema_version: 2\nworkflow_id: observe\ntasks: "
      "[{id: first,type: record_value,value: first,on_failure: continue},"
      "{id: second,type: record_value,value: second}]\n"), registry);
  const auto result = RunHosted(*host, recipe.tasks);
  EXPECT_EQ(result.completion, agx_task_workflow::RunCompletion::CompletedWithFailures);
  EXPECT_EQ(calls, 2);
  EXPECT_EQ(result.tasks.at(1).status, "succeeded");
  ExpectNoSharedActuatorStop();
}

TEST_F(TaskExtensionRos, DeclaredServiceIsAdmittedAndReturnsMeasuredObservation) {
  auto measured = node.advertiseService<std_srvs::Trigger::Request,
                                        std_srvs::Trigger::Response>(
      "/task_extension/measured", +[](std_srvs::Trigger::Request&,
                                       std_srvs::Trigger::Response& response) {
        response.success = true;
        response.message = "measured:sample";
        return true;
      });
  auto host = std::make_shared<Workflow>(false, 1.0);
  agx_task_workflow::TaskRegistry registry;
  auto client = node.serviceClient<std_srvs::Trigger>("/task_extension/measured");
  agx_task_workflow::RegisterTask(registry, MeasuredObservationTask(
      [&](const std::string& id) { return host->RuntimePorts(id); }, client));
  const auto config = agx_task_workflow::ParseConfig(
      "schema_version: 2\nworkflow_id: measured\ntasks: "
      "[{id: read,type: measured_observation,expected: 'measured:sample'}]\n");
  const auto recipe = agx_task_workflow::PrepareRecipe(config, registry);
  ASSERT_EQ(recipe.required_interfaces.size(), 1u);
  EXPECT_EQ(recipe.required_interfaces.front().endpoint, "/task_extension/measured");
  EXPECT_TRUE(host->ValidateAvailability(recipe.required_interfaces, 1.0).ok);
  EXPECT_TRUE(RunHosted(*host, recipe.tasks).success);
  ExpectNoSharedActuatorStop();
}

TEST_F(TaskExtensionRos, MissingDeclaredServiceFailsAdmissionBeforeDispatch) {
  auto host = std::make_shared<Workflow>(false, 1.0);
  agx_task_workflow::TaskRegistry registry;
  auto client = node.serviceClient<std_srvs::Trigger>("/task_extension/measured");
  agx_task_workflow::RegisterTask(registry, MeasuredObservationTask(
      [&](const std::string& id) { return host->RuntimePorts(id); }, client));
  const auto recipe = agx_task_workflow::PrepareRecipe(agx_task_workflow::ParseConfig(
      "schema_version: 2\nworkflow_id: missing\ntasks: "
      "[{id: read,type: measured_observation,expected: observed}]\n"), registry);
  const auto availability = host->ValidateAvailability(recipe.required_interfaces, 0.05);
  EXPECT_FALSE(availability.ok);
  EXPECT_NE(availability.detail.find("/task_extension/measured"), std::string::npos);
  ExpectNoSharedActuatorStop();
}

TEST_F(TaskExtensionRos, ModuleResolvesAndFreezesConfigWithoutHostTaskBranches) {
  auto path=std::filesystem::temp_directory_path()/
      ("task-module-"+std::to_string(ros::WallTime::now().toNSec())+".yaml");
  std::ofstream(path)<<"expected: measured:sample\n";
  std::atomic<int> calls{0};
  auto measured=node.advertiseService<std_srvs::Trigger::Request,std_srvs::Trigger::Response>(
      "/task_extension/measured",[&](auto&,auto& response) {
        ++calls;response.success=true;response.message="measured:sample";return true;
      });
  auto host=std::make_shared<Workflow>(false,1.0);
  agx_task_workflow::TaskRegistry registry;
  host->RegisterRobotTasks(registry); // Same registration path used by the CLI.
  auto client=node.serviceClient<std_srvs::Trigger>("/task_extension/measured");
  agx_task_workflow::RegisterTask(registry,MeasuredObservationTask(
      [&](const std::string& id){return host->RuntimePorts(id);},client));
  auto config=agx_task_workflow::ParseConfig(
      "schema_version: 2\nworkflow_id: configured\ntasks: [{id: read,type: measured_observation}]\n");
  config.tasks[0].config_path=path;
  auto recipe=agx_task_workflow::PrepareRecipe(config,registry);
  EXPECT_EQ(calls.load(),0);
  std::ofstream(path)<<"invalid_later_field: typo\n";
  host->AdmitRecipe(recipe);
  ASSERT_TRUE(host->ValidateAvailability(recipe.required_interfaces,1.0).ok);
  EXPECT_TRUE(RunHosted(*host,recipe.tasks).success); // Uses the frozen value.
  EXPECT_EQ(calls.load(),1);
  auto later=config.tasks[0];later.id="later";
  config.tasks[0].config_path.reset();config.tasks[0].parameters["expected"]="measured:sample";
  config.tasks.push_back(later);
  EXPECT_THROW(agx_task_workflow::PrepareRecipe(config,registry),agx_task_workflow::PreparationError);
  EXPECT_EQ(calls.load(),1); // A valid earlier invocation was never dispatched.
  std::filesystem::remove(path);
  ExpectNoSharedActuatorStop();
}

TEST_F(TaskExtensionRos, MissingCertificateAndPersistentRobotFaultBothPreventContinuation) {
  for(const bool fault:{false,true}) {
    auto host=std::make_shared<Workflow>(false,1.0);
    agx_task_workflow::TaskRegistry registry;
    int calls=0;
    example::RegisterRecordValueTask(registry,[&](const auto& id){return host->RuntimePorts(id);},
        [&](const auto&,const auto&) {
          ++calls;
          if(fault)WorkflowGraspBindingTestAccess::Fault(*host);
          agx_task_workflow::StepResult result{false,"observation rejected"};
          result.failure=agx_task_workflow::FailureKind::Ordinary;return result;
        });
    auto recipe=agx_task_workflow::PrepareRecipe(agx_task_workflow::ParseConfig(
        "schema_version: 2\nworkflow_id: continuation\ntasks: "
        "[{id: first,type: record_value,value: sample,on_failure: continue},"
        "{id: next,type: record_value,value: later}]\n"),registry);
    if(!fault)recipe.tasks[0].continuation_check={};
    const auto result=RunHosted(*host,recipe.tasks);
    EXPECT_EQ(calls,1);EXPECT_FALSE(result.tasks[1].dispatched);
    EXPECT_EQ(result.completion,agx_task_workflow::RunCompletion::Failed);
    if(fault) {
      agx_task_workflow::PreparedTask next;next.id="later";
      host->BeginTask(next); // New local guards cannot erase a shared fault.
      EXPECT_EQ(host->RuntimeFailure(),agx_task_workflow::FailureKind::Fault);
    }
    ExpectNoSharedActuatorStop();
  }
}

TEST_F(TaskExtensionRos, NextInvocationDropsTravelGuardAndRobotEntryStillMeasuresReadiness) {
  auto host=std::make_shared<Workflow>(false,1.0);
  agx_task_workflow::PreparedTask observation;observation.id="observation";
  WorkflowGraspBindingTestAccess::PriorNavigation(*host);
  host->BeginTask(observation);
  EXPECT_FALSE(WorkflowGraspBindingTestAccess::HasTravelGuard(*host));
  EXPECT_EQ(host->RuntimeFailure(),agx_task_workflow::FailureKind::None);
  host->ExpireRuntimePorts();
  // No preceding observation can manufacture held/stopped robot evidence.
  agx_task_workflow::PreparedTask robot;robot.id="robot";
  robot.resources={agx_robot_task_runtime::resources::Base};
  host->BeginTask(robot);
  EXPECT_FALSE(host->GraspEntryCheck().ok);
  host->ExpireRuntimePorts();
}

TEST_F(TaskExtensionRos, ConflictingRobotObservationContractsRejectBeforeLiveAdmission) {
  const auto file=std::filesystem::path(__FILE__).parent_path()/"workflow_ros_travel.yaml";
  std::ifstream input(file);std::stringstream text;text<<input.rdbuf();
  auto first=std::make_shared<agx_navigation_tasks::TravelProfile>(
      agx_navigation_tasks::ParseTravelProfile(text.str(),file.string()));
  auto second=std::make_shared<agx_navigation_tasks::TravelProfile>(*first);
  // Topics alone match; the observer contract must still prevent mixed robots.
  second->robot.observer.close_command=first->robot.observer.close_command+1;
  auto host=std::make_shared<Workflow>(false,1.0);
  WorkflowGraspBindingTestAccess::NavigationSelections(*host,first,second);
  try {host->ValidateRobotSelections();FAIL()<<"mixed robot selections were accepted";}
  catch(const std::invalid_argument& error) {
    EXPECT_NE(std::string(error.what()).find("same shared physical robot"),std::string::npos);
  }
  ExpectNoSharedActuatorStop();
}

TEST_F(TaskExtensionRos, DeadlineAndCancellationInterruptPostAdmissionWaits) {
  for (const bool cancel : {false, true}) {
    std::atomic<bool> waiting{false};
    auto measured = node.advertiseService<std_srvs::Trigger::Request,
                                          std_srvs::Trigger::Response>(
        "/task_extension/measured", +[](std_srvs::Trigger::Request&,
                                         std_srvs::Trigger::Response& response) {
          response.success = true;
          response.message = "observed";
          return true;
        });
    auto host = std::make_shared<Workflow>(false, cancel ? 1.0 : 0.08);
    agx_task_workflow::TaskRegistry registry;
    auto client = node.serviceClient<std_srvs::Trigger>("/task_extension/measured");
    agx_task_workflow::RegisterTask(registry, MeasuredObservationTask(
        [&](const std::string& id) { return host->RuntimePorts(id); }, client,
        [&] { waiting = true; }));
    const auto recipe = agx_task_workflow::PrepareRecipe(agx_task_workflow::ParseConfig(
        "schema_version: 2\nworkflow_id: interrupted\ntasks: "
        "[{id: read,type: measured_observation,expected: observed}]\n"), registry);
    ASSERT_TRUE(host->ValidateAvailability(recipe.required_interfaces, 1.0).ok);
    measured.shutdown();
    std::thread canceller;
    if (cancel) {
      canceller = std::thread([&] {
        while (!waiting.load()) std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        interrupted = 1;
      });
    }
    const auto result = RunHosted(*host, recipe.tasks);
    if (canceller.joinable()) canceller.join();
    EXPECT_FALSE(result.success);
    EXPECT_EQ(result.failure, cancel ? agx_task_workflow::FailureKind::Cancelled
                                     : agx_task_workflow::FailureKind::Deadline);
    EXPECT_EQ(result.stop, agx_task_workflow::StopState::Confirmed);
    ExpectNoSharedActuatorStop();
    interrupted = 0;
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "task_extension_fixture");
  ros::NodeHandle root_node;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
