// Test-only Linux child process. Included after the actual host friend fixture.
#pragma once
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <locale>
#include <spawn.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;

namespace {
constexpr std::size_t kChildReportLimit=4096;
[[noreturn]] void ChildReportExit(int fd,const char* data,std::size_t size,int code) {
  // One small nonblocking pipe write; failure is explicit, never ordinary teardown.
  if(size>kChildReportLimit)std::_Exit(95);
  ssize_t written=-1;
  const double deadline=Now()+1.0;
  do {written=::write(fd,data,size);} while(written<0 && errno==EINTR && Now()<deadline);
  ::close(fd);
  std::_Exit(written==static_cast<ssize_t>(size)?code:95);
}
struct ChildResources {
  LeafProbe leaf;
  std::shared_ptr<Workflow> host;
  ros::NodeHandle node;
  ros::Publisher stop;
  ros::WallTimer inputs;
  ros::AsyncSpinner spinner{2};
  std::atomic<bool> stop_sent{false},input_failed{false};
  std::atomic<double> stop_sent_s{0};
};

[[noreturn]] void RunUnjoinedChild(int report_fd) {
  // Deliberately retain all objects referenced by the detached real worker until
  // _Exit. No child path enters gtest or unwinds through these owners.
  try {
    auto* resources=new ChildResources;
    probe=&resources->leaf;
    resources->leaf.block_run=true;
    ros::NodeHandle private_node("~");
    private_node.setParam("run_id","unjoined-run");
    private_node.setParam("publish_grasp_effect_observations",true);
    resources->host=std::make_shared<Workflow>(true,30.0);
    auto& host=*resources->host;
    WorkflowGraspBindingTestAccess::Prepare(host,{{"A","grasp_test_tube"}});
    WorkflowGraspBindingTestAccess::PauseObservationTimer(host);
    resources->stop=resources->node.advertise<std_msgs::Bool>("/workbench_workflow/stop",1,false);
    resources->spinner.start();
    const double connected_deadline=Now()+2.0;
    while(resources->stop.getNumSubscribers()==0 && Now()<connected_deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if(resources->stop.getNumSubscribers()==0)throw std::runtime_error("child stop input not connected");
    WorkflowGraspBindingTestAccess::InstallHeldEmptyFixture(host);
    host.BeginTask(ActuatingTask("A"));
    auto config=std::make_shared<agx_xarm_pick::ResolvedTubeTaskConfig>();
    config->container.id="fixture-container";config->fixture_profile_id="fixture-profile";
    const agx_tube_tasks::GraspParameters request{"fixture-profile","green",config,""};
    // Keep the existing measured-held fixture fresh; do not replace its guard,
    // observer, receipt policy or captured hold while Grasp is running.
    resources->inputs=resources->node.createWallTimer(ros::WallDuration(.02),
        [resources](const ros::WallTimerEvent&) {
          try {
            WorkflowGraspBindingTestAccess::RefreshHeldInputs(*resources->host);
            if(resources->leaf.run_entered.load() && !resources->stop_sent.exchange(true)) {
              resources->stop_sent_s=Now();
              std_msgs::Bool message;message.data=true;resources->stop.publish(message);
            }
          } catch(...) {resources->input_failed=true;}
        });
    const auto result=host.Grasp(request); // Actual unchanged default 15s monitor timeout.
    const double returned_s=Now();
    const bool stop_seen=WorkflowGraspBindingTestAccess::StopInputObserved(host);
    const bool cancelled=host.RuntimeFailure()==agx_task_workflow::FailureKind::Cancelled;
    agx_task_workflow::TaskResult terminal;terminal.id="A";terminal.type="grasp_test_tube";
    terminal.status="failed";terminal.dispatched=true;host.TaskEvent(terminal);
    WorkflowGraspBindingTestAccess::RunReturned(host);
    std::vector<std::string> published;
    const auto pump=[&] {
      WorkflowGraspBindingTestAccess::Pump(host,[&](const std::string& text) {
        if(published.size()>=64)throw std::runtime_error("excess child status output");
        published.push_back(text);
      });
    };
    const double drain_deadline=Now()+1.0;
    for(;;) {
      pump();
      const auto snapshot=WorkflowGraspBindingTestAccess::Summary(host);
      if(snapshot["snapshot_available"].as<bool>() &&
          !snapshot["slots"][0]["status"]["drain"]["last_sequence"].IsNull())break;
      if(Now()>=drain_deadline)throw std::runtime_error("no child diagnostic acquisition");
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // Later pumps cannot close an actual unjoined producer, even with both task
    // terminal and run-return notifications. No worker flag is assigned by test.
    for(unsigned i=0;i<3;++i)pump();
    const auto summary_json=WorkflowGraspBindingTestAccess::SummaryJson(host);
    const auto summary=YAML::Load(summary_json);
    const auto slot=summary["slots"][0];const auto status=slot["status"];
    std::vector<std::uint64_t> sequences;
    std::vector<std::string> sources;
    bool bound=true,final_event=false;
    for(const auto& text:published) {
      const auto row=YAML::Load(text);const auto kind=row["kind"].as<std::string>();
      bound=bound && row["run_id"].as<std::string>()=="unjoined-run" &&
          row["task_id"].as<std::string>()=="A" &&
          row["operation_id"].as<std::string>()=="unjoined-run/A/grasp";
      if(kind=="grasp_effect_observation") {
        sequences.push_back(row["observation_sequence"].as<std::uint64_t>());
        sources.push_back(row["source_event"]["evidence"]["source"].as<std::string>());
      } else if(kind=="grasp_effect_journal_status") {
        final_event=final_event || row["stage"].as<std::string>()=="final_drain";
      } else throw std::runtime_error("unexpected child diagnostic kind");
    }
    unsigned request_stop_calls=0,observe_stopped_calls=0;
    {std::lock_guard<std::mutex> lock(resources->leaf.mutex);
      request_stop_calls=std::count(resources->leaf.calls.begin(),resources->leaf.calls.end(),"request_stop");
      observe_stopped_calls=std::count(resources->leaf.calls.begin(),resources->leaf.calls.end(),"observe_stopped");}
    const double elapsed=returned_s-resources->stop_sent_s.load();
    const bool ok=resources->leaf.run_entered && !resources->leaf.run_returned &&
        resources->stop_sent && stop_seen && cancelled && !resources->input_failed &&
        !result.ok && result.failure==agx_task_workflow::FailureKind::UnconfirmedStop &&
        result.stop==agx_task_workflow::StopState::NotConfirmed && host.ExitRequired() &&
        status["stage"].as<std::string>()=="progress" &&
        status["producer"]["entered"].as<bool>() && status["producer"]["adapter_bound"].as<bool>() &&
        status["producer"]["monitor_returned"].as<bool>() &&
        !status["producer"]["worker_joined"].as<bool>() &&
        status["producer"]["exit_required"].as<bool>() &&
        status["producer"]["finish_completed"].as<bool>() &&
        status["producer"]["runner_terminal_seen"].as<bool>() &&
        !status["drain"]["final_acquired"].as<bool>() &&
        !slot["terminal_status_attempted"].as<bool>() &&
        !slot["terminal_status_publish_returned"].as<bool>() && !final_event && bound &&
        sequences==std::vector<std::uint64_t>({1,2}) &&
        sources==std::vector<std::string>({"leaf-constructor","leaf-post-join-outcome"}) &&
        request_stop_calls>0 && observe_stopped_calls==0 && std::isfinite(elapsed) && elapsed>=15.0;
    std::ostringstream report;report.imbue(std::locale::classic());
    report<<"{\"schema_version\":1,\"ok\":"<<(ok?"true":"false")
      <<",\"stop_input_seen\":"<<(stop_seen?"true":"false")
      <<",\"context_cancelled\":"<<(cancelled?"true":"false")
      <<",\"worker_run_returned\":"<<(resources->leaf.run_returned?"true":"false")
      <<",\"request_stop_calls\":"<<request_stop_calls
      <<",\"observe_stopped_calls\":"<<observe_stopped_calls
      <<",\"stop_to_return_s\":"<<std::setprecision(17)<<elapsed
      <<",\"failure\":"<<static_cast<int>(result.failure)<<",\"stop\":"<<static_cast<int>(result.stop)
      <<",\"row_count\":"<<sequences.size()<<",\"rows_bound\":"<<(bound?"true":"false")
      <<",\"final_event\":"<<(final_event?"true":"false")
      <<",\"host_exit_required\":"<<(host.ExitRequired()?"true":"false")
      <<",\"sequences\":[";
    for(std::size_t i=0;i<sequences.size();++i) {if(i)report<<',';report<<sequences[i];}
    report<<"],\"row_sources\":[";
    for(std::size_t i=0;i<sources.size();++i) {if(i)report<<',';report<<Json(sources[i]);}
    report<<"],\"summary\":"<<summary_json<<"}\n";
    const auto text=report.str();
    ChildReportExit(report_fd,text.data(),text.size(),ok?0:94);
  } catch(...) {
    constexpr char failure[]="{\"schema_version\":1,\"ok\":false,\"child_exception\":true}\n";
    ChildReportExit(report_fd,failure,sizeof(failure)-1,93);
  }
}

struct ChildOwner {
  pid_t pid=-1;int read_fd=-1,write_fd=-1;
  void ReapKilled() {
    if(pid<=0)return;
    ::kill(pid,SIGKILL);
    int status=0;while(::waitpid(pid,&status,0)<0 && errno==EINTR) {}
    pid=-1;
  }
  ~ChildOwner() {ReapKilled();if(read_fd>=0)::close(read_fd);if(write_fd>=0)::close(write_fd);}
};
struct ChildResult {int status=-1;bool timed_out=false;std::string report;};
ChildResult SpawnUnjoinedChild() {
  ChildOwner child;
  int pipe_fds[2];
  if(::pipe2(pipe_fds,O_CLOEXEC|O_NONBLOCK)!=0)throw std::runtime_error("child report pipe failed");
  child.read_fd=pipe_fds[0];child.write_fd=pipe_fds[1];
  int report_fd=198;while(report_fd==child.read_fd || report_fd==child.write_fd)++report_fd;
  const auto suffix=std::to_string(::getpid());
  std::vector<std::string> args={"workflow_grasp_host_binding_test",
      "--journal-child-fd="+std::to_string(report_fd),"--journal-parent-pid="+suffix,
      "__name:=grasp_journal_unjoined_child_"+suffix,
      "/workbench_workflow/stop:=/grasp_journal_unjoined_"+suffix+"/stop",
      "/workbench_workflow/events:=/grasp_journal_unjoined_"+suffix+"/events",
      "/workbench_workflow/phase:=/grasp_journal_unjoined_"+suffix+"/phase"};
  std::vector<char*> argv;for(auto& arg:args)argv.push_back(&arg[0]);argv.push_back(nullptr);
  posix_spawn_file_actions_t actions;
  if(::posix_spawn_file_actions_init(&actions)!=0)throw std::runtime_error("child spawn actions failed");
  int error=::posix_spawn_file_actions_addclose(&actions,child.read_fd);
  if(!error)error=::posix_spawn_file_actions_adddup2(&actions,child.write_fd,report_fd);
  if(!error)error=::posix_spawn_file_actions_addclose(&actions,child.write_fd);
  pid_t spawned=-1;
  if(!error)error=::posix_spawn(&spawned,"/proc/self/exe",&actions,nullptr,argv.data(),environ);
  ::posix_spawn_file_actions_destroy(&actions);
  if(error)throw std::runtime_error("child posix_spawn failed");
  child.pid=spawned;
  ::close(child.write_fd);child.write_fd=-1;
  ChildResult result;
  const double deadline=Now()+30.0;
  const auto drain=[&] {
    for(;;) {
      char buffer[512];const auto size=::read(child.read_fd,buffer,sizeof(buffer));
      if(size>0) {
        result.report.append(buffer,static_cast<std::size_t>(size));
        if(result.report.size()>kChildReportLimit)throw std::runtime_error("oversized child report");
      } else if(size==0 || errno==EAGAIN) {
        return;
      } else if(errno!=EINTR || Now()>=deadline) {
        throw std::runtime_error("child report read failed");
      }
    }
  };
  while(true) {
    drain();
    const auto waited=::waitpid(child.pid,&result.status,WNOHANG);
    if(waited==child.pid) {
      child.pid=-1;
      // The exited process owns the only remaining writer. Drain trailing bytes.
      drain();break;
    }
    if(waited<0 && errno!=EINTR) {
      if(errno==ECHILD)child.pid=-1;
      throw std::runtime_error("child waitpid failed");
    }
    if(Now()>=deadline) {result.timed_out=true;child.ReapKilled();break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return result; // RAII closes pipe; child is already reaped on every return.
}
}

TEST(GraspJournalUnjoinedProcess, ActualMonitorCannotCloseBlockedWorkerJournal) {
  const auto child=SpawnUnjoinedChild();
  SCOPED_TRACE(child.report);
  ASSERT_FALSE(child.timed_out);ASSERT_TRUE(WIFEXITED(child.status));
  ASSERT_EQ(WEXITSTATUS(child.status),0);
  const auto report=YAML::Load(child.report);
  ASSERT_EQ(report["schema_version"].as<int>(),1);EXPECT_TRUE(report["ok"].as<bool>());
  EXPECT_TRUE(report["stop_input_seen"].as<bool>());EXPECT_TRUE(report["context_cancelled"].as<bool>());
  EXPECT_FALSE(report["worker_run_returned"].as<bool>());
  EXPECT_GT(report["request_stop_calls"].as<unsigned>(),0u);
  EXPECT_EQ(report["observe_stopped_calls"].as<unsigned>(),0u);
  EXPECT_GE(report["stop_to_return_s"].as<double>(),15.0);
  EXPECT_EQ(report["failure"].as<int>(),static_cast<int>(agx_task_workflow::FailureKind::UnconfirmedStop));
  EXPECT_EQ(report["stop"].as<int>(),static_cast<int>(agx_task_workflow::StopState::NotConfirmed));
  EXPECT_EQ(report["row_count"].as<unsigned>(),2u);EXPECT_TRUE(report["rows_bound"].as<bool>());
  EXPECT_EQ(report["sequences"].as<std::vector<std::uint64_t>>(),(std::vector<std::uint64_t>{1,2}));
  EXPECT_EQ(report["row_sources"].as<std::vector<std::string>>(),
      (std::vector<std::string>{"leaf-constructor","leaf-post-join-outcome"}));
  EXPECT_FALSE(report["final_event"].as<bool>());EXPECT_TRUE(report["host_exit_required"].as<bool>());
  const auto slot=report["summary"]["slots"][0];const auto producer=slot["status"]["producer"];
  EXPECT_EQ(slot["status"]["stage"].as<std::string>(),"progress");
  EXPECT_TRUE(producer["entered"].as<bool>());EXPECT_TRUE(producer["adapter_bound"].as<bool>());
  EXPECT_TRUE(producer["monitor_returned"].as<bool>());EXPECT_FALSE(producer["worker_joined"].as<bool>());
  EXPECT_TRUE(producer["exit_required"].as<bool>());EXPECT_TRUE(producer["finish_completed"].as<bool>());
  EXPECT_TRUE(producer["runner_terminal_seen"].as<bool>());
  EXPECT_FALSE(slot["status"]["drain"]["final_acquired"].as<bool>());
  EXPECT_FALSE(slot["terminal_status_attempted"].as<bool>());
  EXPECT_FALSE(slot["terminal_status_publish_returned"].as<bool>());
  std::cout<<"UNJOINED_CHILD_REPORT\t"<<child.report<<std::flush;
}
