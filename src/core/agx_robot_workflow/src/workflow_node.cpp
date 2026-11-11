#include "agx_robot_task_runtime/robot_task_ports.hpp"
#include "agx_robot_task_runtime/task_metadata.hpp"
#include "agx_navigation_tasks/navigate_task.hpp"
#include "agx_tube_tasks/grasp_test_tube_task.hpp"
#include "agx_place_tasks/place_task.hpp"
#include "agx_place_tasks/place_motion.hpp"
#include "agx_place_tasks/pick_task.hpp"
#include "agx_place_tasks/pick_execution.hpp"
#include "agx_place_tasks/onboard_positions.hpp"
#include "agx_place_tasks/place_pose.hpp"
#include "agx_robot_task_runtime/base_guard.hpp"
#include "agx_task_workflow/execution_context.hpp"
#include "agx_task_workflow/task_runtime_ports.hpp"
#include "agx_task_workflow/examples/record_value_task.hpp"
#include "agx_tube_tasks/grasp_task.hpp"
#include "agx_tube_tasks/grasp_execution.hpp"
#include "agx_tube_tasks/grasp_observation_json.hpp"
#include "agx_navigation_tasks/navigation_task.hpp"
#include "agx_navigation_tasks/travel_policy.hpp"
#include "agx_navigation_tasks/travel_motion.hpp"
#include "agx_navigation_tasks/travel_envelope_ros.hpp"
#include <agx_workbench_navigation/AcquireBaseHold.h>
#include <agx_workbench_navigation/PrepareBaseDeparture.h>
#include <agx_workbench_navigation/PrepareNavigationRoute.h>
#include <agx_workbench_navigation/ReleaseBaseHold.h>
#include <agx_workbench_navigation/TravelReadiness.h>
#include <bt_task_msgs/LiftMotorSrv.h>
#include <ros/master.h>
#include <ros/service.h>
#include "agx_xarm_pick/tube_mission_runner.hpp"
#include <actionlib/client/simple_action_client.h>
#include <agx_workbench_navigation/NavigateToWorkbenchAction.h>
#include <agx_workbench_navigation/BaseAuthorityState.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <csignal>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <set>
#include <limits>
#include <random>

using Authority = agx_workbench_navigation::BaseAuthorityState;
using Client = actionlib::SimpleActionClient<agx_workbench_navigation::NavigateToWorkbenchAction>;
using Clock = std::chrono::steady_clock;
static volatile std::sig_atomic_t interrupted = 0;
static void Signal(int) { interrupted = 1; }
static double Now() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
static void Poll() { std::this_thread::sleep_for(std::chrono::milliseconds(25)); }
static double LoadTaskTimeout(ros::NodeHandle& private_node) {
  double timeout=900.0;
  if(private_node.hasParam("task_timeout_s")) {
    XmlRpc::XmlRpcValue value;
    if(!private_node.getParam("task_timeout_s",value))
      throw std::invalid_argument("cannot read explicit task_timeout_s");
    if(value.getType()==XmlRpc::XmlRpcValue::TypeInt)timeout=static_cast<int>(value);
    else if(value.getType()==XmlRpc::XmlRpcValue::TypeDouble)timeout=static_cast<double>(value);
    else throw std::invalid_argument("task_timeout_s must be a finite positive number");
  }
  const double now=Now();
  if(!std::isfinite(timeout) || timeout<=0 || !std::isfinite(now) ||
      !std::isfinite(now+timeout) || now+timeout<=now)
    throw std::invalid_argument("task_timeout_s requires a finite positive duration and representable future steady deadline");
  return timeout;
}
static std::string Json(const std::string& value) {
  std::ostringstream out; out << '"';
  for (unsigned char c : value) {
    if (c=='"' || c=='\\') out << '\\' << c;
    else if (c<32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
    else out << c;
  }
  out << '"'; return out.str();
}

static std::string TaskJson(const agx_task_workflow::TaskResult& task) {
  std::ostringstream out;
  out << "{\"id\":" << Json(task.id) << ",\"type\":" << Json(task.type)
      << ",\"status\":" << Json(task.status) << ",\"dispatched\":" << (task.dispatched?"true":"false")
      << ",\"detail\":" << Json(task.detail) << ",\"failed_phase\":" << Json(task.failed_phase)
      << ",\"elapsed_wall_s\":" << task.elapsed_wall_s
      << ",\"failure_kind\":" << int(task.failure)
      << ",\"stop_confirmed\":" << (task.stop==agx_task_workflow::StopState::Confirmed?"true":"false")
      << ",\"state_revision\":" << task.state_revision << ",\"physical_effect_ids\":[";
  for (std::size_t i=0;i<task.physical_effect_ids.size();++i) {
    if(i)out<<',';out<<Json(task.physical_effect_ids[i]);
  }
  out << "]}";
  return out.str();
}

template<class Service>
static std::optional<Service> BoundedService(ros::ServiceClient client,Service service,double timeout_s,
    std::function<bool()> interrupted={},std::shared_ptr<std::atomic<bool>> active={}) {
  if(active && active->exchange(true))return std::nullopt;
  auto promise=std::make_shared<std::promise<std::optional<Service>>>();auto future=promise->get_future();
  std::thread([client,service,promise,active]() mutable {
    try {if(client.call(service))promise->set_value(service);else promise->set_value(std::nullopt);}
    catch(...) {promise->set_value(std::nullopt);}
    if(active)active->store(false);
  }).detach();
  const double deadline=Now()+timeout_s;
  while(future.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready) {
    if((interrupted && interrupted()) || Now()>=deadline)return std::nullopt;
    future.wait_for(std::chrono::milliseconds(25));
  }
  return future.get();
}

class Workflow : public std::enable_shared_from_this<Workflow> {
  friend class WorkflowGraspBindingTestAccess;
 public:
  Workflow(bool publish_progress,double task_timeout_s)
      : client_("/workbench_nav/navigate", true), task_timeout_s_(task_timeout_s) {
    run_id_="workflow/"+std::to_string(ros::WallTime::now().toNSec());
    // A bounded process/session identity independent of user-visible run IDs.
    // Even a deliberately reused run ID must not reuse an invocation token.
    std::random_device random;std::ostringstream route_session;route_session<<std::hex;
    for(int i=0;i<4;++i)route_session<<random()<<'-';route_session_=route_session.str();
    ros::NodeHandle private_node("~");
    grasp_observations_execute_=publish_progress;
    if(private_node.hasParam("publish_grasp_effect_observations")) {
      XmlRpc::XmlRpcValue selected;
      if(!private_node.getParam("publish_grasp_effect_observations",selected) ||
          selected.getType()!=XmlRpc::XmlRpcValue::TypeBoolean)
        throw std::invalid_argument("publish_grasp_effect_observations must be bool");
      grasp_observations_requested_=static_cast<bool>(selected);
    }
    std::string requested_run_id;
    if(private_node.getParam("run_id",requested_run_id)) {
      if(requested_run_id.empty() || requested_run_id.size()>256)
        throw std::invalid_argument("run_id must contain 1 to 256 characters");
      run_id_=requested_run_id;
    }
    state_=std::make_shared<agx_robot_task_runtime::RobotStateStore>([]{return ros::Time::now().toSec();},Now);
    runtime_binding_=std::make_unique<agx_robot_task_runtime::RobotTaskRuntimeBinding>(state_);
    authority_sub_ = node_.subscribe("/workbench_nav/authority_state", 10, &Workflow::AuthorityState, this);
    odom_sub_ = node_.subscribe("/odom", 10, &Workflow::Odom, this);
    command_sub_ = node_.subscribe("/cmd_vel", 10, &Workflow::Command, this);
    goal_sub_ = node_.subscribe("/move_base/goal", 10, &Workflow::Goal, this);
    status_sub_ = node_.subscribe("/move_base/status", 10, &Workflow::Status, this);
    arm_execution_sub_=node_.subscribe<actionlib_msgs::GoalStatusArray>("/execute_trajectory/status",10,
        [this](const actionlib_msgs::GoalStatusArray::ConstPtr& m){ArmStatus(*m,true);});
    arm_planning_sub_=node_.subscribe<actionlib_msgs::GoalStatusArray>("/move_group/status",10,
        [this](const actionlib_msgs::GoalStatusArray::ConstPtr& m){ArmStatus(*m,false);});
    stop_sub_ = node_.subscribe<std_msgs::Bool>("/workbench_workflow/stop", 1,
      [this](const std_msgs::Bool::ConstPtr& m) { if (m->data) external_stop_=true; });
    watchdog_sub_ = node_.subscribe<std_msgs::Bool>("/workbench_workflow/watchdog_ready", 1,
      [this](const std_msgs::Bool::ConstPtr& m) { watchdog_ready_=m->data; watchdog_seen_=Now(); });
    if (publish_progress) {
    phase_pub_ = node_.advertise<std_msgs::String>("/workbench_workflow/phase", 10, true);
    event_pub_ = node_.advertise<std_msgs::String>("/workbench_workflow/events", 100, false);
    readiness_pub_=node_.advertise<agx_workbench_navigation::TravelReadiness>("/workbench_nav/travel_readiness",1,false);
    heartbeat_ = node_.createWallTimer(ros::WallDuration(.1), [this](const ros::WallTimerEvent&) {
      std_msgs::String msg; { std::lock_guard<std::mutex> lock(mutex_); msg.data=phase_; }
      phase_pub_.publish(msg);
    });
    }
    hold_ = node_.serviceClient<std_srvs::Trigger>("/workbench_nav/hold_for_manipulation");
    acquire_hold_=node_.serviceClient<agx_workbench_navigation::AcquireBaseHold>("/workbench_nav/acquire_base_hold");
    prepare_departure_=node_.serviceClient<agx_workbench_navigation::PrepareBaseDeparture>("/workbench_nav/prepare_base_departure");
    release_hold_=node_.serviceClient<agx_workbench_navigation::ReleaseBaseHold>("/workbench_nav/release_base_hold");
    prepare_route_=node_.serviceClient<agx_workbench_navigation::PrepareNavigationRoute>("/workbench_nav/prepare_navigation_route");
    arm_stop_ = node_.advertise<std_msgs::String>("/trajectory_execution_event", 1);
  }
  ~Workflow() {
    if(runtime_binding_)runtime_binding_->Expire();
    grasp_observation_timer_.stop();
    readiness_timer_.stop();
    heartbeat_.stop();
    for (auto* sub : {&authority_sub_,&odom_sub_,&command_sub_,&goal_sub_,&status_sub_,&stop_sub_,&watchdog_sub_,&arm_execution_sub_,&arm_planning_sub_})
      sub->shutdown();
    // Completion flags own no Workflow memory. The normal path joins every
    // worker; a terminal timeout retains this owner through summary flush and
    // reaches _Exit. Destruction on another exception still cannot block or
    // destroy a joinable thread while a transport reply is outstanding.
    std::lock_guard<std::mutex> lock(stop_mutex_);
    lift_stop_job_.retry=hold_stop_job_.retry=false;
    ReapStopRpcsLocked();
    for(auto* job:{&lift_stop_job_,&hold_stop_job_})if(job->worker.joinable()) {
      exit_required_=true;job->worker.detach();
    }
  }
  bool Cancelled() const { return interrupted || external_stop_ || !ros::ok(); }
  void RegisterRobotTasks(agx_task_workflow::TaskRegistry& registry) {
    // Snapshot installation/legacy ROS parameters once. Task-owned resolvers
    // subsequently prepare immutable invocation values using file reads only.
    ros::NodeHandle private_node("~");std::string mode,path,override_path;
    node_.param<std::string>("/workbench_workflow/execution_mode",mode,"hardware");
    private_node.param<std::string>("execution_mode",mode,mode);
    if(mode!="hardware" && mode!="simulation")throw std::invalid_argument("execution_mode must be hardware or simulation");
    travel_mode_=mode=="simulation"?agx_navigation_tasks::TravelMode::Simulation:agx_navigation_tasks::TravelMode::Hardware;
    agx_navigation_tasks::NavigationInstallation navigation;
    navigation.execution_mode=travel_mode_;
    node_.getParam("/workbench_workflow/travel_profile",path);
    private_node.getParam("travel_profile",override_path);
    navigation.default_travel_profile=override_path.empty()?path:override_path;
    path.clear();node_.getParam("/workbench_mission/registry_path",path);
    navigation.registry_path=path;
    agx_tube_tasks::TubeInstallation tubes;
    try {tubes.legacy_config=agxcore::SnapshotLegacyTubeTaskConfig(node_);}
    catch(...) {tubes.legacy_error=std::current_exception();}
    agx_navigation_tasks::NavigateTaskPorts nav_ports;
    nav_ports.composed_navigation=[this](const auto& request,const auto& phase){return ComposedNavigate(request,phase);};
    nav_ports.cancelled=[this]{return RuntimeFailure()!=agx_task_workflow::FailureKind::None;};
    nav_ports.entry_check=[this]{return NavigationEntryCheck();};
    nav_ports.stop=[this](double timeout){return StopTask(timeout);};
    nav_ports.cleanup=[this](double timeout){return CleanupTask(timeout);};
    nav_ports.admit=[this](const std::string& id,const agx_navigation_tasks::NavigateParameters& request) {
      admitted_travel_.emplace(id,request.travel_profile);
      return agx_task_workflow::StepResult{true,"navigation selection bound"};
    };
    nav_ports.begin=[this](const auto& request){BeginNavigation(request);};
    nav_ports.continuation_check=[this](const auto& result){return RobotContinuationAllowed(result);};
    agx_navigation_tasks::RegisterNavigationTasks(registry,navigation,nav_ports);
    agx_tube_tasks::TubeTaskPorts tube_ports;
    tube_ports.grasp=[this](const auto& request){return Grasp(request);};
    tube_ports.cancelled=nav_ports.cancelled;
    tube_ports.entry_check=[this]{return GraspEntryCheck();};
    tube_ports.stop=nav_ports.stop;tube_ports.cleanup=nav_ports.cleanup;
    tube_ports.admit=[this](const std::string& id,const agx_tube_tasks::GraspParameters& request) {
      if(!request.config)return agx_task_workflow::StepResult{false,"grasp requires an immutable configuration"};
      admitted_configs_.emplace(id,request.config);
      return agx_task_workflow::StepResult{true,"grasp selection bound"};
    };
    tube_ports.begin=[this](const auto& request){BeginGrasp(request);};
    tube_ports.continuation_check=nav_ports.continuation_check;
    agx_tube_tasks::RegisterTubeTasks(registry,tubes,tube_ports);
    agx_place_tasks::PlaceInstallation place;
    std::string rack_path,rack_id;
    private_node.param<std::string>("onboard_rack_profile",rack_path,"");
    private_node.param<std::string>("onboard_rack_id",rack_id,"");
    if(rack_path.empty()!=rack_id.empty())
      throw std::invalid_argument("onboard_rack_profile and onboard_rack_id must be set together");
    if(!rack_path.empty()) {
      place.rack=std::make_shared<const agx_place_tasks::OnboardRackProfile>(
          agx_place_tasks::LoadRackProfile(rack_path,rack_id));
      onboard_rack_profile_=place.rack;
      onboard_rack_installed_=true;
    }
    private_node.param<double>("place_velocity_scaling",place_motion_.velocity_scaling,place_motion_.velocity_scaling);
    private_node.param<double>("place_acceleration_scaling",place_motion_.acceleration_scaling,place_motion_.acceleration_scaling);
    private_node.param<double>("place_eef_step_m",place_motion_.eef_step_m,place_motion_.eef_step_m);
    private_node.param<double>("place_gripper_force",place_motion_.gripper_force,place_motion_.gripper_force);
    private_node.param<double>("place_gripper_speed",place_motion_.gripper_speed,place_motion_.gripper_speed);
    std::string positions_path;
    private_node.param<std::string>("onboard_positions",positions_path,std::string());
    if(!positions_path.empty()){
      try{
        measured_positions_=std::make_shared<const agx_place_tasks::PositionTable>(
            agx_place_tasks::LoadPositions(positions_path));
      }catch(const std::exception& error){
        ROS_FATAL("onboard position table '%s' is unusable: %s",positions_path.c_str(),error.what());
        std::exit(1);
      }
    }
    agx_place_tasks::PlaceTaskPorts place_ports;
    place_ports.place=[this](const auto& request){return Place(request);};
    place_ports.positions=measured_positions_;
    place_ports.cancelled=nav_ports.cancelled;
    place_ports.entry_check=[this]{return PlaceEntryCheck();};
    place_ports.stop=nav_ports.stop;place_ports.cleanup=nav_ports.cleanup;
    place_ports.continuation_check=nav_ports.continuation_check;
    agx_place_tasks::RegisterPlaceTasks(registry,place,place_ports);
    agx_place_tasks::PickTaskPorts pick_ports;
    pick_ports.positions=measured_positions_;
    pick_ports.pick=[this](const auto& request){return Pick(request);};
    pick_ports.cancelled=nav_ports.cancelled;
    pick_ports.entry_check=[this]{return PickEntryCheck();};
    pick_ports.stop=nav_ports.stop;pick_ports.cleanup=nav_ports.cleanup;
    pick_ports.continuation_check=nav_ports.continuation_check;
    agx_place_tasks::RegisterPickTasks(registry,place,pick_ports);
  }
  void AdmitRecipe(const agx_task_workflow::PreparedRecipe& recipe) {
    admitted_travel_.clear();admitted_configs_.clear();envelopes_.clear();
    for(const auto& task:recipe.tasks) {
      try {
        if(task.admission_check) {
          const auto admitted=task.admission_check();
          if(!admitted.ok || admitted.failure!=agx_task_workflow::FailureKind::None)
            throw std::invalid_argument(admitted.detail);
        }
      } catch(const std::exception& error) {
        agx_task_workflow::TaskSpec spec;spec.id=task.id;spec.type=task.type;
        throw agx_task_workflow::PreparationError(spec,error.what());
      }
    }
    ValidateRobotSelections();
    if(!agx_robot_task_runtime::UsesSharedRobotActuators(recipe.resources))return;
    std::string authority, parking;
    if (!node_.getParam("/base_command_arbiter/config_file",authority) ||
        !node_.getParam("/workbench_mission/parking_config",parking))
      throw std::invalid_argument("loaded base-guard configuration is missing");
    const auto authority_config=YAML::LoadFile(authority);
    const auto parking_limits=YAML::LoadFile(parking)["blind_motion"];
    std::lock_guard<std::mutex> lock(mutex_);
    position_limit_=parking_limits["position_tolerance_m"].as<double>();
    yaw_limit_=parking_limits["max_yaw_drift_rad"].as<double>();
    receipt_limit_=authority_config["freshness"]["odometry_receipt_s"].as<double>();
    header_limit_=authority_config["freshness"]["odometry_header_s"].as<double>();
    future_skew_=authority_config["freshness"]["maximum_header_future_skew_s"].as<double>();
    zero_=authority_config["limits"]["zero_epsilon"].as<double>();
    linear_=authority_config["handover"]["stop_linear_speed_mps"].as<double>();
    angular_=authority_config["handover"]["stop_angular_speed_radps"].as<double>();
    dwell_=authority_config["handover"]["stop_dwell_s"].as<double>();
    base_guard_=std::make_unique<agx_robot_task_runtime::BaseGuard>(agx_robot_task_runtime::BaseGuardConfig{
        receipt_limit_,header_limit_,future_skew_,linear_,angular_,dwell_,
        position_limit_,yaw_limit_,1.0,.5});
  }
  void ValidateRobotSelections() {
    // The current physical adapter controls one xarm6/lift/gripper. Admission
    // must not let a task command different devices from the shared observer.
    for(const auto& nav:admitted_travel_) {
      const auto& robot=nav.second->robot;
      const auto& shared=admitted_travel_.begin()->second->robot;
      if(robot.arm_group!=shared.arm_group || robot.arm_joint_state_topic!=shared.arm_joint_state_topic ||
          robot.gripper_state_topic!=shared.gripper_state_topic || robot.lift_state_topic!=shared.lift_state_topic ||
          robot.lift_service!=shared.lift_service || robot.observer.fully_open_command!=shared.observer.fully_open_command ||
          robot.observer.close_command!=shared.observer.close_command ||
          robot.observer.command_tolerance!=shared.observer.command_tolerance)
        throw std::invalid_argument("navigation profiles must bind the same shared physical robot");
      for(std::size_t i=0;i<nav.second->arm.size();++i)
        if(nav.second->arm[i].name!="joint"+std::to_string(i+1))
          throw std::invalid_argument("travel profile must name xarm6 joints in order");
      if(robot.arm_group!="xarm6" || nav.second->arm.size()!=6)
        throw std::invalid_argument("travel profile must select the installed xarm6 robot");
      for(const auto& grasp:admitted_configs_) {
        const auto& c=*grasp.second;
        if(c.container.id==nav.second->container_profile_id &&
            (std::max(c.container.body_diameter_m,c.container.cap_diameter_m)+c.container.diameter_tolerance_m>
                2*nav.second->held_max_radius_m ||
             c.container.body_height_m+c.container.cap_height_m+2*c.container.height_tolerance_m>
                nav.second->held_max_length_m))
          throw std::invalid_argument("grasp container geometry exceeds the qualified travel envelope");
        if(robot.arm_joint_state_topic!=c.robot.joint_state_topic ||
            robot.gripper_state_topic!=c.robot.gripper_state_topic ||
            robot.lift_state_topic!=c.motion.lift_state_topic || robot.lift_service!=c.motion.lift_service_name ||
            robot.observer.fully_open_command!=c.motion.fully_open_command ||
            robot.observer.close_command!=c.motion.close_command ||
            robot.observer.command_tolerance!=c.motion.gripper_command_tolerance)
          throw std::invalid_argument("travel and grasp must bind the same physical robot and gripper observation contract");
      }
    }
    // Grasp-only or multiple-grasp recipes must also select one physical robot.
    for(const auto& item:admitted_configs_) {
      const auto& c=*item.second;const auto& shared=*admitted_configs_.begin()->second;
      if(c.robot.joint_state_topic!=shared.robot.joint_state_topic ||
          c.robot.gripper_state_topic!=shared.robot.gripper_state_topic ||
          c.robot.gripper_command_topic!=shared.robot.gripper_command_topic ||
          c.motion.lift_state_topic!=shared.motion.lift_state_topic ||
          c.motion.lift_service_name!=shared.motion.lift_service_name ||
          c.motion.fully_open_command!=shared.motion.fully_open_command ||
          c.motion.close_command!=shared.motion.close_command ||
          c.motion.gripper_command_tolerance!=shared.motion.gripper_command_tolerance)
        throw std::invalid_argument("grasp configurations must bind the same shared physical robot");
    }
    // Read-only effective-envelope probes are admission, never preparation.
    for(const auto& item:admitted_travel_)
      if(!envelopes_.count(item.second->config_id))
        envelopes_[item.second->config_id]=agx_navigation_tasks::AdmitTravelEnvelope(node_,*item.second,3.0);
  }
  void BeginNavigation(const agx_navigation_tasks::NavigateParameters& request) {
    active_profile_=request.travel_profile;
    if(!active_profile_)throw std::invalid_argument("navigation has no prepared travel profile");
    active_envelope_=envelopes_.at(active_profile_->config_id);
    const auto& robot=active_profile_->robot;
    live_observer_=agxcore::MakeTubeLiveObserver(node_,
        {robot.arm_joint_state_topic,robot.gripper_state_topic,robot.lift_state_topic},robot.observer);
    lift_stop_=node_.serviceClient<bt_task_msgs::LiftMotorSrv>(robot.lift_service);
  }
  void BeginGrasp(const agx_tube_tasks::GraspParameters& request) {
    if(!request.config)throw std::invalid_argument("grasp has no prepared configuration");
    const auto bound=agx_xarm_pick::BindTubeTaskMotion(*request.config,request.cap_color);
    live_observer_=agxcore::MakeTubeLiveObserver(node_,*request.config,bound);
    lift_stop_=node_.serviceClient<bt_task_msgs::LiftMotorSrv>(bound.lift_service_name);
  }
  agx_task_workflow::StepResult ValidateAvailability(const std::vector<agx_task_workflow::InterfaceRequirement>& requirements,
                                      double timeout_s=10.0) {
    if(!std::isfinite(timeout_s) || timeout_s<=0)
      return {false,"interface admission requires a finite positive timeout"};
    const double deadline=Now()+timeout_s;
    std::string missing;
    while (!Cancelled() && Now()<deadline) {
      ros::master::V_TopicInfo topics;
      XmlRpc::XmlRpcValue args,reply,state;
      args[0]=ros::this_node::getName();
      const bool graph_ready=ros::master::getTopics(topics) &&
          ros::master::execute("getSystemState",args,reply,state,true);
      missing.clear();
      for (const auto& dependency:requirements) {
        bool ready=graph_ready && !dependency.endpoint.empty();
        if (ready && dependency.kind==agx_task_workflow::InterfaceKind::Service)
          ready=ros::service::exists(dependency.endpoint,false);
        else if (ready && dependency.kind==agx_task_workflow::InterfaceKind::Topic &&
                 dependency.direction==agx_task_workflow::InterfaceDirection::Command) {
          ready=false;
          if (state.getType()==XmlRpc::XmlRpcValue::TypeArray && state.size()==3)
            for (int i=0;i<state[1].size();++i)
              if (std::string(state[1][i][0])==dependency.endpoint && state[1][i][1].size()>0) ready=true;
        } else if (ready) {
          const auto endpoint=dependency.endpoint+(dependency.kind==agx_task_workflow::InterfaceKind::Action?"/status":"");
          const auto type=dependency.kind==agx_task_workflow::InterfaceKind::Action?"actionlib_msgs/GoalStatusArray":dependency.ros_type;
          ready=std::any_of(topics.begin(),topics.end(),[&](const ros::master::TopicInfo& topic) {
            return topic.name==endpoint && topic.datatype==type;
          });
        }
        if (!ready) {missing=dependency.key+" ("+dependency.endpoint+")";break;}
      }
      if (missing.empty() && graph_ready) return {true,"declared task interfaces are available"};
      Poll();
    }
    return {false,"required task interface unavailable: "+missing};
  }
  agx_task_workflow::StepResult Navigate(const agx_navigation_tasks::NavigateParameters& request) {
    if(!context_)return {false,"navigation requires an active task execution context"};
    const auto before=RuntimeFailure();
    if(before!=agx_task_workflow::FailureKind::None) {
      agx_task_workflow::StepResult denied{false,"workflow interrupted before navigation dispatch"};
      denied.failure=before;return denied;
    }
    const auto stop=[&](agx_task_workflow::FailureKind cause,const std::string& detail) {
      // The observed task cause precedes cancellation callbacks/readiness withdrawal.
      agx_task_workflow::StepResult result{false,detail};result.failure=cause;
      client_.cancelGoal();return result;
    };
    if(route_.route_id.empty() || route_.invocation_id!=route_invocation_)
      return {false,"25: navigation has no route admitted for this invocation"};
    { std::lock_guard<std::mutex> lock(mutex_); navigation_started_=true; owned_goal_.clear();owned_terminal_=false; target_station_=request.target; }
    agx_workbench_navigation::NavigateToWorkbenchGoal goal; goal.workbench_id=request.target;
    goal.invocation_id=route_invocation_;goal.route_id=route_.route_id;
    goal.task_deadline_s=context_->Deadline();
    client_.sendGoal(goal); goal_sent_=true;
    while (true) {
      const auto cause=RuntimeFailure();
      if(cause!=agx_task_workflow::FailureKind::None)return stop(cause,"navigation cancelled, faulted or task deadline exceeded");
      if(client_.getState().isDone())break;
      if (!NavigationAvailable()) {
        runtime_fault_=true;
        return stop(RuntimeFailure(),"navigation server, arbiter state or workflow watchdog lost");
      }
      Poll();
    }
    auto result=client_.getResult();
    const bool succeeded=client_.getState()==actionlib::SimpleClientGoalState::SUCCEEDED;
    const auto cause=RuntimeFailure(); // Action result/state retrieval is an interruption boundary.
    if(cause!=agx_task_workflow::FailureKind::None)return stop(cause,"navigation cancelled, faulted or task deadline exceeded");
    if (!result) return {false,"navigation returned no result"};
    navigation_detail_=std::to_string(result->code)+": "+result->message;
    if (!succeeded || result->code!=0)
      return {false,navigation_detail_};
    return {true,navigation_detail_};
  }
  agx_task_workflow::StepResult VerifyStop() {
    const double deadline=Now()+10; double since=0;
    while (RuntimeFailure()==agx_task_workflow::FailureKind::None && Now()<deadline) {
      if (BaseReady(false,true)) {
        if (!since) since=Now();
        if (Now()-since>=dwell_) return {true,"owned move_base goal terminal; fresh measured stop and zero output"};
      } else since=0;
      Poll();
    }
    return {false,"stopped handoff not confirmed (owned goal, authority, zero output or fresh odometry)"};
  }
  agx_task_workflow::StepResult Hold() {
    const double deadline=Now()+10;
    while (!Cancelled() && Now()<deadline) {
      auto response=CallHold();
      if (response.ok && BaseReady(true)) {
        std::lock_guard<std::mutex> lock(guard_mutex_);
        const auto sample=BaseSnapshot();
        const auto captured=base_guard_->CaptureHold(sample,Now(),ros::Time::now().toSec(),target_station_);
        if (captured.ok) return response;
      }
      Poll();
    }
    return {false,"persistent manipulation hold was not confirmed"};
  }
  agx_task_workflow::StepResult Grasp(const agx_tube_tasks::GraspParameters& request) {
    auto* journal=active_grasp_observation_;
    if(journal)journal->facts.fetch_or(JournalEntered,std::memory_order_release);
    {std::lock_guard<std::mutex> lock(guard_mutex_);grasp_stop_reason_.clear();}
    const auto owner=shared_from_this();
    const auto held=[owner] {return owner->GraspMayContinue();};
    agx_tube_tasks::GraspExecutionPorts ports;
    ports.state=state_;ports.run_id=run_id_;ports.task_id=active_task_id_;
    ports.held_base=[this]{return BaseReady(true) && GraspMayContinue();};
    ports.refresh_custody=[this]{RefreshCustody();};
    ports.effects_bound=[this,&journal](const auto& effects){journal=BindGraspObservation(journal,effects);};
    ports.run=[this,held](const auto& selected,auto sink) {
      return agxcore::RunTubeMissionMonitored(node_,selected.fixture_profile,selected.cap_color,
          held,selected.config,sink,{},held);
    };
    ports.mission_returned=[this,&journal](const auto& result) {
      if(journal)journal->facts.fetch_or(JournalReturned |
          (result.outcome.worker_joined?static_cast<unsigned>(JournalJoined):0u) |
          (result.exit_required?static_cast<unsigned>(JournalExitRequired):0u),std::memory_order_release);
      if(!active_profile_ && result.observer)live_observer_=result.observer;
      if(result.exit_required)exit_required_=true;
    };
    ports.effects_finished=[&journal]{if(journal)journal->facts.fetch_or(JournalFinished,std::memory_order_release);};
    ports.stop_reason=[this]{std::lock_guard<std::mutex> lock(guard_mutex_);return grasp_stop_reason_;};
    ports.runtime_failure=[this]{return RuntimeFailure();};
    return agx_tube_tasks::ExecuteGrasp(request,ports);
  }
  agx_task_workflow::StepResult Pick(const agx_place_tasks::PickParameters& request) {
    if(!state_)return {false,"pick requires the robot state store"};
    agx_place_tasks::PickExecutionPorts ports;
    ports.state=state_;
    ports.run_id=run_id_;
    ports.task_id=active_task_id_;
    ports.held_base=[this]{return BaseReady(true) && GraspMayContinue();};
    ports.refresh_custody=[this]{RefreshCustody();};
    ports.run=[this](const agx_place_tasks::PickParameters& parameters) {
      const auto stop=[this]{return Cancelled() || RuntimeFailure()!=agx_task_workflow::FailureKind::None;};
      agx_place_tasks::PickMotionOutcome motion;
      if(parameters.uses_measured_position&&parameters.measured){
        agx_place_tasks::PlaceMotionConfig config=place_motion_;
        if(measured_positions_){
          config.velocity_scaling=measured_positions_->velocity_scaling;
          config.acceleration_scaling=measured_positions_->acceleration_scaling;
          config.eef_step_m=measured_positions_->eef_step_m;
          config.gripper_open_position=measured_positions_->open_before_descent;
          config.grasp_close_position=measured_positions_->close_on_cap;
          config.grasp_held_margin=measured_positions_->held_margin;
          config.gripper_force=measured_positions_->gripper_force;
          config.gripper_speed=measured_positions_->gripper_speed;
          // The measured rows were taught with these heights; keep them exact.
          config.descent_open_position=measured_positions_->open_before_descent;
        }
        agx_place_tasks::PickTarget target;
        target.frame=parameters.measured_frame;
        target.tip_link=parameters.rack?parameters.rack->tip_link:std::string("tcp_link");
        target.approach_m={parameters.measured->x,parameters.measured->y,parameters.measured->approach_z};
        target.grasp_m={parameters.measured->x,parameters.measured->y,parameters.measured->grasp_z};
        target.approach_orientation_xyzw=
            agx_place_tasks::QuaternionFromRpyDeg(parameters.measured_rpy_deg);
        target.orientation_xyzw=agx_place_tasks::QuaternionFromRpyDeg(parameters.measured_rpy_deg);
        target.orientation_xyzw=agx_place_tasks::RotateAboutToolZ(
            target.orientation_xyzw,parameters.measured_yaw_offset_deg*M_PI/180.0);
        motion=agx_place_tasks::RunPickMotionAt(target,config,stop);
      }else{
        motion=agx_place_tasks::RunPickMotion(*parameters.rack,parameters.slot,place_motion_,stop);
      }
      agx_place_tasks::PickMeasurement measurement;
      measurement.grasped=motion.grasped;
      measurement.grasp_uncertain=motion.grasp_uncertain;
      measurement.worker_joined=motion.worker_joined;
      measurement.measured_stop_confirmed=motion.measured_stop_confirmed;
      measurement.detail=motion.detail;
      measurement.evidence=motion.evidence;
      return measurement;
    };
    ports.runtime_failure=[this]{return RuntimeFailure();};
    ports.stop_reason=[this]{std::lock_guard<std::mutex> lock(guard_mutex_);return grasp_stop_reason_;};
    return agx_place_tasks::ExecutePick(request,ports);
  }
  agx_task_workflow::StepResult Place(const agx_place_tasks::PlaceParameters& request) {
    if(!state_)return {false,"place requires the robot state store"};
    agx_place_tasks::PlaceExecutionPorts ports;
    ports.state=state_;
    ports.run_id=run_id_;
    ports.task_id=active_task_id_;
    ports.held_base=[this]{return BaseReady(true) && GraspMayContinue();};
    ports.refresh_custody=[this]{RefreshCustody();};
    // Every move is bounded and cancellable through the same stop path as the
    // grasp mission; the motion itself never blocks past a stop request.
    ports.run=[this](const agx_place_tasks::PlaceParameters& parameters) {
      const auto stop=[this]{return Cancelled() || RuntimeFailure()!=agx_task_workflow::FailureKind::None;};
      if(!parameters.uses_measured_position){
        return agx_place_tasks::RunPlaceMotion(*parameters.rack,parameters.slot,place_motion_,stop);
      }
      agx_place_tasks::PlaceMotionConfig config=place_motion_;
      if(measured_positions_){
        config.velocity_scaling=measured_positions_->velocity_scaling;
        config.acceleration_scaling=measured_positions_->acceleration_scaling;
        config.eef_step_m=measured_positions_->eef_step_m;
        config.gripper_open_position=measured_positions_->open_before_descent;
        config.descent_open_position=measured_positions_->open_before_descent;
        config.gripper_force=measured_positions_->gripper_force;
        config.gripper_speed=measured_positions_->gripper_speed;
      }
      agx_place_tasks::PickTarget target;
      target.frame=parameters.measured_frame;
      target.tip_link=parameters.rack?parameters.rack->tip_link:std::string("tcp_link");
      target.approach_m={parameters.measured_x,parameters.measured_y,parameters.measured_approach_z};
      target.grasp_m={parameters.measured_x,parameters.measured_y,parameters.measured_release_z};
      target.approach_orientation_xyzw=
          agx_place_tasks::QuaternionFromRpyDeg(parameters.measured_rpy_deg);
      target.orientation_xyzw=agx_place_tasks::RotateAboutToolZ(
          target.approach_orientation_xyzw,parameters.measured_yaw_offset_deg*M_PI/180.0);
      return agx_place_tasks::RunPlaceMotionAt(target,config,stop);
    };
    ports.runtime_failure=[this]{return RuntimeFailure();};
    ports.stop_reason=[this]{std::lock_guard<std::mutex> lock(guard_mutex_);return grasp_stop_reason_;};
    return agx_place_tasks::ExecutePlace(request,ports);
  }
  agx_task_workflow::FailureKind RuntimeFailure() const {
    if(context_) {
      const auto cause=context_->Check();
      if(cause==agx_task_workflow::FailureKind::Fault)runtime_fault_=true;
      return cause;
    }
    if (Cancelled()) return agx_task_workflow::FailureKind::Cancelled;
    if (runtime_fault_ || (motion_started_ && active_envelope_ && !active_envelope_->Guard())) {
      runtime_fault_=true;
      return agx_task_workflow::FailureKind::Fault;
    }
    return agx_task_workflow::FailureKind::None;
  }
  void BeginTask(const agx_task_workflow::PreparedTask& task) {
    std::string pending;
    if(!StopRpcsQuiescent(&pending) || exit_required_) {
      exit_required_=true;
      throw std::runtime_error("previous task stop workers are not quiescent; "+pending);
    }
    {std::lock_guard<std::mutex> lock(stop_mutex_);stop_rpcs_sealed_=false;}
    EndReadiness();
    {std::lock_guard<std::mutex> lock(travel_mutex_);travel_failure_detail_.clear();}
    // Admission is complete. Only tasks declaring one of the host's shared
    // actuators acquire its base/arm/lift/gripper emergency-stop path.
    motion_started_=agx_robot_task_runtime::UsesSharedRobotActuators(task.resources);
    active_task_id_=task.id;
    active_grasp_observation_=FindGraspObservationSlot(task.id);
    // Guard selection never leaks from a previous invocation. Persistent
    // fault/cancel latches, custody and outstanding stop-worker ownership remain.
    active_profile_.reset();active_envelope_.reset();
    travel_policy_.reset();qualified_path_=false;
    robot_work_started_=robot_work_started_ || motion_started_;
    context_=std::make_shared<agx_task_workflow::ExecutionContext>(task_timeout_s_,Now,[this]{return Cancelled();},
        agx_task_workflow::PhaseSink{},[this] {
          if(motion_started_ && active_envelope_ && !active_envelope_->Guard())runtime_fault_=true;
          return runtime_fault_.load();
        });
    std::function<agx_task_workflow::StepResult(const std::string&)> held_arrival;
    const bool needs_arrival=std::any_of(task.requirements.begin(),task.requirements.end(),[](const auto& requirement) {
      return requirement.kind==agx_robot_task_runtime::requirements::HeldArrival;
    });
    if(needs_arrival)held_arrival=[this](const std::string& station) {
      std::lock_guard<std::mutex> lock(guard_mutex_);
      const auto sample=BaseSnapshot();
      const bool matches=base_guard_ && base_guard_->ArrivalMatches(
          station,sample,Now(),ros::Time::now().toSec());
      return agx_task_workflow::StepResult{matches,matches?"fresh measured held arrival":"held arrival is absent, stale or at another station"};
    };
    runtime_binding_->Activate(run_id_,task.id,run_id_+"/"+task.id,context_,std::move(held_arrival));
  }
  agx_task_workflow::StepResult NavigationEntryCheck() {
    if (RuntimeFailure()!=agx_task_workflow::FailureKind::None) return {false,"workflow interrupted before task entry"};
    // Bounded subscriber/action handshake after graph admission; no commands.
    const double deadline=Now()+3.0;
    while(RuntimeFailure()==agx_task_workflow::FailureKind::None && Now()<deadline) {
      if(NavigationAvailable())return {true,"navigation entry interfaces"};
      Poll();
    }
    return {false,"navigation entry interfaces did not become ready within the startup bound"};
  }
  agx_task_workflow::StepResult GraspEntryCheck() {
    return {BaseReady(true) && GraspMayContinue(),"measured held-base entry guard"};
  }
  agx_task_workflow::StepResult PlaceEntryCheck() {
    if(!onboard_rack_installed_)return {false,"no measured onboard rack profile is installed"};
    return {BaseReady(true) && GraspMayContinue(),"measured held-base entry guard"};
  }
  agx_task_workflow::StepResult PickEntryCheck() {
    if(!onboard_rack_installed_)return {false,"no measured onboard rack profile is installed"};
    return {BaseReady(true) && GraspMayContinue(),"measured held-base entry guard"};
  }
  agx_task_workflow::StepResult StopTask(double timeout_s) {
    // Return evidence inside the lifecycle wrapper's own monotonic deadline.
    const double deadline=Now()+std::max(0.0,timeout_s-std::min(.2,timeout_s*.1));
    agxcore::TubeEvidence stop_request{"workflow_stop",ros::Time::now().toSec(),Now()};
    {
      std::lock_guard<std::mutex> lock(guard_mutex_);
      if(base_guard_)base_guard_->BeginEmergencyStop(stop_request.receipt_steady_s,stop_request.source_stamp_s);
    }
    EndReadiness();Stop(true);
    agx_robot_task_runtime::BaseGuardResult base_stop{false,"base stop was not observed"};
    bool producers_stopped=false,arm_lift_stopped=false,final_rpcs_joined=false;
    std::string pending;
    while (Now()<deadline) {
      const bool quiescent=StopRpcsQuiescent(&pending);
      if(quiescent && !final_rpcs_joined) {
        // Earlier stop RPCs may have preceded a late accepted motion command.
        // Observe only after the coalesced final stops have returned and joined.
        stop_request={"workflow_stop_rpcs_joined",ros::Time::now().toSec(),Now()};
        std::lock_guard<std::mutex> lock(guard_mutex_);
        if(base_guard_)base_guard_->BeginEmergencyStop(stop_request.receipt_steady_s,stop_request.source_stamp_s);
        final_rpcs_joined=true;
      }
      RefreshCustody();
      {
        std::lock_guard<std::mutex> lock(guard_mutex_);
        const auto sample=BaseSnapshot();
        if(base_guard_)base_stop=base_guard_->ObserveEmergencyStop(sample,Now(),ros::Time::now().toSec());
      }
      producers_stopped=ProducersStoppedAfter(stop_request);
      arm_lift_stopped=live_observer_ && live_observer_->Snapshot().ArmLiftStoppedSince(stop_request);
      if (!exit_required_ && final_rpcs_joined && quiescent && base_stop.ok && producers_stopped && arm_lift_stopped) {
        agx_task_workflow::StepResult result{true,"owned stop RPCs joined; measured base, arm and lift stop confirmed afterward"};
        result.stop=agx_task_workflow::StopState::Confirmed;result.state_revision=state_->Snapshot().revision;
        return result;
      }
      Poll();
    }
    // Do not launch an unstarted retry after the lifecycle deadline. In-flight
    // stop-only workers make this terminal even if their replies arrive later.
    CancelStopRpcRetries();
    if(!StopRpcsQuiescent(&pending))exit_required_=true;
    agx_task_workflow::StepResult result{false,"base/arm/lift stop could not be confirmed within the task bound; base: "+
        base_stop.detail+"; producers_stopped="+(producers_stopped?"true":"false")+
        "; arm_lift_stopped="+(arm_lift_stopped?"true":"false")+
        "; blocked_worker="+(exit_required_?"true":"false")+"; "+pending};
    result.failure=agx_task_workflow::FailureKind::UnconfirmedStop;
    result.state_revision=state_->Snapshot().revision;
    return result;
  }
  bool ContinuationAllowed(const agx_task_workflow::PreparedTask& task,const agx_task_workflow::StepResult&) {
    if(exit_required_ || !StopRpcsQuiescent() || RuntimeFailure()!=agx_task_workflow::FailureKind::None)
      return false;
    const auto current=state_->Snapshot();
    // A task-local certificate cannot override unresolved shared robot state.
    if(current.halted || current.transport.pending ||
        (robot_work_started_ && current.transport.uncertain))return false;
    return static_cast<bool>(task.continuation_check);
  }
  bool RobotContinuationAllowed(const agx_task_workflow::StepResult& result) {
    if(exit_required_ || !StopRpcsQuiescent())return false;
    RefreshCustody();
    // Reobserve the already-established safe clearance at a qualified measured
    // start pose. This never turns an unknown/in-fixture failure into safe travel.
    if(active_profile_) {
      std::lock_guard<std::mutex> lock(travel_mutex_);
      const auto sample=TravelSample();
      const auto start=agx_navigation_tasks::EvaluateStartClearance(*active_profile_,travel_mode_,sample,ros::Time::now().toSec(),Now());
      if(start.ok && sample.robot.transport.eligibility==agx_robot_task_runtime::TravelEligibility::SafeClear)
        RefreshClearance(sample,"measured_qualified_start");
    }
    const auto current=state_->Snapshot();
    if (current.halted || !current.transport.ready || current.transport.pending ||
        current.transport.uncertain || RuntimeFailure()!=agx_task_workflow::FailureKind::None ||
        !BaseReady(true) || !live_observer_ || exit_required_) return false;
    for (const auto& id:result.physical_effect_ids)
      if (std::none_of(current.effects.begin(),current.effects.end(),[&](const agx_robot_task_runtime::EffectRecord& e) {
            return e.effect.id==id && e.status==agx_robot_task_runtime::UpdateStatus::Applied;
          })) return false;
    return true;
  }
  agx_task_workflow::StepResult CleanupTask(double timeout_s) {
    const double deadline=Now()+std::max(0.0,timeout_s-std::min(.2,timeout_s*.1));
    std::string pending;
    while(!exit_required_ && Now()<deadline) {
      if(StopRpcsQuiescent(&pending)) {
        agx_task_workflow::StepResult result{true,"stop workers joined; task effects preserved; no generic gripper release"};
        result.state_revision=state_->Snapshot().revision;return result;
      }
      Poll();
    }
    CancelStopRpcRetries();
    if(!StopRpcsQuiescent(&pending))exit_required_=true;
    agx_task_workflow::StepResult result{false,"stop worker cleanup is terminal; "+pending};
    result.failure=agx_task_workflow::FailureKind::UnconfirmedStop;result.state_revision=state_->Snapshot().revision;
    return result;
  }
  void DispatchStarted(bool shared_actuators) {
    motion_started_=motion_started_ || shared_actuators;
  }
  void Stop(bool terminal=false) {
    if (!motion_started_) return;
    if (goal_sent_) client_.cancelGoal();
    // Independent stop paths: a stalled hold RPC must not delay arm/lift stop.
    std_msgs::String event;event.data="stop";arm_stop_.publish(event);
    std::lock_guard<std::mutex> lock(stop_mutex_);
    ReapStopRpcsLocked();
    if(stop_rpcs_sealed_)return; // Terminal fallback cannot schedule new late RPCs.
    if(terminal)stop_rpcs_sealed_=true;
    if(lift_stop_) {
      if(!lift_stop_job_.worker.joinable())LaunchStopRpcLocked(lift_stop_job_,lift_stop_,true);
      else if(terminal)lift_stop_job_.retry=true;
    }
    if(!hold_stop_job_.worker.joinable())LaunchStopRpcLocked(hold_stop_job_,hold_,false);
    else if(terminal)hold_stop_job_.retry=true;
  }
  void Phase(const std::string& phase) {
    { std::lock_guard<std::mutex> lock(mutex_); phase_=phase; }
    std_msgs::String message; message.data=phase; if (phase_pub_) phase_pub_.publish(message);
    ROS_INFO("workflow phase: %s",phase.c_str());
    Event("phase",true,phase);
  }
  void TaskEvent(const agx_task_workflow::TaskResult& task) {
    if(task.status=="succeeded" || task.status=="failed" || task.status=="cancelled")
      if(auto* slot=FindGraspObservationSlot(task.id))
        slot->facts.fetch_or(JournalTerminal,std::memory_order_release);
    std_msgs::String message;
    message.data="{\"kind\":\"task_result\",\"run_id\":"+Json(run_id_)+
        ",\"task_id\":"+Json(task.id)+",\"invocation_id\":"+Json(run_id_+"/"+task.id)+
        ",\"task\":"+TaskJson(task)+"}";
    if (event_pub_) event_pub_.publish(message);
    ROS_INFO_STREAM("workflow task: " << TaskJson(task));
  }
  void Event(const std::string& kind,bool accepted,const std::string& detail,const std::string& operation={}) {
    std_msgs::String message;
    message.data="{\"kind\":"+Json(kind)+",\"run_id\":"+Json(run_id_)+
        ",\"task_id\":"+Json(active_task_id_)+",\"invocation_id\":"+Json(run_id_+"/"+active_task_id_)+
        ",\"departure_id\":"+Json(departure_id_)+",\"operation_id\":"+Json(operation.empty()?run_id_+"/"+active_task_id_:operation)+
        ",\"accepted\":"+(accepted?"true":"false")+",\"detail\":"+Json(detail)+
        ",\"source_time_s\":"+std::to_string(ros::Time::now().toSec())+
        ",\"target\":"+Json(target_station_)+",\"workbench_id\":"+Json(target_station_)+
        ",\"session_id\":"+Json(departure_session_)+",\"hold_epoch\":"+std::to_string(departure_epoch_)+
        ",\"readiness_sequence\":"+std::to_string(permit_readiness_sequence_)+
        ",\"state_revision\":"+std::to_string(state_->Snapshot().transport.revision)+
        ",\"config_id\":"+Json(active_profile_?active_profile_->config_id:std::string{})+
        ",\"travel_profile_id\":"+Json(active_profile_?active_profile_->profile_id:std::string{})+"}";
    if (event_pub_) event_pub_.publish(message);
  }
  std::string ConfigManifest() const {
    std::ostringstream out;out<<'[';bool first=true;
    for (const auto& item:admitted_configs_) {
      if(!first)out<<',';first=false;
      const auto& c=*item.second;
      out<<"{\"task_id\":"<<Json(item.first)<<",\"config_id\":"<<Json(c.config_id)
         <<",\"fixture_instance_id\":"<<Json(c.fixture_instance_id)<<",\"fixture_profile_id\":"
         <<Json(c.fixture_profile_id)<<",\"sources\":[";
      for (std::size_t i=0;i<c.sources.size();++i) {
        const auto& s=c.sources[i];if(i)out<<',';
        out<<"{\"kind\":"<<Json(s.kind)<<",\"id\":"<<Json(s.id)<<",\"sha256\":"<<Json(s.sha256)
           <<",\"path\":"<<Json(s.absolute_path)<<'}';
      }
      out<<"]}";
    }
    for(const auto& item:admitted_travel_) {
      if(!first)out<<',';first=false;const auto& c=*item.second;
      out<<"{\"task_id\":"<<Json(item.first)<<",\"config_id\":"<<Json(c.config_id)
         <<",\"travel_profile_id\":"<<Json(c.profile_id)
         <<",\"base_envelope_digest\":"<<Json(envelopes_.at(c.config_id)->Digest())
         <<",\"base_envelope_snapshot_yaml\":"<<Json(envelopes_.at(c.config_id)->SnapshotYaml())
         <<",\"sources\":[{\"kind\":\"travel_profile\",\"sha256\":"
         <<Json(c.config_id)<<",\"path\":"<<Json(c.source_path)<<"}]}";
    }
    out<<']';return out.str();
  }
  bool ExitRequired() {
    if(!StopRpcsQuiescent()) {CancelStopRpcRetries();exit_required_=true;}
    return exit_required_;
  }
  const std::string& NavigationDetail() const { return navigation_detail_; }

  agx_task_workflow::StepResult PrepareRoute(bool revalidate) {
    if(!context_ || !active_profile_ || route_invocation_.empty())
      return {false,"27: route admission requires an active task and installed travel profile"};
    if(revalidate && route_.route_id.empty())return {false,"25: no candidate route to revalidate"};
    const auto interrupted=[this]{return RuntimeFailure()!=agx_task_workflow::FailureKind::None;};
    if(interrupted()) {
      agx_task_workflow::StepResult result{false,"route admission interrupted"};result.failure=RuntimeFailure();return result;
    }
    agx_workbench_navigation::PrepareNavigationRoute request;
    request.request.invocation_id=route_invocation_;request.request.target=target_station_;
    request.request.travel_profile_id=active_profile_->profile_id;
    request.request.travel_profile_digest=active_profile_->config_id;
    request.request.for_execution=true;
    if(revalidate)request.request.existing_route_id=route_.route_id;
    auto reply=BoundedService(prepare_route_,request,std::min(7.,context_->Deadline()-Now()),interrupted,route_rpc_active_);
    const auto cause=RuntimeFailure();
    if(cause!=agx_task_workflow::FailureKind::None) {
      route_={};agx_task_workflow::StepResult result{false,"route response arrived after interruption or deadline"};
      result.failure=cause;return result;
    }
    if(!reply) {route_={};return {false,"27: bounded route service unavailable, busy or timed out"};}
    const auto& response=reply->response;
    if(response.status!=response.READY) {
      unsigned code=27;
      switch(response.status) {
        case agx_workbench_navigation::PrepareNavigationRouteResponse::NO_ROUTE:code=23;break;
        case agx_workbench_navigation::PrepareNavigationRouteResponse::SEARCH_LIMIT:code=24;break;
        case agx_workbench_navigation::PrepareNavigationRouteResponse::ROUTE_INVALIDATED:code=25;break;
        case agx_workbench_navigation::PrepareNavigationRouteResponse::START_MISMATCH:code=28;break;
      }
      route_={};navigation_detail_=std::to_string(code)+": "+response.detail;
      return {false,navigation_detail_};
    }
    const auto& candidate=response.route;
    const auto identity=[](const std::string& value){return !value.empty() && value.size()<=128;};
    std::size_t samples=0;for(const auto& segment:candidate.segments)samples+=segment.samples.size();
    if(!identity(candidate.route_id) || candidate.invocation_id!=route_invocation_ ||
       !identity(candidate.model_digest) || !identity(candidate.map_digest) || !identity(candidate.registry_digest) ||
       candidate.header.frame_id!="map" || candidate.header.stamp.isZero() || candidate.segments.empty() ||
       samples==0 || samples>20000 || (revalidate && (candidate.route_id!=route_.route_id ||
       candidate.model_digest!=route_.model_digest || candidate.map_digest!=route_.map_digest ||
       candidate.registry_digest!=route_.registry_digest))) {
      route_={};return {false,"25: route metadata does not match this task's admitted candidate"};
    }
    route_=candidate;
    return {true,revalidate?"route revalidated before base release": "complete route admitted before physical preparation"};
  }

  agx_task_workflow::StepResult ComposedNavigate(const agx_navigation_tasks::NavigateParameters& request,const agx_task_workflow::PhaseSink& phase) {
    if (!request.travel_profile || request.travel_profile!=active_profile_)
      return {false,"navigation has no admitted travel profile"};
    target_station_=request.target;
    departure_id_=run_id_+"/"+active_task_id_+"/departure";
    route_={};route_invocation_=route_session_+std::to_string(++route_sequence_);
    agx_navigation_tasks::NavigationTaskPorts ports;
    ports.runtime_guard=[this]{return RuntimeFailure();};
    ports.plan_route=[this]{return PrepareRoute(false);};
    ports.revalidate_route=[this]{return PrepareRoute(true);};
    ports.invalidate_arrival=[this] {
      std::lock_guard<std::mutex> lock(guard_mutex_);base_guard_->InvalidateArrival();
    };
    ports.establish_departure_hold=[this]{return AcquireHold(false);};
    ports.prepare_travel=[this]{return PrepareTravel();};
    ports.verify_travel_ready=[this]{return VerifyTravel();};
    ports.release_hold=[this]{return ReleaseForTravel();};
    ports.travel=[this,request]{return Navigate(request);};
    ports.verify_stop=[this]{return VerifyStop();};
    ports.acquire_arrival_hold=[this] {EndReadiness();return AcquireHold(true);};
    ports.commit_arrival=[this] {
      std::lock_guard<std::mutex> lock(guard_mutex_);
      const auto sample=BaseSnapshot();
      const bool matches=base_guard_->ArrivalMatches(target_station_,sample,Now(),ros::Time::now().toSec());
      agx_task_workflow::StepResult result{matches,matches?"fresh measured destination arrival hold":"arrival hold no longer matches destination"};
      result.state_revision=state_->Snapshot().revision;
      Event("arrival",matches,result.detail);
      return result;
    };
    auto result=agx_navigation_tasks::RunNavigationTask(ports,phase);
    if(!result.ok) {
      EndReadiness();
      std::lock_guard<std::mutex> lock(travel_mutex_);
      if(!travel_failure_detail_.empty())result.detail+="; "+travel_failure_detail_;
    }
    return result;
  }
  const std::string& RunId() const {return run_id_;}
  agx_robot_task_runtime::RobotTaskPorts RuntimePorts(const std::string& task_id) const {
    return runtime_binding_->Bind(task_id);
  }
  void ExpireRuntimePorts() {runtime_binding_->Expire();}
  // Source-local main entry points, not installed or exported consumer APIs.
  void PrepareGraspObservations(const std::vector<agx_task_workflow::PreparedTask>& tasks) noexcept {
    if(grasp_observations_prepared_)return;
    grasp_observations_prepared_=true;
    if(!grasp_observations_requested_ || !grasp_observations_execute_)return;
    try {
      std::vector<std::unique_ptr<GraspObservationSlot>> slots;
      for(std::size_t index=0;index<tasks.size();++index) {
        if(tasks[index].type!="grasp_test_tube")continue;
        auto slot=std::make_unique<GraspObservationSlot>();
        auto& status=slot->status;status.recipe_index=index;
        // Invalid diagnostic identity changes neither task admission nor control.
        // Such a slot retains only its index and fixed unavailability category.
        const auto& id=tasks[index].id;
        if(run_id_.size()>512 || id.size()>512 || run_id_.size()+id.size()+7>512) {
          slot->unavailable=static_cast<int>(JournalUnavailable::IdentityInvalid);
        } else {
          status.run_id=run_id_;status.task_id=id;status.operation_id=run_id_+"/"+id+"/grasp";
          try { (void)agx_tube_tasks::detail::EncodeGraspJournalStatus(status);slot->identity_valid=true; }
          catch(const std::invalid_argument&) {
            status.run_id.clear();status.task_id.clear();status.operation_id.clear();
            slot->unavailable=static_cast<int>(JournalUnavailable::IdentityInvalid);
          }
        }
        slots.push_back(std::move(slot));
      }
      grasp_observation_slots_=std::move(slots);
      const bool any=std::any_of(grasp_observation_slots_.begin(),grasp_observation_slots_.end(),
          [](const auto& slot) {return slot->identity_valid;});
      if(!any)return;
      const std::weak_ptr<Workflow> weak=shared_from_this();
      grasp_observation_timer_=node_.createWallTimer(ros::WallDuration(.1),
          [weak](const ros::WallTimerEvent&) {
            if(const auto owner=weak.lock())owner->PumpGraspObservations([&](const std::string& text) {
              std_msgs::String message;message.data=text;owner->event_pub_.publish(message);
            });
          },false,false);
      grasp_observations_active_=true;
      grasp_observation_timer_.start();
    } catch(...) {
      grasp_observations_active_=false;grasp_observations_preparation_failed_=true;
      // Stable slots are never cleared after a timer could have observed them.
      for(const auto& slot:grasp_observation_slots_)
        slot->unavailable=static_cast<int>(JournalUnavailable::PreparationFailed);
    }
  }
  void MarkGraspObservationRunReturned() noexcept {grasp_observations_run_returned_=true;}
  std::string GraspObservationSummary() const {
    try {
      const bool requested=grasp_observations_requested_,active=grasp_observations_active_.load(),
          returned=grasp_observations_run_returned_.load(),
          preparation_failed=grasp_observations_preparation_failed_.load(),
          pump_failed=grasp_observations_pump_failed_.load();
      const auto prefix=[&](bool available) {
        return std::string("{\"requested\":")+(requested?"true":"false")+
            ",\"active\":"+(active?"true":"false")+
            ",\"run_returned\":"+(returned?"true":"false")+
            ",\"preparation_failed\":"+(preparation_failed?"true":"false")+
            ",\"pump_failed\":"+(pump_failed?"true":"false")+
            ",\"snapshot_available\":"+(available?"true":"false")+
            ",\"in_flight\":"+(available?"false":"true")+",\"slots\":[";
      };
      std::vector<JournalSummarySlot> copy;
      {
        JournalClaim claim(grasp_observation_claim_);
        if(!claim.held)return prefix(false)+"]}";
        copy.reserve(grasp_observation_slots_.size());
        for(const auto& slot:grasp_observation_slots_) {
          RefreshJournalStatus(*slot,slot->facts.load(std::memory_order_acquire),
              slot->unavailable.load(),returned);
          copy.push_back({slot->status,slot->identity_valid,slot->terminal_status_attempted,
                          slot->terminal_status_publish_returned});
        }
      } // Formatting is outside the claim; no mutable pump state is read below.
      auto text=prefix(true);
      for(std::size_t index=0;index<copy.size();++index) {
        const auto& item=copy[index];if(index)text+=',';
        text+="{\"recipe_index\":"+std::to_string(item.status.recipe_index)+",\"status\":";
        text+=item.identity_valid?agx_tube_tasks::detail::EncodeGraspJournalStatus(item.status):"null";
        text+=",\"unavailable_reason\":";
        text+=item.status.unavailable_reason?Json(JournalUnavailableName(*item.status.unavailable_reason)):"null";
        text+=std::string(",\"terminal_status_attempted\":")+(item.attempted?"true":"false")+
            ",\"terminal_status_publish_returned\":"+(item.returned?"true":"false")+"}";
      }
      return text+"]}";
    } catch(...) {return "null";} // Optional diagnostics never replace the task result.
  }
 private:
  using JournalStatus=agx_tube_tasks::detail::GraspJournalStatus;
  using JournalStage=agx_tube_tasks::detail::GraspJournalStage;
  using JournalUnavailable=agx_tube_tasks::detail::GraspJournalUnavailable;
  enum JournalFact : unsigned {JournalEntered=1,JournalBound=2,JournalReturned=4,
      JournalJoined=8,JournalExitRequired=16,JournalFinished=32,JournalTerminal=64};
  struct GraspObservationSlot {
    JournalStatus status; // Immutable identity plus pump-owned metadata.
    std::shared_ptr<agx_tube_tasks::GraspEffectAdapter> adapter; // Assigned once before Bound release.
    std::atomic<unsigned> facts{0};
    std::atomic<int> unavailable{-1};
    bool identity_valid=false,closed=false,terminal_status_attempted=false,
        terminal_status_publish_returned=false;
    unsigned reported_facts=0;int reported_unavailable=-1;
    bool reported_run_returned=false;
  };
  struct JournalClaim {
    std::atomic_flag& flag;bool held;
    explicit JournalClaim(std::atomic_flag& value):flag(value),held(!value.test_and_set(std::memory_order_acquire)) {}
    ~JournalClaim() {if(held)flag.clear(std::memory_order_release);}
  };
  struct JournalSummarySlot {JournalStatus status;bool identity_valid,attempted,returned;};
  static const char* JournalUnavailableName(JournalUnavailable reason) {
    switch(reason) {
      case JournalUnavailable::PreparationFailed:return "preparation_failed";
      case JournalUnavailable::IdentityInvalid:return "identity_invalid";
      case JournalUnavailable::EnableFailed:return "enable_failed";
      case JournalUnavailable::BindingUnavailable:return "binding_unavailable";
    }
    throw std::invalid_argument("unknown grasp journal unavailability");
  }
  GraspObservationSlot* FindGraspObservationSlot(const std::string& id) const noexcept {
    if(!grasp_observations_active_)return nullptr;
    for(const auto& slot:grasp_observation_slots_)
      if(slot->identity_valid && slot->status.task_id==id)return slot.get();
    return nullptr;
  }
  GraspObservationSlot* BindGraspObservation(GraspObservationSlot* slot,
      const std::shared_ptr<agx_tube_tasks::GraspEffectAdapter>& adapter) noexcept {
    if(!slot)return nullptr;
    if(slot->facts.load(std::memory_order_acquire)&JournalBound) {
      slot->unavailable=static_cast<int>(JournalUnavailable::BindingUnavailable);
      return nullptr; // Never replace or relabel an already exposed producer.
    }
    try {
      if(!adapter->EnableObservationJournal())throw std::runtime_error("journal enable failed");
      slot->adapter=adapter;
      slot->facts.fetch_or(JournalBound,std::memory_order_release);
    } catch(...) {slot->unavailable=static_cast<int>(JournalUnavailable::EnableFailed);}
    return slot;
  }
  void RefreshJournalStatus(GraspObservationSlot& slot,unsigned facts,int unavailable,bool run_returned) const noexcept {
    auto& status=slot.status;
    status.entered=facts&JournalEntered;status.adapter_bound=facts&JournalBound;
    status.monitor_returned=facts&JournalReturned;
    if(status.monitor_returned) {
      status.worker_joined=bool(facts&JournalJoined);status.exit_required=bool(facts&JournalExitRequired);
    }
    status.finish_completed=facts&JournalFinished;status.runner_terminal_seen=facts&JournalTerminal;
    status.unavailable_reason=unavailable<0?std::optional<JournalUnavailable>{}:
        std::optional<JournalUnavailable>{static_cast<JournalUnavailable>(unavailable)};
    if(status.entered && !status.adapter_bound && (status.runner_terminal_seen || run_returned) &&
        !status.unavailable_reason)status.unavailable_reason=JournalUnavailable::BindingUnavailable;
    status.active=grasp_observations_active_ && !status.unavailable_reason;
    status.stage=status.final_acquired?JournalStage::FinalDrain:
        status.unavailable_reason?JournalStage::Unavailable:
        !status.entered && run_returned?JournalStage::NotStarted:JournalStage::Progress;
  }
  static void JournalIncrement(std::uint64_t& value,JournalStatus& status) noexcept {
    if(value==std::numeric_limits<std::uint64_t>::max())status.counter_overflow=true;
    else ++value;
  }
  template<class Publish> void PumpGraspObservations(Publish&& publish) noexcept {
    if(!grasp_observations_active_)return;
    JournalClaim claim(grasp_observation_claim_);if(!claim.held)return;
    try {
      const auto count=grasp_observation_slots_.size();
      for(std::size_t searched=0;searched<count;++searched) {
        const auto index=grasp_observation_cursor_;
        grasp_observation_cursor_=(index+1)%count;
        auto& slot=*grasp_observation_slots_[index];
        if(!slot.identity_valid || slot.closed)continue;
        const auto facts=slot.facts.load(std::memory_order_acquire);
        const bool returned=grasp_observations_run_returned_.load();
        const int unavailable=slot.unavailable.load();
        if(!(facts&JournalBound) && unavailable<0 && !returned)continue;
        bool changed=facts!=slot.reported_facts || unavailable!=slot.reported_unavailable ||
            returned!=slot.reported_run_returned;
        bool terminal=false;
        if(facts&JournalBound) {
          // Snapshot closure BEFORE acquisition. Later completion cannot relabel
          // this normal batch final; it requires a subsequent eligible attempt.
          const unsigned complete=JournalReturned|JournalJoined|JournalFinished|JournalTerminal;
          const bool final=(facts&complete)==complete;
          try {
            const auto adapter=slot.adapter;
            auto batch=adapter->TryTakeObservations();
            if(!batch)return; // Busy/spurious means no acquired metadata, no retry.
            changed=changed || slot.status.last_sequence!=batch->last_sequence ||
                slot.status.capture_gap!=batch->gap || batch->count!=0 || final;
            slot.status.last_sequence=batch->last_sequence;slot.status.capture_gap=batch->gap;
            if(final) {slot.closed=true;slot.status.final_acquired=true;terminal=true;}
            for(std::size_t row=0;row<batch->count;++row) {
              const auto& observation=batch->records[row];
              try {
                const auto text=agx_tube_tasks::detail::EncodeGraspEffectObservation(observation);publish(text);
                JournalIncrement(slot.status.rows_publish_returned,slot.status);
              } catch(...) {
                slot.status.transport_gap=true;JournalIncrement(slot.status.failed_rows,slot.status);
                if(!slot.status.first_failed_sequence)slot.status.first_failed_sequence=observation.observation_sequence;
                slot.status.last_failed_sequence=observation.observation_sequence;
              }
            }
          } catch(...) {
            changed=true;slot.status.transport_gap=true;JournalIncrement(slot.status.drain_error_count,slot.status);
          }
        } else if(returned) {
          // A never-started/unavailable slot has no closed producer acquisition.
          terminal=true;slot.closed=true;changed=true;
        }
        const auto current_facts=slot.facts.load(std::memory_order_acquire);
        const auto current_unavailable=slot.unavailable.load();
        const bool current_returned=grasp_observations_run_returned_.load();
        RefreshJournalStatus(slot,current_facts,current_unavailable,current_returned);
        changed=changed || current_facts!=slot.reported_facts ||
            current_unavailable!=slot.reported_unavailable || current_returned!=slot.reported_run_returned;
        if(changed) {
          slot.reported_facts=current_facts;slot.reported_unavailable=current_unavailable;
          slot.reported_run_returned=current_returned;
          if(terminal)slot.terminal_status_attempted=true;
          try {
            const auto text=agx_tube_tasks::detail::EncodeGraspJournalStatus(slot.status);publish(text);
            if(terminal)slot.terminal_status_publish_returned=true;
          } catch(...) {
            slot.status.transport_gap=true;JournalIncrement(slot.status.prior_status_error_count,slot.status);
          }
        }
        return; // At most one selected slot and one batch per callback.
      }
    } catch(...) {
      // Claim release is unconditional. No optional diagnostic reaches control.
      grasp_observations_pump_failed_=true;
    }
  }
  bool grasp_observations_requested_=false,grasp_observations_execute_=false,
      grasp_observations_prepared_=false;
  std::atomic<bool> grasp_observations_active_{false},grasp_observations_run_returned_{false},
      grasp_observations_preparation_failed_{false},grasp_observations_pump_failed_{false};
  mutable std::atomic_flag grasp_observation_claim_=ATOMIC_FLAG_INIT;
  std::vector<std::unique_ptr<GraspObservationSlot>> grasp_observation_slots_;
  GraspObservationSlot* active_grasp_observation_=nullptr;
  std::size_t grasp_observation_cursor_=0;
  ros::WallTimer grasp_observation_timer_;

  static agx_robot_task_runtime::Evidence Evidence(const agxcore::TubeEvidence& e) {
    return {e.source,e.source_stamp_s,e.receipt_steady_s};
  }
  static bool TerminalStatus(const actionlib_msgs::GoalStatusArray& message) {
    for(const auto& item:message.status_list)
      if(item.status!=2 && item.status!=3 && item.status!=4 && item.status!=5 && item.status!=8 && item.status!=9)return false;
    return true;
  }
  bool FreshSourceReceipt(double source,double receipt,double max_source_age,double max_receipt_age) const {
    return agx_robot_task_runtime::FreshStatusEvidence(source,receipt,ros::Time::now().toSec(),Now(),
        max_source_age,max_receipt_age,future_skew_);
  }
  bool ProducersStoppedAfter(const agxcore::TubeEvidence& barrier) {
    if(!StopRpcsQuiescent())return false;
    std::lock_guard<std::mutex> lock(mutex_);
    return !motion_worker_active_ && !any_active_ &&
        FreshSourceReceipt(status_stamp_,status_at_,1.0,1.0) && status_stamp_>barrier.source_stamp_s &&
        FreshSourceReceipt(arm_execution_status_.header.stamp.toSec(),arm_execution_at_,1.0,1.0) &&
        FreshSourceReceipt(arm_planning_status_.header.stamp.toSec(),arm_planning_at_,1.0,1.0) &&
        status_at_>barrier.receipt_steady_s &&
        Now()-status_at_<=1.0 && arm_execution_at_>barrier.receipt_steady_s &&
        arm_planning_at_>barrier.receipt_steady_s && Now()-arm_execution_at_<=1.0 && Now()-arm_planning_at_<=1.0 &&
        arm_execution_status_.header.stamp.toSec()>barrier.source_stamp_s &&
        arm_planning_status_.header.stamp.toSec()>barrier.source_stamp_s &&
        TerminalStatus(arm_execution_status_) && TerminalStatus(arm_planning_status_);
  }
  void RefreshClearance(const agx_navigation_tasks::TravelSample& sample,const std::string& source) {
    const double stamp=std::min(sample.arm.evidence.source_stamp_s,sample.lift.evidence.source_stamp_s);
    const double receipt=std::min(sample.arm.evidence.receipt_steady_s,sample.lift.evidence.receipt_steady_s);
    const auto prior=sample.robot.transport.clearance_evidence;
    if(stamp<=last_clearance_stamp_ || (prior &&
        (stamp<=prior->source_stamp_s || receipt<prior->receipt_steady_s)))return;
    agx_robot_task_runtime::PhysicalEffect effect;effect.id=run_id_+"/clearance/"+std::to_string(++observation_sequence_);
    effect.kind=agx_robot_task_runtime::EffectKind::TravelEligibility;effect.eligibility=agx_robot_task_runtime::TravelEligibility::SafeClear;
    effect.object=sample.robot.transport.object;
    const std::string operation=departure_id_+"/endpoint";
    const auto update=state_->ApplyEffect(effect,{source,stamp,
        std::min(sample.arm.evidence.receipt_steady_s,sample.lift.evidence.receipt_steady_s),run_id_,active_task_id_,operation},
        operation,sample.robot.revision);
    last_clearance_stamp_=stamp;if(update.halt)runtime_fault_=true;
  }
  void ArmStatus(const actionlib_msgs::GoalStatusArray& message,bool execution) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto& order=execution?arm_execution_order_:arm_planning_order_;
    auto& current=execution?arm_execution_status_:arm_planning_status_;
    auto& original_receipt=execution?arm_execution_at_:arm_planning_at_;
    const double source=message.header.stamp.toSec(),receipt=Now();
    if(!std::isfinite(source) || source<=0 || !std::isfinite(receipt) || receipt<=0 ||
        source<order.source || receipt<order.receipt) {
      order.travel_valid=false;
      // An old terminal report must not replace newer active evidence. Bad
      // active input can only deny stopping, never manufacture a terminal state.
      if(!TerminalStatus(message)) {current=message;original_receipt=0;}
      return;
    }
    // Actionlib can change state within one /clock tick. Keep that legitimate
    // transition, without pretending its same-stamp evidence was newly sampled.
    current=message;
    if(source>order.source)original_receipt=receipt;
    order.source=source;order.receipt=receipt;
  }
  agx_navigation_tasks::TravelSample TravelSample() {
    RefreshCustody();
    agx_navigation_tasks::TravelSample sample;sample.robot=state_->Snapshot();
    if(!live_observer_ || !active_profile_)return sample;
    const auto observed=live_observer_->Snapshot();
    sample.arm.evidence=Evidence(observed.arm.evidence);
    sample.arm.valid=observed.arm.valid && observed.arm.fresh;
    sample.arm.stationary=observed.arm.stationary;
    sample.arm.stationary_source_since_s=observed.arm.stationary_source_since_s;
    sample.arm.stationary_receipt_since_s=observed.arm.stationary_receipt_since_s;
    for(std::size_t i=0;i<6;++i) {
      sample.arm.positions_rad["joint"+std::to_string(i+1)]=observed.arm.positions[i];
      sample.arm.velocities_rad_s["joint"+std::to_string(i+1)]=observed.arm.velocities[i];
    }
    sample.lift.evidence=Evidence(observed.lift.evidence);
    sample.lift.valid=observed.lift.valid && observed.lift.fresh;
    sample.lift.position_m=observed.lift.position_m;
    sample.lift.velocity_mps=observed.lift.velocity_mps;
    sample.lift.stationary=observed.lift.stationary;
    sample.lift.stationary_source_since_s=observed.lift.stationary_source_since_s;
    sample.lift.stationary_receipt_since_s=observed.lift.stationary_receipt_since_s;
    sample.motion_window=observed.motion_window;
    // A false active read precedes the exit read: terminal outcome is published
    // before active is cleared. A detached worker permanently requires exit.
    const bool worker_active=motion_worker_active_.load();
    const bool worker_exit=exit_required_.load();
    const auto stop_jobs=StopJobsSnapshot();
    const bool joined=!worker_active && !worker_exit && !stop_jobs.lift_pending &&
        !stop_jobs.hold_pending && !stop_jobs.sealed;
    const auto base=BaseSnapshot();
    {
      std::lock_guard<std::mutex> lock(guard_mutex_);
      sample.hold.held_stopped=base_guard_ && base_guard_->CheckHeld(base,Now(),ros::Time::now().toSec()).ok;
    }
    sample.hold.command_guard_ok=sample.hold.held_stopped && !base.any_goal_active &&
        base.goal_status_received && FreshSourceReceipt(base.goal_status_stamp_s,base.goal_status_receipt_s,1.0,1.0);
    sample.hold.session_id=base.session_id;sample.hold.hold_epoch=base.hold_epoch;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sample.hold.evidence={"measured_base_hold",std::min(odom_.header.stamp.toSec(),authority_.header.stamp.toSec()),
          std::min({odom_at_,authority_at_,command_at_})};
      sample.producers.evidence={"arm_lift_producers",std::min({arm_execution_status_.header.stamp.toSec(),
          arm_planning_status_.header.stamp.toSec(),observed.arm.evidence.source_stamp_s,observed.lift.evidence.source_stamp_s}),
          std::min({arm_execution_at_,arm_planning_at_,observed.arm.evidence.receipt_steady_s,observed.lift.evidence.receipt_steady_s})};
      sample.producers.valid=arm_execution_order_.travel_valid && arm_planning_order_.travel_valid &&
          FreshSourceReceipt(arm_execution_status_.header.stamp.toSec(),arm_execution_at_,
          active_profile_->source_max_age_s,active_profile_->receipt_max_age_s) &&
          FreshSourceReceipt(arm_planning_status_.header.stamp.toSec(),arm_planning_at_,
          active_profile_->source_max_age_s,active_profile_->receipt_max_age_s) && observed.arm.fresh && observed.lift.fresh;
      sample.producers.arm_stopped=joined && TerminalStatus(arm_execution_status_) &&
          TerminalStatus(arm_planning_status_);
      sample.producers.lift_stopped=joined;
    }
    // A fresh endpoint observation can refresh clearance only after a qualified
    // start/path was admitted; it cannot clear unknown custody or an in-fixture grasp.
    if(qualified_path_ && !sample.robot.transport.pending && !sample.robot.transport.uncertain &&
        sample.robot.transport.eligibility!=agx_robot_task_runtime::TravelEligibility::CapturedInFixture &&
        sample.robot.transport.eligibility!=agx_robot_task_runtime::TravelEligibility::Unsupported &&
        sample.arm.valid && sample.lift.valid) {
      bool at_default=std::abs(sample.lift.position_m-active_profile_->lift_position_m)<=active_profile_->lift_tolerance_m;
      for(const auto& j:active_profile_->arm)
        at_default=at_default && sample.arm.positions_rad.count(j.name) &&
            std::abs(sample.arm.positions_rad.at(j.name)-j.position_rad)<=j.tolerance_rad &&
            std::abs(sample.arm.velocities_rad_s.at(j.name))<=j.stopped_velocity_rad_s;
      const double stamp=std::min(sample.arm.evidence.source_stamp_s,sample.lift.evidence.source_stamp_s);
      if(at_default && std::abs(sample.lift.velocity_mps)<=active_profile_->lift_stopped_velocity_mps && stamp>last_clearance_stamp_) {
        RefreshClearance(sample,"measured_travel_endpoint");
        sample.robot=state_->Snapshot();
      }
    }
    return sample;
  }
  agx_task_workflow::StepResult AcquireHold(bool arrival) {
    const std::string operation=run_id_+"/"+active_task_id_+(arrival?"/arrival":"/departure_hold");
    const double deadline=Now()+10;
    while(RuntimeFailure()==agx_task_workflow::FailureKind::None && Now()<deadline) {
      agx_workbench_navigation::AcquireBaseHold request;
      request.request.run_id=run_id_;request.request.operation_id=operation;
      request.request.reason=arrival?"workflow arrival":"workflow travel preparation";
      const auto response=BoundedService(acquire_hold_,request,2.0);
      if(!response)return {false,"typed hold service timed out"};
      if(!response->response.accepted)return {false,response->response.reason};
      const auto sample=BaseSnapshot();
      if(response->response.stopped_confirmed && sample.session_id==response->response.session_id &&
          sample.hold_epoch==response->response.hold_epoch && !sample.any_goal_active &&
          sample.goal_status_received && FreshSourceReceipt(sample.goal_status_stamp_s,sample.goal_status_receipt_s,1.0,1.0)) {
        std::lock_guard<std::mutex> lock(guard_mutex_);
        const auto captured=base_guard_->CaptureHold(sample,Now(),ros::Time::now().toSec(),arrival?target_station_:std::string{});
        if(captured.ok) {departure_session_=sample.session_id;departure_epoch_=sample.hold_epoch;
          Event(arrival?"arrival_hold":"departure_hold",true,captured.detail,operation);return {true,captured.detail};}
      }
      Poll();
    }
    return {false,"fresh typed held/stopped base could not be established"};
  }
  agx_task_workflow::StepResult PrepareTravel() {
    const auto owner=shared_from_this();agx_navigation_tasks::StartClearance start;agx_robot_task_runtime::TransportView load;
    const double observation_deadline=Now()+5.0;
    while(RuntimeFailure()==agx_task_workflow::FailureKind::None && Now()<observation_deadline) {
      std::lock_guard<std::mutex> lock(travel_mutex_);
      const auto sample=TravelSample();
      start=agx_navigation_tasks::EvaluateStartClearance(*active_profile_,travel_mode_,sample,ros::Time::now().toSec(),Now());
      if(start.ok && sample.arm.stationary && sample.lift.stationary &&
          sample.hold.held_stopped && sample.hold.command_guard_ok && sample.producers.valid &&
          sample.producers.arm_stopped && sample.producers.lift_stopped) {
        travel_policy_=std::make_unique<agx_navigation_tasks::TravelPolicy>(*active_profile_,travel_mode_);
        start=travel_policy_->BeginPreparation(sample,ros::Time::now().toSec(),Now());
        if(!start.ok)return {false,start.detail};
        qualified_path_=true;load=sample.robot.transport;break;
      }
      Poll();
    }
    if(!travel_policy_)return {false,"travel start was not observed: "+start.detail};
    motion_worker_active_=true;
    struct PreparationFailure {
      std::string detail;
      double source=0,steady=0,arm_stamp=0,arm_receipt=0,lift_stamp=0,lift_receipt=0;
      double lift_velocity=0,producer_stamp=0,producer_receipt=0;
      bool arm_valid=false,lift_valid=false,producer_valid=false;
    };
    auto preparation_failure=std::make_shared<PreparationFailure>();
    auto result=agx_navigation_tasks::MoveToTravelPosture(node_,*active_profile_,start,load,live_observer_,
      [owner,preparation_failure] {
        const auto failure=owner->RuntimeFailure();if(failure!=agx_task_workflow::FailureKind::None)return failure;
        std::lock_guard<std::mutex> lock(owner->travel_mutex_);
        // Snapshot first: C++ does not order argument evaluation, so an inline
        // snapshot can contain evidence newer than the supplied "now" clocks.
        const auto sample=owner->TravelSample();
        const double source=ros::Time::now().toSec(),steady=Now();
        const auto d=owner->travel_policy_->CheckPreparation(sample,source,steady);
        if(d.failure!=agx_task_workflow::FailureKind::None) {
          owner->runtime_fault_=true;
          // Retain decision-time evidence without logging on the stop path.
          // Result reporting occurs after the runner's physical stop attempt.
          *preparation_failure={d.detail,source,steady,
              sample.arm.evidence.source_stamp_s,sample.arm.evidence.receipt_steady_s,
              sample.lift.evidence.source_stamp_s,sample.lift.evidence.receipt_steady_s,
              sample.lift.velocity_mps,sample.producers.evidence.source_stamp_s,
              sample.producers.evidence.receipt_steady_s,
              sample.arm.valid,sample.lift.valid,sample.producers.valid};
        }
        return d.failure;
      },[owner]{owner->Stop();});
    if(result.exit_required)exit_required_=true;
    motion_worker_active_=false;
    if(!result.step.ok) {
      if(!preparation_failure->detail.empty()) {
        const auto& d=*preparation_failure;
        std::ostringstream detail;
        detail<<"; "<<d.detail<<std::setprecision(17)
            <<"; source_now="<<d.source<<" steady_now="<<d.steady
            <<" arm_valid="<<d.arm_valid<<" arm_stamp="<<d.arm_stamp<<" arm_receipt="<<d.arm_receipt
            <<" lift_valid="<<d.lift_valid<<" lift_stamp="<<d.lift_stamp<<" lift_receipt="<<d.lift_receipt
            <<" lift_velocity="<<d.lift_velocity<<" producer_valid="<<d.producer_valid
            <<" producer_stamp="<<d.producer_stamp<<" producer_receipt="<<d.producer_receipt;
        result.step.detail+=detail.str();
      }
      return result.step;
    }
    std::lock_guard<std::mutex> lock(travel_mutex_);
    const auto final_sample=TravelSample();
    const auto final_preparation=travel_policy_->CheckPreparation(final_sample,ros::Time::now().toSec(),Now());
    if(final_preparation.failure!=agx_task_workflow::FailureKind::None) {
      agx_task_workflow::StepResult r{false,final_preparation.detail};r.failure=final_preparation.failure;return r;
    }
    const auto window=live_observer_->ArmMotionWindow();
    const auto barrier=travel_policy_->CommandsFinished(window,ros::Time::now().toSec(),Now());
    if(barrier.failure!=agx_task_workflow::FailureKind::None) {agx_task_workflow::StepResult r{false,barrier.detail};r.failure=barrier.failure;return r;}
    return result.step;
  }
  agx_task_workflow::StepResult VerifyTravel() {
    while(RuntimeFailure()==agx_task_workflow::FailureKind::None) {
      agx_navigation_tasks::TravelDecision decision;
      {
        std::lock_guard<std::mutex> lock(travel_mutex_);
        const auto sample=TravelSample();
        decision=travel_policy_->Observe(sample,ros::Time::now().toSec(),Now());
        if(decision.ready) {readiness_active_=true;last_readiness_stamp_=0;readiness_sequence_=0;}
      }
      if(decision.failure!=agx_task_workflow::FailureKind::None) {agx_task_workflow::StepResult r{false,decision.detail};r.failure=decision.failure;return r;}
      if(decision.ready) {
        const auto owner=shared_from_this();
        readiness_timer_=node_.createWallTimer(ros::WallDuration(.025),[owner](const ros::WallTimerEvent&){owner->PublishReadiness();});
        PublishReadiness();
        return {true,decision.detail,{},decision.state_revision};
      }
      Poll();
    }
    return {false,"travel verification interrupted"};
  }
  void PublishReadiness() {
    std::lock_guard<std::mutex> lock(travel_mutex_);
    if(!readiness_active_ || !travel_policy_)return;
    const auto sample=TravelSample();const auto stamp=ros::Time::now();
    auto decision=travel_policy_->CheckTravel(sample,stamp.toSec(),Now());
    // An observed physical failure is a fault input at this check. Withdrawing
    // healthy readiness for an existing interruption must not invent a fault.
    if(!decision.ready)runtime_fault_=true;
    if(RuntimeFailure()!=agx_task_workflow::FailureKind::None && decision.failure==agx_task_workflow::FailureKind::None) {
      decision.ready=false;decision.detail="workflow interrupted";
    }
    if(!decision.ready) {
      if(travel_failure_detail_.empty())travel_failure_detail_=decision.detail;
    }
    if(stamp.toSec()<=last_readiness_stamp_)return; // Never restamp unchanged source time.
    agx_workbench_navigation::TravelReadiness message;message.header.stamp=stamp;
    message.run_id=run_id_;message.departure_id=departure_id_;message.session_id=departure_session_;
    message.hold_epoch=departure_epoch_;message.sequence=++readiness_sequence_;
    message.state_revision=decision.state_revision;message.config_id=active_profile_->config_id;
    message.travel_profile_id=active_profile_->profile_id;
    message.arm_ready=decision.arm_ready;message.lift_ready=decision.lift_ready;
    message.producers_stopped=decision.producers_stopped;message.clearance_verified=decision.clearance_verified;
    message.ready=decision.ready;message.reason=decision.detail;
    message.payload_state=sample.robot.transport.load==agx_robot_task_runtime::TransportLoad::Empty?message.EMPTY:
        sample.robot.transport.load==agx_robot_task_runtime::TransportLoad::SingleHeld?message.HOLDING:message.UNKNOWN;
    if(sample.robot.transport.object) {message.payload_ref=sample.robot.transport.object->id;
      message.container_profile_id=sample.robot.transport.object->container_profile;}
    readiness_pub_.publish(message);last_readiness_stamp_=stamp.toSec();last_readiness_=message;
  }
  void EndReadiness() {
    readiness_timer_.stop();std::lock_guard<std::mutex> lock(travel_mutex_);readiness_active_=false;
  }
  agx_task_workflow::StepResult ReleaseForTravel() {
    // Wait only for acknowledgement propagation. Every retry keeps the same
    // departure identity and the arbiter never extends a previously issued permit.
    std::optional<agx_workbench_navigation::PrepareBaseDeparture> permit;
    const double deadline=Now()+2.0;
    while(RuntimeFailure()==agx_task_workflow::FailureKind::None && Now()<deadline) {
      agx_workbench_navigation::PrepareBaseDeparture request;
      {
        std::lock_guard<std::mutex> lock(travel_mutex_);
        request.request.run_id=run_id_;request.request.departure_id=departure_id_;
        request.request.expected_session_id=departure_session_;request.request.expected_hold_epoch=departure_epoch_;
        request.request.readiness_sequence=last_readiness_.sequence;request.request.state_revision=last_readiness_.state_revision;
      }
      permit=BoundedService(prepare_departure_,request,3.0);
      if(!permit)return {false,"departure permit RPC timed out; no release dispatched"};
      if(permit->response.accepted) {permit_readiness_sequence_=request.request.readiness_sequence;break;}
      Poll();
    }
    if(!permit || !permit->response.accepted)return {false,permit?permit->response.reason:"departure permit interrupted"};
    if(RuntimeFailure()!=agx_task_workflow::FailureKind::None)return {false,"interrupted before hold release"};
    agx_workbench_navigation::ReleaseBaseHold request;
    request.request.run_id=run_id_;request.request.departure_id=departure_id_;
    request.request.expected_session_id=departure_session_;request.request.expected_hold_epoch=departure_epoch_;
    request.request.permit_id=permit->response.permit_id;
    // Longer than the arbiter's hard two-second maximum permit lifetime: a late
    // release cannot become valid after this caller reports a timeout.
    const auto released=BoundedService(release_hold_,request,3.0);
    const bool accepted=released && released->response.accepted;
    Event("release",accepted,released?released->response.reason:"release RPC timed out",departure_id_);
    if(accepted) {
      std::lock_guard<std::mutex> lock(travel_mutex_);
      const auto sample=TravelSample();
      if(RuntimeFailure()!=agx_task_workflow::FailureKind::None) return {false,"interrupted after accepted hold release"};
      const auto decision=travel_policy_->BeginTravel(sample,ros::Time::now().toSec(),Now());
      if(!decision.ready) {
        runtime_fault_=true;
        if(travel_failure_detail_.empty())travel_failure_detail_=decision.detail;
        agx_task_workflow::StepResult result{false,decision.detail};result.failure=decision.failure;return result;
      }
    }
    return {accepted,released?released->response.reason:"release RPC timed out"};
  }
  void RefreshCustody() {
    if (!live_observer_) return;
    const auto observed=live_observer_->Snapshot().gripper;
    if (!observed.valid || !observed.fresh || observed.evidence.source_stamp_s<=last_custody_stamp_)
      return;
    const auto before=state_->Snapshot();
    if (before.halted || before.transport.pending) return;
    if(before.gripper.evidence && (observed.evidence.source_stamp_s<=before.gripper.evidence->source_stamp_s ||
        observed.evidence.receipt_steady_s<before.gripper.evidence->receipt_steady_s))return;
    agx_robot_task_runtime::PhysicalEffect effect;
    effect.id=run_id_+"/observation/"+std::to_string(++observation_sequence_);
    if (observed.known_empty) effect.kind=agx_robot_task_runtime::EffectKind::GripperEmpty;
    else if (observed.caught && before.gripper.occupancy==agx_robot_task_runtime::GripperOccupancy::Holding && before.gripper.object) {
      effect.kind=agx_robot_task_runtime::EffectKind::GripperHolding;effect.object=before.gripper.object;
    } else if (before.gripper.occupancy==agx_robot_task_runtime::GripperOccupancy::Holding) {
      effect.kind=agx_robot_task_runtime::EffectKind::GripperUncertain;effect.object=before.gripper.object;
    } else return; // A caught sensor cannot identify an unknown object after restart.
    const auto operation=run_id_+"/live-observer";
    const auto& evidence=observed.evidence;
    const auto update=state_->ApplyEffect(effect,{evidence.source,evidence.source_stamp_s,
        evidence.receipt_steady_s,run_id_,active_task_id_.empty()?"admission":active_task_id_,operation},
        operation,before.revision);
    last_custody_stamp_=evidence.source_stamp_s;
    if (update.halt) runtime_fault_=true;
  }
  bool NavigationAvailable() {
    std::lock_guard<std::mutex> lock(mutex_);
    return authority_.state!=Authority::FAULT && client_.isServerConnected() && Now()-authority_at_<=receipt_limit_ && watchdog_ready_ && Now()-watchdog_seen_.load()<.5;
  }
  agx_task_workflow::StepResult CallHold() {
    auto promise=std::make_shared<std::promise<agx_task_workflow::StepResult>>(); auto future=promise->get_future();
    auto client=hold_;
    std::thread([client,promise]() mutable {
      std_srvs::Trigger request; const bool received=client.call(request);
      promise->set_value({received && request.response.success,request.response.message});
    }).detach();
    if (future.wait_for(std::chrono::seconds(2))!=std::future_status::ready)
      return {false,"hold service wall timeout"};
    return future.get();
  }
  agx_robot_task_runtime::BaseSample BaseSnapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    agx_robot_task_runtime::BaseSample sample;
    sample.authority_received=authority_at_>0;
    sample.odometry_received=odom_at_>0;
    sample.command_received=command_at_>0;
    sample.goal_status_received=status_at_>0;
    sample.authority_receipt_s=authority_at_;
    sample.odometry_receipt_s=odom_at_;
    sample.command_receipt_s=command_at_;
    sample.goal_status_receipt_s=status_at_;
    sample.watchdog_receipt_s=watchdog_seen_.load();
    sample.watchdog_received=sample.watchdog_receipt_s>0;
    sample.watchdog_ready=watchdog_ready_;
    sample.odometry_stamp_s=odom_.header.stamp.toSec();
    sample.authority_stamp_s=authority_.header.stamp.toSec();sample.goal_status_stamp_s=status_stamp_;
    sample.authority_none=authority_.state==Authority::NONE;
    sample.authority_fault=authority_.state==Authority::FAULT;
    sample.manipulation_hold=authority_.manipulation_hold_active;
    sample.command_zero=command_zero_;
    sample.owned_goal_terminal=owned_terminal_;
    sample.any_goal_active=any_active_;
    sample.owned_goal_id=owned_goal_;
    sample.session_id=authority_.session_id;
    sample.hold_epoch=authority_.hold_epoch;
    sample.x_m=odom_.pose.pose.position.x;
    sample.y_m=odom_.pose.pose.position.y;
    sample.yaw_rad=Yaw(odom_.pose.pose.orientation);
    sample.linear_x_mps=odom_.twist.twist.linear.x;
    sample.linear_y_mps=odom_.twist.twist.linear.y;
    sample.angular_z_radps=odom_.twist.twist.angular.z;
    return sample;
  }
  bool BaseReady(bool held, bool owned_goal=false) {
    std::lock_guard<std::mutex> lock(guard_mutex_);
    const auto sample=BaseSnapshot();
    return base_guard_ && base_guard_->CheckStopped(
        sample,Now(),ros::Time::now().toSec(),held,owned_goal).ok;
  }
  static double Yaw(const geometry_msgs::Quaternion& q) {
    const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
    if (!std::isfinite(norm) || std::abs(norm-1.0)>1e-3)
      return std::numeric_limits<double>::quiet_NaN();
    return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  }
  bool GraspMayContinue() {
    std::lock_guard<std::mutex> lock(guard_mutex_);
    if (RuntimeFailure()!=agx_task_workflow::FailureKind::None) {grasp_stop_reason_="cancel, fault or deadline requested";return false;}
    if (!base_guard_) {grasp_stop_reason_="base guard is not configured";return false;}
    const auto sample=BaseSnapshot();
    const auto result=base_guard_->CheckHeld(sample,Now(),ros::Time::now().toSec());
    if (!result.ok) {grasp_stop_reason_=result.detail;runtime_fault_=true;}
    return result.ok;
  }
  void AuthorityState(const Authority::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_); authority_=*m; authority_at_=Now();
    if(m->state==Authority::FAULT)runtime_fault_=true;
  }
  void Odom(const nav_msgs::Odometry::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_); odom_=*m; odom_at_=Now();
  }
  void Command(const geometry_msgs::Twist::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_); command_at_=Now();
    command_zero_=true;
    for (double value : {m->linear.x,m->linear.y,m->linear.z,m->angular.x,m->angular.y,m->angular.z})
      if (!std::isfinite(value) || std::abs(value)>zero_) command_zero_=false;
  }
  void Goal(const ros::MessageEvent<move_base_msgs::MoveBaseActionGoal const>& event) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (navigation_started_ && event.getPublisherName()=="/workbench_mission") {
      owned_goal_=event.getMessage()->goal_id.id; owned_terminal_=false;
    }
  }
  void Status(const actionlib_msgs::GoalStatusArray::ConstPtr& m) {
    std::lock_guard<std::mutex> lock(mutex_); any_active_=false; status_at_=Now();status_stamp_=m->header.stamp.toSec();
    for (const auto& s : m->status_list) {
      bool terminal=s.status==2 || s.status==3 || s.status==4 || s.status==5 || s.status==8 || s.status==9;
      if (!terminal) any_active_=true;
      if (s.goal_id.id==owned_goal_ && terminal) owned_terminal_=true;
    }
  }
  struct StopRpcJob {
    std::thread worker;
    std::shared_ptr<std::atomic<bool>> active=std::make_shared<std::atomic<bool>>(false);
    bool retry=false;
  };
  void LaunchStopRpcLocked(StopRpcJob& job,ros::ServiceClient client,bool lift) {
    const auto active=job.active;active->store(true);
    try {
      job.worker=std::thread([active,client,lift]() mutable {
        try {
          if(lift) {bt_task_msgs::LiftMotorSrv request;request.request.mode=-2;request.request.val=0;client.call(request);}
          else {std_srvs::Trigger request;client.call(request);}
        } catch(...) {}
        active->store(false); // Only the completion flag survives owner destruction.
      });
    } catch(...) {active->store(false);throw;}
  }
  void ReapStopRpcsLocked() {
    for(auto* job:{&lift_stop_job_,&hold_stop_job_})
      if(job->worker.joinable() && !job->active->load())job->worker.join();
    if(lift_stop_job_.retry && !lift_stop_job_.worker.joinable()) {
      lift_stop_job_.retry=false;LaunchStopRpcLocked(lift_stop_job_,lift_stop_,true);
    }
    if(hold_stop_job_.retry && !hold_stop_job_.worker.joinable()) {
      hold_stop_job_.retry=false;LaunchStopRpcLocked(hold_stop_job_,hold_,false);
    }
  }
  bool StopRpcsQuiescent(std::string* detail=nullptr) {
    std::lock_guard<std::mutex> lock(stop_mutex_);ReapStopRpcsLocked();
    const bool lift=lift_stop_job_.worker.joinable() || lift_stop_job_.retry;
    const bool hold=hold_stop_job_.worker.joinable() || hold_stop_job_.retry;
    if(detail)*detail=std::string("lift_stop_rpc_pending=")+(lift?"true":"false")+
        "; hold_stop_rpc_pending="+(hold?"true":"false");
    return !lift && !hold;
  }
  agx_navigation_tasks::TravelStopJobs StopJobsSnapshot() {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    // Readiness must not join workers or launch coalesced/final stop requests.
    return {lift_stop_job_.active->load() || lift_stop_job_.worker.joinable() || lift_stop_job_.retry,
        hold_stop_job_.active->load() || hold_stop_job_.worker.joinable() || hold_stop_job_.retry,
        stop_rpcs_sealed_};
  }
  void CancelStopRpcRetries() {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    stop_rpcs_sealed_=true;lift_stop_job_.retry=hold_stop_job_.retry=false;
  }
  ros::NodeHandle node_;
  Client client_;
  ros::Subscriber authority_sub_,odom_sub_,command_sub_,goal_sub_,status_sub_,stop_sub_,watchdog_sub_;
  ros::Publisher phase_pub_,event_pub_,arm_stop_; ros::WallTimer heartbeat_; ros::ServiceClient hold_,lift_stop_;
  ros::Subscriber arm_execution_sub_,arm_planning_sub_;
  ros::Publisher readiness_pub_;ros::WallTimer readiness_timer_;
  ros::ServiceClient acquire_hold_,prepare_departure_,release_hold_,prepare_route_;
  agx_workbench_navigation::NavigationRoute route_;
  std::string route_session_,route_invocation_;std::uint64_t route_sequence_=0;
  std::shared_ptr<std::atomic<bool>> route_rpc_active_=std::make_shared<std::atomic<bool>>(false);
  actionlib_msgs::GoalStatusArray arm_execution_status_,arm_planning_status_;
  struct ArmStatusOrder {double source=0,receipt=0;bool travel_valid=true;};
  ArmStatusOrder arm_execution_order_,arm_planning_order_;
  double arm_execution_at_=0,arm_planning_at_=0;
  std::mutex travel_mutex_,guard_mutex_;
  std::atomic<bool> motion_worker_active_{false},exit_required_{false};
  std::mutex stop_mutex_;StopRpcJob lift_stop_job_,hold_stop_job_;bool stop_rpcs_sealed_=false;
  agx_navigation_tasks::TravelMode travel_mode_=agx_navigation_tasks::TravelMode::Hardware;
  std::map<std::string,std::shared_ptr<const agx_navigation_tasks::TravelProfile>> admitted_travel_;
  bool robot_work_started_=false;
  bool onboard_rack_installed_=false;
  std::shared_ptr<const agx_place_tasks::OnboardRackProfile> onboard_rack_profile_;
  std::shared_ptr<const agx_place_tasks::PositionTable> measured_positions_;
  agx_place_tasks::PlaceMotionConfig place_motion_;
  std::shared_ptr<const agx_navigation_tasks::TravelProfile> active_profile_;
  std::shared_ptr<agx_navigation_tasks::TravelEnvelopeAdmission> active_envelope_;
  std::map<std::string,std::shared_ptr<agx_navigation_tasks::TravelEnvelopeAdmission>> envelopes_;
  std::unique_ptr<agx_navigation_tasks::TravelPolicy> travel_policy_;
  std::string departure_session_,travel_failure_detail_;std::uint64_t departure_epoch_=0,readiness_sequence_=0,permit_readiness_sequence_=0;
  bool qualified_path_=false,readiness_active_=false;
  double last_readiness_stamp_=0,last_clearance_stamp_=0;
  agx_workbench_navigation::TravelReadiness last_readiness_;
  std::shared_ptr<agx_robot_task_runtime::RobotStateStore> state_;
  std::unique_ptr<agx_robot_task_runtime::RobotTaskRuntimeBinding> runtime_binding_;
  std::shared_ptr<agxcore::TubeLiveObserver> live_observer_;
  std::string run_id_,departure_id_;
  std::map<std::string,std::shared_ptr<const agx_xarm_pick::ResolvedTubeTaskConfig>> admitted_configs_;
  std::uint64_t observation_sequence_=0;
  double last_custody_stamp_=0;
  std::mutex mutex_; Authority authority_; nav_msgs::Odometry odom_;
  std::shared_ptr<agx_task_workflow::ExecutionContext> context_;
  const double task_timeout_s_;
  std::string active_task_id_;
  mutable std::atomic<bool> runtime_fault_{false};
  std::unique_ptr<agx_robot_task_runtime::BaseGuard> base_guard_; std::string grasp_stop_reason_,target_station_;
  std::atomic<bool> external_stop_{false},watchdog_ready_{false}; std::atomic<double> watchdog_seen_{0};
  std::string phase_="IDLE",owned_goal_,navigation_detail_;
  bool navigation_started_=false,owned_terminal_=false,any_active_=true,command_zero_=false;
  bool goal_sent_=false,motion_started_=false;
  double authority_at_=0,odom_at_=0,command_at_=0,status_at_=0,status_stamp_=0;
  double receipt_limit_=.25,header_limit_=.35,future_skew_=.05,zero_=1e-9,linear_=.02,angular_=.01,dwell_=.5;
  double position_limit_=.015,yaw_limit_=.035;
};

// Select read-only node identity before ROS registration: reusing the execution
// node name would shut down an active workflow before validation even starts.
static bool CheckOnlyArgument(int argc,char** argv) {
  bool check=false;
  const std::string prefix="_check_only:=";
  for (int i=1;i<argc;++i) {
    const std::string argument=argv[i];
    if (argument.compare(0,prefix.size(),prefix)==0)
      check=YAML::Load(argument.substr(prefix.size())).as<bool>();
  }
  return check;
}

int main(int argc,char** argv) {
  std::setvbuf(stdout,nullptr,_IOLBF,0);
  bool check_only=false;
  try {check_only=CheckOnlyArgument(argc,argv);}
  catch (const std::exception& e) {std::cerr << "invalid _check_only: " << e.what() << std::endl;return EXIT_FAILURE;}
  ros::init(argc,argv,check_only?"workbench_workflow_check":"workbench_workflow",
            ros::init_options::NoSigintHandler | (check_only?ros::init_options::AnonymousName:0));
  std::signal(SIGINT,Signal); std::signal(SIGTERM,Signal);
  ros::NodeHandle private_node("~"); std::string path,summary;
  private_node.getParam("config",path); private_node.param<std::string>("summary",summary,"/tmp/workbench_workflow_summary.json");
  bool valid=false;
  agx_task_workflow::Config config; agx_task_workflow::Result result; bool exit_required=false; std::string nav_detail,manifest="[]",run_id;
  std::optional<double> task_timeout_s;
  std::shared_ptr<Workflow> workflow_owner; // Retain pending leaf workers through summary flush and _Exit.
  const double started=Now();
  ros::AsyncSpinner spinner(4); spinner.start();
  try {
    task_timeout_s=LoadTaskTimeout(private_node);
    config=agx_task_workflow::LoadConfig(path);
    for (const auto& task:config.tasks) {agx_task_workflow::TaskResult r;r.id=task.id;r.type=task.type;result.tasks.push_back(r);}
    workflow_owner=std::make_shared<Workflow>(!check_only,*task_timeout_s);auto& workflow=*workflow_owner;run_id=workflow.RunId();
    workflow.Phase("VALIDATE");
    agx_task_workflow::TaskRegistry registry;
    workflow.RegisterRobotTasks(registry);
    agx_task_workflow::examples::record_value::RegisterRecordValueTask(
        registry,[&](const std::string& task_id){return workflow.RuntimePorts(task_id);},
        [](const std::string& value,const agx_task_workflow::TaskRuntimePorts& ports) {
          const auto before=ports.check();
          if(before!=agx_task_workflow::FailureKind::None) {
            agx_task_workflow::StepResult result{false,"record_value interrupted before log callback"};result.failure=before;return result;
          }
          ROS_INFO("record_value: %s",value.c_str());
          const auto after=ports.check();
          agx_task_workflow::StepResult result{after==agx_task_workflow::FailureKind::None,"Value recorded"};result.failure=after;
          return result;
        });
    auto recipe=agx_task_workflow::PrepareRecipe(config,registry);
    auto& tasks=recipe.tasks;
    workflow.AdmitRecipe(recipe);
    manifest=workflow.ConfigManifest();
    const auto availability=workflow.ValidateAvailability(recipe.required_interfaces);
    if (!availability.ok) throw std::runtime_error(availability.detail);
    valid=true;
    workflow.PrepareGraspObservations(tasks);
    if (!check_only) {
      for (auto& task:tasks) {
        const auto execute=task.execute;
        const bool shared_actuators=agx_robot_task_runtime::UsesSharedRobotActuators(task.resources);
        task.execute=[&workflow,execute,shared_actuators](const agx_task_workflow::PhaseSink& phase) {
          workflow.DispatchStarted(shared_actuators);return execute(phase);
        };
      }
      agx_task_workflow::Ports ports;
      ports.task_begin=[&](const agx_task_workflow::PreparedTask& task){workflow.BeginTask(task);};
      ports.cancelled=[&]{return workflow.RuntimeFailure()==agx_task_workflow::FailureKind::Cancelled;};
      ports.stop=[&]{workflow.Stop();};
      ports.phase=[&](const std::string& phase){workflow.Phase(phase);};
      ports.task_event=[&](const agx_task_workflow::TaskResult& task){workflow.TaskEvent(task);};
      ports.runtime_guard=[&]{return workflow.RuntimeFailure();};
      ports.continuation_allowed=[&](const agx_task_workflow::PreparedTask& task,const agx_task_workflow::StepResult& step) {
        return workflow.ContinuationAllowed(task,step);
      };
      result=agx_task_workflow::Run(tasks,ports);workflow.ExpireRuntimePorts();workflow.MarkGraspObservationRunReturned();
      exit_required=workflow.ExitRequired();nav_detail=workflow.NavigationDetail();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  } catch (const agx_task_workflow::PreparationError& e) {
    result.failed_phase="VALIDATE";result.detail=e.what();
    result.failed_task_id=e.task_id;result.failed_task_type=e.task_type;
  } catch (const std::exception& e) {result.failed_phase="VALIDATE";result.detail=e.what();}
  std::string grasp_observation_summary="null";
  if(workflow_owner) {
    if(valid && !check_only)workflow_owner->MarkGraspObservationRunReturned();
    try {grasp_observation_summary=workflow_owner->GraspObservationSummary();} catch(...) {}
    exit_required=exit_required || workflow_owner->ExitRequired();
  }
  const bool grasp_dispatched=std::any_of(result.tasks.begin(),result.tasks.end(),[](const agx_task_workflow::TaskResult& task) {
    return task.type=="grasp_test_tube" && task.dispatched;
  });
  std::ostringstream report;
  report << "{\"task_timeout_s\":";
  if(task_timeout_s)report<<std::setprecision(17)<<*task_timeout_s;else report<<"null";
  report << ",\"task_timeout_clock\":\"steady\",\"run_id\":" << Json(run_id) << ",\"workflow_id\":" << Json(config.workflow_id) << ",\"config\":" << Json(path)
    << ",\"success\":" << (result.success?"true":"false")
    << ",\"validation_only\":" << (check_only?"true":"false") << ",\"valid\":" << (valid?"true":"false")
    << ",\"grasp_dispatched\":" << (grasp_dispatched?"true":"false")
    << ",\"failed_task_id\":" << Json(result.failed_task_id) << ",\"failed_task_type\":" << Json(result.failed_task_type)
    << ",\"failed_phase\":" << Json(result.failed_phase) << ",\"detail\":" << Json(result.detail)
    << ",\"navigation_result\":" << Json(nav_detail) << ",\"elapsed_wall_s\":" << Now()-started
    << ",\"completion\":" << Json(check_only && valid && !exit_required && result.failed_phase.empty()?"Validated":
        result.completion==agx_task_workflow::RunCompletion::Succeeded?"Succeeded":
        result.completion==agx_task_workflow::RunCompletion::CompletedWithFailures?"CompletedWithFailures":
        result.completion==agx_task_workflow::RunCompletion::Cancelled?"Cancelled":"Failed")
    << ",\"state_revision\":" << result.state_revision << ",\"resolved_configs\":" << manifest
    << ",\"blocked_child_exit\":" << (exit_required?"true":"false") << ",\"tasks\":[";
  for (std::size_t i=0;i<result.tasks.size();++i) {
    if(i)report<<',';auto task_json=TaskJson(result.tasks[i]);
    const auto definition=std::find_if(config.tasks.begin(),config.tasks.end(),[&](const agx_task_workflow::TaskSpec& task){return task.id==result.tasks[i].id;});
    if(definition!=config.tasks.end() && definition->parameters["target"]) {
      task_json.pop_back();task_json+=",\"target\":"+Json(definition->parameters["target"].as<std::string>())+"}";
    }
    report<<task_json;
  }
  report << "],\"failed_task_ids\":[";
  for(std::size_t i=0;i<result.failed_task_ids.size();++i) {if(i)report<<',';report<<Json(result.failed_task_ids[i]);}
  report << "],\"grasp_effect_observations\":" << grasp_observation_summary << "}\n";
  std::ofstream file(summary); file << report.str(); file.close();
  if (!file) ROS_ERROR("cannot write workflow summary: %s",summary.c_str());
  std::cout << report.str() << std::flush;
  if (check_only) {
    if (valid && !exit_required && result.failed_phase.empty())
      std::cout << "Validation passed. No tasks were executed.\n" << std::flush;
    else
      std::cerr << "Validation failed: " << (result.detail.empty()?"workflow could not exit cleanly":result.detail) << '\n';
  }
  if (exit_required) std::_Exit(EXIT_FAILURE);
  spinner.stop();
  if (result.completion==agx_task_workflow::RunCompletion::CompletedWithFailures) return 3;
  return (result.success || (check_only && valid))?EXIT_SUCCESS:EXIT_FAILURE;
}
