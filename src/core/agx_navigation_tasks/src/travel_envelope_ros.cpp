#include "agx_navigation_tasks/travel_envelope_ros.hpp"
#include "agx_navigation_tasks/travel_envelope.hpp"
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <vector>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
namespace agx_navigation_tasks {
namespace {
double Now() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}
struct TravelEnvelopeAdmission::Impl {
  Impl(ros::NodeHandle& source,const TravelProfile& profile)
      :node(source),spinner(1,&queue),monitor(profile) {
    node.setCallbackQueue(&queue);
    for (const auto& component:TravelEnvelopeComponents()) {
      subscribers.push_back(node.subscribe<std_msgs::String>(
          "/workbench_nav/effective_config/"+component,1,
          [this,component](const std_msgs::String::ConstPtr& report) {
            std::lock_guard<std::mutex> lock(mutex);
            monitor.Observe(component,report->data,Now());
          }));
    }
    spinner.start();
  }
  ~Impl() { spinner.stop(); }
  bool Publishers() const {
    for (const auto& subscriber:subscribers) if (subscriber.getNumPublishers()!=1) return false;
    return true;
  }
  ros::NodeHandle node;
  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner;
  std::vector<ros::Subscriber> subscribers;
  mutable std::mutex mutex;
  TravelEnvelopeMonitor monitor;
};
TravelEnvelopeAdmission::TravelEnvelopeAdmission(ros::NodeHandle& node,const TravelProfile& profile)
    :impl_(new Impl(node,profile)) {}
TravelEnvelopeAdmission::~TravelEnvelopeAdmission()=default;
bool TravelEnvelopeAdmission::Guard() {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!ros::ok() || !impl_->Publishers()) impl_->monitor.Fail("effective report publisher missing/ambiguous");
  return impl_->monitor.Guard(Now());
}
std::string TravelEnvelopeAdmission::Digest() const {
  std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->monitor.Digest();
}
std::string TravelEnvelopeAdmission::SnapshotYaml() const {
  std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->monitor.SnapshotYaml();
}
std::string TravelEnvelopeAdmission::Detail() const {
  std::lock_guard<std::mutex> lock(impl_->mutex); return impl_->monitor.Detail();
}
std::shared_ptr<TravelEnvelopeAdmission> AdmitTravelEnvelope(ros::NodeHandle& node,const TravelProfile& profile,double timeout) {
  if (!std::isfinite(timeout) || timeout<=0 || timeout>60)
    throw std::invalid_argument("effective report admission timeout must be in (0,60]");
  auto admission=std::shared_ptr<TravelEnvelopeAdmission>(new TravelEnvelopeAdmission(node,profile));
  const double deadline=Now()+timeout;
  while (ros::ok() && Now()<deadline) {
    {
      std::lock_guard<std::mutex> lock(admission->impl_->mutex);
      auto& state=admission->impl_->monitor;
      if (state.Faulted()) throw std::runtime_error(state.Detail());
      if (admission->impl_->Publishers() && state.TryAdmit(Now())) return admission;
      if (state.Faulted()) throw std::runtime_error(state.Detail());
    }
    ros::WallDuration(.01).sleep();
  }
  throw std::runtime_error("effective travel envelope admission timed out: "+admission->Detail());
}
}  // namespace agx_navigation_tasks
