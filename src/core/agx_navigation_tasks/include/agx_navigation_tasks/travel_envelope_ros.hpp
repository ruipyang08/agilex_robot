#pragma once
#include <memory>
#include <string>
#include "agx_navigation_tasks/travel_profile.hpp"
namespace ros { class NodeHandle; }
namespace agx_navigation_tasks {
// Owns a dedicated callback queue; the host must retain this object and call
// Guard before/during every departure. A fault permanently invalidates it.
class TravelEnvelopeAdmission {
 public:
  ~TravelEnvelopeAdmission();
  bool Guard();
  std::string Digest() const;
  std::string SnapshotYaml() const;
  std::string Detail() const;
 private:
  struct Impl;
  explicit TravelEnvelopeAdmission(ros::NodeHandle&, const TravelProfile&);
  std::unique_ptr<Impl> impl_;
  friend std::shared_ptr<TravelEnvelopeAdmission> AdmitTravelEnvelope(
      ros::NodeHandle&, const TravelProfile&, double);
};
// Throws on malformed/unsafe reports, missing publishers, or bounded timeout.
// Timeout is wall-clock, finite (0,60] seconds; no motion or parameter writes.
std::shared_ptr<TravelEnvelopeAdmission> AdmitTravelEnvelope(
    ros::NodeHandle&, const TravelProfile&, double admission_timeout_s=3.0);
}  // namespace agx_navigation_tasks
