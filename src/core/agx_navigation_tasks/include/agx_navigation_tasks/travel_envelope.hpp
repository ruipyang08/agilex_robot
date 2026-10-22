#pragma once
#include <map>
#include <string>
#include <vector>
#include "agx_navigation_tasks/travel_profile.hpp"
namespace agx_navigation_tasks {
using TravelEnvelopeReports = std::map<std::string,std::string>;
struct TravelEnvelopeSnapshot { std::string digest, snapshot_yaml; };
const std::vector<std::string>& TravelEnvelopeComponents();
TravelEnvelopeSnapshot EvaluateTravelEnvelope(const TravelProfile&, const TravelEnvelopeReports&);
// Pure receipt-clock monitor. Caller serializes access. A fault never clears.
class TravelEnvelopeMonitor {
 public:
  explicit TravelEnvelopeMonitor(TravelProfile profile);
  void Observe(const std::string& component, const std::string& report, double receipt_s);
  bool TryAdmit(double now_s);
  bool Guard(double now_s);
  void Fail(const std::string& detail);
  bool Faulted() const { return faulted_; }
  const std::string& Digest() const { return snapshot_.digest; }
  const std::string& SnapshotYaml() const { return snapshot_.snapshot_yaml; }
  const std::string& Detail() const { return detail_; }
 private:
  TravelProfile profile_;
  TravelEnvelopeReports reports_;
  std::map<std::string,double> receipts_;
  TravelEnvelopeSnapshot snapshot_;
  bool admitted_=false, faulted_=false;
  std::string detail_="effective startup reports missing";
};
}  // namespace agx_navigation_tasks
