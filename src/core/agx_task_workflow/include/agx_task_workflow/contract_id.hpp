#pragma once
#include <stdexcept>
#include <string>

namespace agx_task_workflow {
// Stable descriptive IDs: lowercase dotted segments, e.g. vendor.instrument.ready.
// A valid ID describes a capability; it never authorizes a runtime action.
inline void ValidateContractId(const std::string& id) {
  bool start = true, namespaced = false;
  for (char c : id) {
    if (c == '.' && !start) { start = true; namespaced = true; continue; }
    const bool letter = c >= 'a' && c <= 'z';
    if (!letter && (start || !((c >= '0' && c <= '9') || c == '_')))
      throw std::invalid_argument("invalid namespaced contract ID: " + id);
    start = false;
  }
  if (start || !namespaced)
    throw std::invalid_argument("invalid namespaced contract ID: " + id);
}
}
