#pragma once
#include <any>
#include <map>
#include <string>
#include <utility>
#include "agx_task_workflow/contract_id.hpp"

namespace agx_task_workflow {
// Prospective values only. T must own its data: no mutable shared pointers,
// device handles or runtime stores. Get returns a copy, never a writable alias.
class PlanningState {
 public:
  template<class T> T Get(const std::string& key) const {
    ValidateContractId(key);
    const auto found = values_.find(key);
    if (found == values_.end()) return T{};
    if (found->second.type() != typeid(T))
      throw std::invalid_argument("planning key type mismatch: " + key);
    return std::any_cast<T>(found->second);
  }
  template<class T> void Set(const std::string& key, T value) {
    ValidateContractId(key);
    const auto found = values_.find(key);
    if (found != values_.end() && found->second.type() != typeid(T))
      throw std::invalid_argument("planning key type mismatch: " + key);
    values_.insert_or_assign(key, std::any(std::move(value)));
  }
 private:
  std::map<std::string, std::any> values_;
};
}
