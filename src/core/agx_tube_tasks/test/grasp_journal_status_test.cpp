#include "agx_tube_tasks/grasp_observation_json.hpp"
#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace detail=agx_tube_tasks::detail;
using Status=detail::GraspJournalStatus;
using Stage=detail::GraspJournalStage;
using Unavailable=detail::GraspJournalUnavailable;

Status Minimal() {
  Status value;
  value.run_id="run";value.task_id="A";value.operation_id="run/A/grasp";
  return value;
}
Status Full() {
  auto value=Minimal();
  value.recipe_index=7;value.stage=Stage::FinalDrain;
  value.entered=value.adapter_bound=value.monitor_returned=true;
  value.worker_joined=true;value.exit_required=false;
  value.finish_completed=value.runner_terminal_seen=value.final_acquired=true;
  value.last_sequence=11;value.capture_gap=false;
  value.rows_publish_returned=10;value.failed_rows=1;
  value.first_failed_sequence=5;value.last_failed_sequence=5;
  value.transport_gap=true;value.drain_error_count=2;value.prior_status_error_count=3;
  value.active=true;
  return value;
}
std::vector<bool*> Booleans(Status& value) {
  return {&value.entered,&value.adapter_bound,&value.monitor_returned,
      &*value.worker_joined,&*value.exit_required,&value.finish_completed,
      &value.runner_terminal_seen,&value.final_acquired,&*value.capture_gap,
      &value.transport_gap,&value.counter_overflow,&value.active};
}
std::vector<std::uint64_t*> Integers(Status& value) {
  return {&value.recipe_index,&*value.last_sequence,&value.rows_publish_returned,
      &value.failed_rows,&*value.first_failed_sequence,&*value.last_failed_sequence,
      &value.drain_error_count,&value.prior_status_error_count};
}
void ResetOptional(Status& value,unsigned index) {
  switch(index) {
    case 0:value.worker_joined.reset();break;
    case 1:value.exit_required.reset();break;
    case 2:value.last_sequence.reset();break;
    case 3:value.capture_gap.reset();break;
    case 4:value.first_failed_sequence.reset();break;
    case 5:value.last_failed_sequence.reset();break;
    default:assert(false);
  }
}
void Invalid(const Status& value) {
  bool rejected=false;
  try { (void)detail::EncodeGraspJournalStatus(value); }
  catch(const std::invalid_argument& error) {
    rejected=true;assert(std::string(error.what()).size()<=128);
  }
  assert(rejected);
}
int main() {
  const auto emit=[](const std::string& name,const Status& value) {
    const auto encoded=detail::EncodeGraspJournalStatus(value);
    assert(encoded==detail::EncodeGraspJournalStatus(value));
    std::cout<<name<<'\t'<<encoded<<'\n';
  };
  emit("minimal",Minimal());emit("full",Full());
  auto sample=Full();
  for(unsigned i=0;i<Booleans(sample).size();++i) {
    auto value=Full();auto bit=Booleans(value)[i];*bit=!*bit;
    emit("bool:"+std::to_string(i),value);
  }
  for(unsigned i=0;i<Integers(sample).size();++i) {
    for(auto number:{UINT64_C(0),UINT64_C(9007199254740993),UINT64_MAX}) {
      auto value=Full();*Integers(value)[i]=number;
      emit("uint:"+std::to_string(i)+":"+std::to_string(number),value);
    }
  }
  for(unsigned i=0;i<6;++i) {
    auto value=Full();ResetOptional(value,i);emit("null:"+std::to_string(i),value);
  }
  for(auto stage:{Stage::Progress,Stage::FinalDrain,Stage::Unavailable,Stage::NotStarted}) {
    auto value=Full();value.stage=stage;emit("stage:"+std::to_string(static_cast<int>(stage)),value);
  }
  for(auto reason:{Unavailable::PreparationFailed,Unavailable::IdentityInvalid,
      Unavailable::EnableFailed,Unavailable::BindingUnavailable}) {
    auto value=Full();value.unavailable_reason=reason;
    emit("unavailable:"+std::to_string(static_cast<int>(reason)),value);
  }
  auto value=Minimal();value.run_id="r/\xc2\xb5";value.task_id="A\n\"";
  value.operation_id=value.run_id+"/"+value.task_id+"/grasp";emit("identity",value);
  value=Minimal();value.run_id="r";value.task_id=std::string(504,'t');
  value.operation_id=value.run_id+"/"+value.task_id+"/grasp";emit("identity512",value);
  value=Minimal();value.run_id.clear();Invalid(value);
  value=Minimal();value.task_id.clear();Invalid(value);
  value=Minimal();value.operation_id+="/wrong";Invalid(value);
  value=Minimal();value.run_id=std::string("\x80",1);value.operation_id=value.run_id+"/A/grasp";Invalid(value);
  value=Minimal();value.run_id="r";value.task_id=std::string(505,'t');
  value.operation_id=value.run_id+"/"+value.task_id+"/grasp";Invalid(value);
  value=Minimal();value.stage=static_cast<Stage>(-1);Invalid(value);
  value=Minimal();value.unavailable_reason=static_cast<Unavailable>(-1);Invalid(value);
}
