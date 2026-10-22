#include "agx_tube_tasks/grasp_observation_json.hpp"
#include "agx_tube_tasks/grasp_task.hpp"
#include <cassert>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <locale>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

namespace { std::atomic<bool> fail_allocation{false}; }
void* operator new(std::size_t n) {
  if(fail_allocation.exchange(false)) throw std::bad_alloc();
  if(void* p=std::malloc(n?n:1)) return p;
  throw std::bad_alloc();
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete(void* p,std::size_t) noexcept { std::free(p); }

namespace wf=agx_tube_tasks;
using Row=wf::GraspEffectObservation;
using wf::detail::EncodeGraspEffectObservation;
using agxcore::TubeEffectKind;

Row Minimal() {
  Row r;r.run_id="run";r.task_id="A";r.operation_id="run/A/grasp";
  r.observation_sequence=1;r.object={"run/A/grasp/selected-tube",agx_robot_task_runtime::ObjectIdentity::RunLocal,"tube-profile"};
  r.source_event={1,TubeEffectKind::MeasuredStop,{"gripper",9.5,123.25}};
  r.adapter_accepted=true;return r;
}
agx_robot_task_runtime::ObjectRef Object(const std::string& tag) {return {"object/"+tag,agx_robot_task_runtime::ObjectIdentity::Registered,"container/"+tag};}
agx_robot_task_runtime::Evidence Evidence(const std::string& tag,double stamp) {
  return {"source/"+tag,stamp,stamp+100,"run/"+tag,"task/"+tag,"operation/"+tag};
}
Row Full() { // Deliberately synthetic, not a claim that these states coexist physically.
  Row r=Minimal();r.observation_sequence=17;r.object=Object("root");
  r.source_event={19,TubeEffectKind::Captured,{"source/event",11.25,111.25}};
  r.application=agx_robot_task_runtime::StateUpdate{agx_robot_task_runtime::UpdateStatus::Conflict,53,false,"application detail"};
  r.adapter_accepted=false;r.snapshot_revision=61;r.snapshot_halted=true;
  r.gripper={agx_robot_task_runtime::GripperOccupancy::Holding,Object("gripper"),Evidence("gripper",12.25),62,true};
  agx_robot_task_runtime::ObjectLocation location;location.object=Object("location");
  location.last_confirmed_holder=agx_robot_task_runtime::HolderRef{agx_robot_task_runtime::HolderKind::FixtureSlot,"fixture/last","slot/last"};
  location.last_confirmed_evidence=Evidence("last",13.25);location.certainty=agx_robot_task_runtime::LocationCertainty::Uncertain;
  location.current_evidence=Evidence("current",14.25);
  location.pending=agx_robot_task_runtime::PendingTransfer{"operation/pending",Object("pending"),
      agx_robot_task_runtime::HolderRef{agx_robot_task_runtime::HolderKind::FixtureSlot,"fixture/source","slot/source"},
      agx_robot_task_runtime::HolderRef{agx_robot_task_runtime::HolderKind::Gripper,"fixture/destination","slot/destination"}};
  location.revision=63;location.fresh=false;r.object_location=location;
  r.transport={agx_robot_task_runtime::TransportLoad::Unsupported,agx_robot_task_runtime::TravelEligibility::SafeClear,Object("transport"),
      Evidence("custody",15.25),Evidence("clearance",16.25),true,false,true,65};
  r.matched_effect=agx_robot_task_runtime::EffectRecord{
      {"effect/match",agx_robot_task_runtime::EffectKind::Released,Object("effect"),
       agx_robot_task_runtime::HolderRef{agx_robot_task_runtime::HolderKind::FixtureSlot,"fixture/effect","slot/effect"},agx_robot_task_runtime::TravelEligibility::CapturedInFixture},
      Evidence("match",17.25),"operation/match",71,73,agx_robot_task_runtime::UpdateStatus::Duplicate};
  return r;
}
std::vector<double*> Doubles(Row& r) {
  std::vector<double*> out{&r.source_event.evidence.source_stamp_s,&r.source_event.evidence.receipt_steady_s};
  for(agx_robot_task_runtime::Evidence* e:{&*r.gripper.evidence,&*r.object_location->last_confirmed_evidence,
      &*r.object_location->current_evidence,&*r.transport.custody_evidence,
      &*r.transport.clearance_evidence,&r.matched_effect->evidence}) {
    out.push_back(&e->source_stamp_s);out.push_back(&e->receipt_steady_s);
  }
  return out;
}
std::vector<std::uint64_t*> Integers(Row& r) {
  return {&r.observation_sequence,&r.source_event.sequence,&r.application->revision,&r.snapshot_revision,
      &r.gripper.revision,&r.object_location->revision,&r.transport.revision,
      &r.matched_effect->expected_revision,&r.matched_effect->resulting_revision};
}
void ResetOptional(Row& r,unsigned field) {
  switch(field) {
    case 0:r.application.reset();break;
    case 1:r.object_location.reset();break;
    case 2:r.matched_effect.reset();break;
    case 3:r.gripper.object.reset();break;
    case 4:r.gripper.evidence.reset();break;
    case 5:r.object_location->last_confirmed_holder.reset();break;
    case 6:r.object_location->last_confirmed_evidence.reset();break;
    case 7:r.object_location->current_evidence.reset();break;
    case 8:r.object_location->pending.reset();break;
    case 9:r.object_location->pending->source.reset();break;
    case 10:r.object_location->pending->destination.reset();break;
    case 11:r.transport.object.reset();break;
    case 12:r.transport.custody_evidence.reset();break;
    case 13:r.transport.clearance_evidence.reset();break;
    case 14:r.matched_effect->effect.object.reset();break;
    case 15:r.matched_effect->effect.holder.reset();break;
    default:assert(false);
  }
}
std::vector<bool*> Booleans(Row& r) {
  return {&r.adapter_accepted,&r.snapshot_halted,&r.application->halt,&r.gripper.fresh,
      &r.object_location->fresh,&r.transport.pending,&r.transport.uncertain,&r.transport.ready};
}
std::vector<std::string*> Strings(Row& r) {
  // Bound top-level identity is exercised separately; this visits every nested string.
  std::vector<std::string*> out{&r.source_event.evidence.source,&r.application->detail,
      &r.object_location->pending->operation_id,&r.matched_effect->effect.id,&r.matched_effect->operation_id};
  for(agx_robot_task_runtime::ObjectRef* o:{&r.object,&*r.gripper.object,&r.object_location->object,
      &r.object_location->pending->object,&*r.transport.object,&*r.matched_effect->effect.object}) {
    out.push_back(&o->id);out.push_back(&o->container_profile);
  }
  for(agx_robot_task_runtime::HolderRef* h:{&*r.object_location->last_confirmed_holder,&*r.object_location->pending->source,
      &*r.object_location->pending->destination,&*r.matched_effect->effect.holder}) {
    out.push_back(&h->fixture_instance);out.push_back(&h->slot);
  }
  for(agx_robot_task_runtime::Evidence* e:{&*r.gripper.evidence,&*r.object_location->last_confirmed_evidence,
      &*r.object_location->current_evidence,&*r.transport.custody_evidence,
      &*r.transport.clearance_evidence,&r.matched_effect->evidence}) {
    out.push_back(&e->source);out.push_back(&e->run_id);out.push_back(&e->task_id);out.push_back(&e->operation_id);
  }
  return out;
}
void SetEnum(Row& r,unsigned type,int value) {
  switch(type) {
    case 0:r.source_event.kind=static_cast<TubeEffectKind>(value);break;
    case 1:r.object.identity=static_cast<agx_robot_task_runtime::ObjectIdentity>(value);break;
    case 2:r.object_location->last_confirmed_holder->kind=static_cast<agx_robot_task_runtime::HolderKind>(value);break;
    case 3:r.object_location->certainty=static_cast<agx_robot_task_runtime::LocationCertainty>(value);break;
    case 4:r.gripper.occupancy=static_cast<agx_robot_task_runtime::GripperOccupancy>(value);break;
    case 5:r.transport.load=static_cast<agx_robot_task_runtime::TransportLoad>(value);break;
    case 6:r.transport.eligibility=static_cast<agx_robot_task_runtime::TravelEligibility>(value);break;
    case 7:r.matched_effect->effect.kind=static_cast<agx_robot_task_runtime::EffectKind>(value);break;
    case 8:r.application->status=static_cast<agx_robot_task_runtime::UpdateStatus>(value);break;
    default:assert(false);
  }
}
struct Harness {
  unsigned clock_calls=0;
  std::shared_ptr<agx_robot_task_runtime::RobotStateStore> state=std::make_shared<agx_robot_task_runtime::RobotStateStore>(
      [this]{++clock_calls;return 10.;},[this]{++clock_calls;return 20.;});
  std::shared_ptr<wf::GraspEffectAdapter> adapter=NewAdapter(0);
  std::shared_ptr<wf::GraspEffectAdapter> NewAdapter(std::uint64_t revision) {
    auto a=std::make_shared<wf::GraspEffectAdapter>(state,"run","A","run/A/grasp","tube-profile",revision);
    assert(a->EnableObservationJournal());return a;
  }
  bool Apply(TubeEffectKind kind,std::uint64_t sequence,const std::string& source="measured") {
    return adapter->Apply({sequence,kind,{source,10.,20.}});
  }
  Row Last() {auto batch=adapter->TakeObservations();assert(batch.count && !batch.gap);return batch.records[batch.count-1];}
};
using Emit=std::function<void(const std::string&,const Row&)>;
void RealRows(const Emit& emit) {
  Harness h;assert(h.Apply(TubeEffectKind::KnownEmpty,1));auto applied=h.Last();
  assert(applied.application && applied.application->status==agx_robot_task_runtime::UpdateStatus::Applied && applied.matched_effect);
  assert(!applied.object_location && !applied.gripper.object && !applied.transport.object);
  assert(!applied.matched_effect->effect.object && !applied.matched_effect->effect.holder);
  const auto calls=h.clock_calls;emit("real:applied",applied);assert(h.clock_calls==calls);
  assert(h.Apply(TubeEffectKind::KnownEmpty,1));auto replay=h.Last();
  assert(replay.adapter_accepted && !replay.application && !replay.matched_effect);emit("real:replay",replay);
  assert(!h.Apply(TubeEffectKind::KnownEmpty,1,"changed"));auto rejected=h.Last();
  assert(!rejected.adapter_accepted && !rejected.application && !rejected.matched_effect);emit("real:rejected",rejected);
  assert(!h.Apply(TubeEffectKind::MeasuredStop,2));emit("real:latched_rejection",h.Last());
  {Harness s;assert(s.Apply(TubeEffectKind::MeasuredStop,1));auto row=s.Last();
    assert(row.adapter_accepted && !row.application && !row.matched_effect);emit("real:stop",row);}
  {Harness s;assert(s.Apply(TubeEffectKind::KnownEmpty,1));assert(s.Apply(TubeEffectKind::TransferBegun,2));auto row=s.Last();
    assert(row.application && !row.matched_effect && row.transport.pending);
    assert(row.object_location && row.object_location->pending && !row.object_location->pending->source);
    assert(row.object_location->pending->destination && !row.object_location->last_confirmed_holder &&
        !row.object_location->last_confirmed_evidence && !row.object_location->current_evidence);
    assert(!row.gripper.object && !row.transport.object && !row.transport.clearance_evidence);
    emit("real:transfer",row);
    assert(s.Apply(TubeEffectKind::Captured,3));auto captured=s.Last();
    assert(captured.object_location && captured.object_location->fresh);emit("real:captured",captured);
    assert(s.Apply(TubeEffectKind::ExtractedSafeClear,4));emit("real:extracted",s.Last());}
  for(bool conflict:{false,true}) {Harness s;assert(s.Apply(TubeEffectKind::KnownEmpty,1));
    s.adapter=s.NewAdapter(s.state->Snapshot().revision);
    assert(s.Apply(TubeEffectKind::KnownEmpty,1,conflict?"changed":"measured")==!conflict);auto row=s.Last();
    assert(row.application && row.application->status==(conflict?agx_robot_task_runtime::UpdateStatus::Conflict:agx_robot_task_runtime::UpdateStatus::Duplicate));
    assert(bool(row.matched_effect)==conflict);
    if(conflict)assert(row.application->halt && row.matched_effect->resulting_revision==row.application->revision &&
        row.matched_effect->evidence.source=="changed");
    emit(conflict?"real:conflict":"real:duplicate",row);}
  {Harness s;assert(!s.Apply(static_cast<TubeEffectKind>(999),1));auto row=s.Last();
    assert(!row.application && !row.matched_effect);emit("real:unknown_kind",row);}
  {Harness s;std::uint64_t bits=UINT64_C(0x7ff8000000000042);agxcore::TubeEffect event{1,TubeEffectKind::KnownEmpty,{"nonfinite",0,20}};
    std::memcpy(&event.evidence.source_stamp_s,&bits,8);assert(!s.adapter->Apply(event));auto row=s.Last();
    assert(row.application && row.application->status==agx_robot_task_runtime::UpdateStatus::Conflict && row.matched_effect);emit("real:nonfinite",row);}
  {unsigned calls=0;auto state=std::make_shared<agx_robot_task_runtime::RobotStateStore>([&]{return ++calls==1?10.:12.;},[]{return 20.;},1.);
    auto a=std::make_shared<wf::GraspEffectAdapter>(state,"run","A","run/A/grasp","tube-profile",0);
    assert(a->EnableObservationJournal());assert(a->Apply({1,TubeEffectKind::KnownEmpty,{"measured",10,20}}));
    auto batch=a->TakeObservations();assert(batch.count==1);const auto& row=batch.records[0];
    assert(row.application->status==agx_robot_task_runtime::UpdateStatus::Applied && !row.gripper.fresh);emit("real:stale_snapshot",row);}
  for(std::size_t size:{512u,513u}) {Harness s;assert(s.Apply(TubeEffectKind::KnownEmpty,1,std::string(size,'s')));
    auto batch=s.adapter->TakeObservations();if(size==512){assert(batch.count==1&&!batch.gap);emit("real:source512",batch.records[0]);}
    else assert(batch.count==0&&batch.gap&&batch.last_sequence==1);}
}
void Invalid(const Row& r) {
  bool threw=false;try{(void)EncodeGraspEffectObservation(r);}catch(const std::invalid_argument& e){threw=true;assert(std::strlen(e.what())<=128);}
  assert(threw);
}
void Rejections() {
  const std::vector<std::string> invalid={std::string("\x80",1),std::string("\xc0\x80",2),std::string("\xe0\x80\xaf",3),
      std::string("\xed\xa0\x80",3),std::string("\xf4\x90\x80\x80",4),std::string("\xf5\x80\x80\x80",4),
      std::string("\xc2",1),std::string("\xe2\x82",2),std::string("\xf0\x9f\xa7",3),std::string("\xc2 ",2),std::string(513,'x')};
  auto sample=Full();const auto count=Strings(sample).size();
  for(std::size_t i=0;i<count;++i)for(const auto& bytes:invalid){auto r=Full();*Strings(r)[i]=bytes;Invalid(r);assert(*Strings(r)[i]==bytes);}
  for(int field=0;field<4;++field) {
    auto r=Minimal();
    if(field==0) {r.run_id.clear();}
    if(field==1) {r.task_id.clear();}
    if(field==2) {r.operation_id.clear();}
    if(field==3) {r.operation_id+="/wrong";}
    Invalid(r);
  }
  for(const auto& bytes:invalid) {auto r=Minimal();r.run_id=bytes;r.operation_id=r.run_id+"/A/grasp";Invalid(r);
    r=Minimal();r.task_id=bytes;r.operation_id="run/"+bytes+"/grasp";Invalid(r);}
  {auto r=Minimal();r.run_id="r";r.task_id=std::string(505,'t');
    r.operation_id=r.run_id+"/"+r.task_id+"/grasp";assert(r.operation_id.size()==513);Invalid(r);}
  {auto r=Minimal();r.run_id=std::string(512,'r');r.operation_id=r.run_id+"/A/grasp";Invalid(r);}
  auto r=Full();fail_allocation=true;bool bad=false;
  try{(void)EncodeGraspEffectObservation(r);}catch(const std::bad_alloc&){bad=true;}
  fail_allocation=false;assert(bad);assert(r.source_event.evidence.source=="source/event");
}
struct Comma : std::numpunct<char> {char do_decimal_point()const override{return ',';}};
void Vectors(const Emit& emit) {
  emit("minimal",Minimal());emit("full",Full());RealRows(emit);
  const std::uint64_t bits[]={0,UINT64_C(0x8000000000000000),1,UINT64_C(0x8000000000000001),
    UINT64_C(0x000fffffffffffff),UINT64_C(0x0010000000000000),UINT64_C(0x0010000000000001),
    UINT64_C(0x3fefffffffffffff),UINT64_C(0x3ff0000000000000),UINT64_C(0x3ff0000000000001),
    UINT64_C(0x3fd5555555555555),UINT64_C(0xbfd5555555555555),UINT64_C(0x7fefffffffffffff),UINT64_C(0xffefffffffffffff),
    UINT64_C(0x7ff0000000000000),UINT64_C(0xfff0000000000000),UINT64_C(0x7ff8000000000001),
    UINT64_C(0xfff8000000000042),UINT64_C(0x7ff0000000000001),UINT64_C(0xfff0000000000042)};
  auto sample=Full();
  for(unsigned field=0;field<16;++field) {auto r=Full();ResetOptional(r,field);
    emit("optional:"+std::to_string(field)+":null",r);}
  for(std::size_t field=0;field<Booleans(sample).size();++field) {auto r=Full();auto bit=Booleans(r)[field];
    *bit=!*bit;emit("boolean:"+std::to_string(field)+":toggle",r);}
  for(std::size_t field=0;field<Doubles(sample).size();++field)for(auto bit:bits) {
    auto r=Full();std::memcpy(Doubles(r)[field],&bit,8);emit("double:"+std::to_string(field)+":"+std::to_string(bit),r);
  }
  for(std::size_t field=0;field<Integers(sample).size();++field)for(auto value:{UINT64_C(0),UINT64_C(9007199254740993),UINT64_MAX}) {
    auto r=Full();*Integers(r)[field]=value;emit("integer:"+std::to_string(field)+":"+std::to_string(value),r);
  }
  const int counts[]={8,2,2,3,3,4,4,6,4};
  for(unsigned type=0;type<9;++type)for(int value=-2;value<counts[type];++value){auto r=Full();
    const int raw=value==-2?-7:value==-1?std::numeric_limits<int>::max():value;SetEnum(r,type,raw);
    emit("enum:"+std::to_string(type)+":"+std::to_string(raw),r);}
  std::string special="\"\\/";for(int c=0;c<32;++c)special.push_back(static_cast<char>(c));
  special+="\x7f\xc2\xb5\xe4\xb8\xad\xf0\x9f\xa7\xaa\xf4\x8f\xbf\xbf";
  const std::string boundary=std::string(508,'x')+"\xf0\x9f\xa7\xaa";
  for(std::size_t field=0;field<Strings(sample).size();++field)for(unsigned kind=0;kind<2;++kind){auto r=Full();
    *Strings(r)[field]=kind?boundary:special;emit("string:"+std::to_string(field)+":"+std::to_string(kind),r);}
  auto identity=Minimal();identity.run_id="r/\xc2\xb5";identity.task_id="t/\xe4\xb8\xad";
  identity.operation_id=identity.run_id+"/"+identity.task_id+"/grasp";emit("identity",identity);
  identity=Minimal();identity.run_id="r";identity.task_id=std::string(504,'t');
  identity.operation_id=identity.run_id+"/"+identity.task_id+"/grasp";
  assert(identity.operation_id.size()==512);emit("identity512",identity);
  for(bool run:{false,true}) {identity=Minimal();(run?identity.run_id:identity.task_id)=special;
    identity.operation_id=identity.run_id+"/"+identity.task_id+"/grasp";
    emit(run?"identity_special_run":"identity_special_task",identity);}
  const auto old=std::locale();std::locale::global(std::locale(old,new Comma));
  try{emit("locale",Full());}catch(...){std::locale::global(old);throw;}
  std::locale::global(old);
}
int main(int argc,char** argv) {
  const bool print=argc==2&&std::string(argv[1])=="--vectors";assert(argc==1||print);
  Rejections();std::size_t rows=0;
  Vectors([&](const std::string& name,const Row& r){const auto encoded=EncodeGraspEffectObservation(r);
    assert(!encoded.empty()&&encoded.front()=='{'&&encoded.back()=='}');
    assert(encoded==EncodeGraspEffectObservation(r));++rows;
    if(print)std::cout<<name<<'\t'<<encoded<<'\n';});
  assert(rows>400);if(!print)std::cout<<"serializer fixture: "<<rows<<" vectors and rejection checks completed\n";
}
