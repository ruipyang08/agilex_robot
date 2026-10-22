#include "agx_workbench_navigation/ranger_control.hpp"
#include <cassert>
#include <cmath>
#include <limits>
#include <iostream>
#include <stdexcept>
using namespace agx_workbench_navigation;

namespace {
RangerControlDecision arcCommand(double v,double w) {
  return {PlannerState::FOLLOW_ARC,v,0,w,0,"arc regression"};
}

void assertStrictBounds(const RangerControlDecision& result,
    const RangerControlDecision& previous,const CanonicalMotionProgress& motion,
    const RangerPlannerConfig& config,double dt) {
  assert(result.state!=PlannerState::BLOCKED);
  const double v=result.linear_x_mps,w=result.angular_z_radps;
  assert(std::isfinite(v) && std::isfinite(w) && result.linear_y_mps==0);
  assert(v>=0 && v<=config.max_linear_speed_mps);
  assert(v-previous.linear_x_mps<=config.max_linear_accel_mps2*dt);
  assert(previous.linear_x_mps-v<=config.max_linear_decel_mps2*dt);
  assert(std::abs(w-previous.angular_z_radps)<=config.max_angular_accel_radps2*dt);
  assert(v<=std::max(0.,motion.forward_mps)+motion.linear_lead_mps);
  assert(w>=std::min(0.,motion.yaw_radps)-motion.angular_lead_radps);
  assert(w<=std::max(0.,motion.yaw_radps)+motion.angular_lead_radps);
  assert(std::abs(v*w)<=config.max_lateral_accel_mps2);
  if(v>0) {
    assert(std::abs(w)<=v/config.min_turn_radius_m);
    assert(std::abs(w)<=std::numeric_limits<double>::epsilon() || std::abs(v/w)>=config.min_turn_radius_m);
    if(previous.linear_x_mps>0)
      assert(std::abs(w/v-previous.angular_z_radps/previous.linear_x_mps)<=
          config.max_curvature_rate_per_m_s*dt);
  } else assert(std::abs(w)<=config.max_spin_speed_radps);
}

void feasibleSetRegressions(const RangerPlannerConfig& config) {
  struct Case {double source,pv,pw,mv,mw,dv,dw;};
  // Recorded previous/nearest-observed evidence with supported synthetic
  // desired arcs. They prove the geometry gap, not callback-exact replay.
  const Case cases[]={
    {8.775,.02305449173112272,-.014602806073145016,.02114318580320685,-.010533373135708684,.023143185803206852,-.015816144538695016},
    {13.075,.03263252101102769,-.027918003728247202,.031495002717944004,-.023821761563105656,.033495002717944006,-.03033063027587176},
    {17.675,.02902599243952326,-.029640208350584174,.027645488285812505,-.02560485676305908,.029645488285812503,-.03175508769628559},
    {20.175,.014098597047529245,-.0126018619433354,.013300609461631497,-.008429160359444896,.015300609461631497,-.014441296801876481},
    {21.875,.010128084172404508,-.007426076345991498,.00950541561568214,-.003023511640356468,.01150541561568214,-.009011228983739487},
    {23.575,.010144835770872235,-.007431137601783076,.009038915794311073,-.003385011977025771,.011038915794311073,-.008638001009806037},
    {26.175,.014940389348168806,-.013461058407170228,.01403088920174303,-.009063180368718443,.01603088920174303,-.015245126272765044},
    {28.975,.016395748708375173,-.01565403060461313,.014740709700914314,-.011183668517818742,.016740709700914312,-.016820421584833455},
    {31.775,.016020971658201877,-.015317580175740395,.01539678081208982,-.011065674676542984,.01739678081208982,-.017502824266100864},
    {33.375,.009779313256498137,-.00679645668008309,.008082636884738866,-.0022312732090327423,.010082636884738866,-.007511393296118698},
    {34.975,.009851134667524087,-.0068373582643784375,.008154562460544723,-.0027352522604840303,.010154562460544723,-.007555685937434277},
    {36.675,.009499300388363151,-.006998757396626639,.008194659314585524,-.002959640279817737,.010194659314585524,-.008020806872966628},
    {38.375,.010171048794650663,-.0075133542476397575,.009652505410293749,-.0034608963528359226,.011652505410293749,-.0091903315933986474}
  };
  unsigned failures=0;
  for(const auto& c:cases) for(double sign:{1.,-1.}) {
    RangerCommandLimiter limiter(config);
    const auto prior=arcCommand(c.pv,sign*c.pw);
    CanonicalMotionProgress seed{true,c.pv,0,sign*c.pw,.002,.004};
    limiter.limit(prior,c.source-.3,seed);
    const auto accepted=limiter.limit(prior,c.source-.1,seed);
    assert(accepted.linear_x_mps==prior.linear_x_mps && accepted.angular_z_radps==prior.angular_z_radps);
    CanonicalMotionProgress motion{true,c.mv,0,sign*c.mw,.002,.004};
    const auto result=limiter.limit(arcCommand(c.dv,sign*c.dw),c.source,motion);
    if(result.state==PlannerState::BLOCKED) {
      ++failures;std::cerr<<"false infeasibility source="<<c.source<<" sign="<<sign<<'\n';
    } else {
      assert(result.linear_x_mps>0);
      assertStrictBounds(result,prior,motion,config,c.source-(c.source-.1));
    }
  }
  // Both desired interpolation and previous-to-zero interpolation miss this.
  RangerCommandLimiter limiter(config);
  const auto prior=arcCommand(.2,.3);
  CanonicalMotionProgress seed{true,.2,0,.3,.002,.004};
  limiter.limit(prior,1,seed);
  RangerControlDecision accepted;
  for(int i=1;i<=5;++i)accepted=limiter.limit(prior,1+.2*i,seed);
  assert(accepted.linear_x_mps==prior.linear_x_mps && accepted.angular_z_radps==prior.angular_z_radps);
  CanonicalMotionProgress motion{true,.169,0,.294,.002,.004};
  const auto result=limiter.limit(arcCommand(.22,.32),2.1,motion);
  if(result.state==PlannerState::BLOCKED) {++failures;std::cerr<<"false infeasibility generic\n";}
  else {assert(result.linear_x_mps>0);assertStrictBounds(result,prior,motion,config,2.1-2.);}
  assert(failures==0);
}

void seedCommand(RangerCommandLimiter& limiter,const RangerControlDecision& prior) {
  const CanonicalMotionProgress seed{true,prior.linear_x_mps,0,prior.angular_z_radps,.002,.004};
  limiter.limit(prior,0,seed);
  RangerControlDecision result;
  for(int i=1;i<=20;++i)result=limiter.limit(prior,.2*i,seed);
  assert(result.state!=PlannerState::BLOCKED && result.linear_x_mps==prior.linear_x_mps &&
      result.angular_z_radps==prior.angular_z_radps);
}

void feasibilityBoundaryRegressions(const RangerPlannerConfig& config) {
  // Quotient rounding must not turn a supported forward cone into vendor spin.
  const double v=.271493213830477,w=v/config.min_turn_radius_m;
  assert(v/w<config.min_turn_radius_m);
  RangerCommandLimiter limiter(config);
  const auto prior=arcCommand(v,std::nextafter(w,0.));
  seedCommand(limiter,prior);
  CanonicalMotionProgress motion{true,v,0,w,.002,.004};
  auto output=limiter.limit(arcCommand(v,w),4.1,motion);
  assertStrictBounds(output,prior,motion,config,4.1-4.);
  assert(output.angular_z_radps>config.max_spin_speed_radps); // cap is spin-only

  // A directly supplied helper jump must now obey the producer curvature rate.
  limiter.reset();const auto turning=arcCommand(.5,.3);seedCommand(limiter,turning);
  motion={true,.5,0,.3,.002,.004};
  output=limiter.limit(arcCommand(.5,0),4.05,motion);
  assertStrictBounds(output,turning,motion,config,4.05-4.);
  assert(output.angular_z_radps>0 && output.angular_z_radps<.3);
  // A valid preferred component interpolation is preserved exactly.
  limiter.reset();seedCommand(limiter,turning);
  output=limiter.limit(arcCommand(.5,.304),4.05,motion);
  assert(output.linear_x_mps==.5 && output.angular_z_radps==.304);

  // The lateral/curvature product supplies a speed bound before yaw selection.
  limiter.reset();const auto tight=arcCommand(.3,.6);seedCommand(limiter,tight);
  motion={true,.5,0,.6,.002,.004};
  output=limiter.limit(arcCommand(.5,.3),4.1,motion);
  assertStrictBounds(output,tight,motion,config,4.1-4.);
  assert(output.linear_x_mps>.3 && output.linear_x_mps<.325);

  // Curvature/yaw interaction can be empty although independent intervals overlap.
  limiter.reset();const auto small=arcCommand(.02,.03);seedCommand(limiter,small);
  motion={true,.0152,0,.03,.002,.004};
  output=limiter.limit(small,4.01,motion);
  assert(output.state==PlannerState::BLOCKED && output.linear_x_mps==0 && output.angular_z_radps==0);
  assert(!limiter.commandStopped());
  // A separate empty yaw interval, with ample forward progress.
  seedCommand(limiter,small);motion={true,.02,0,0,.002,.004};
  output=limiter.limit(small,4.001,motion);
  assert(output.state==PlannerState::BLOCKED);

  // A fallback terminal arc must pass its own collision callback and retain state.
  const auto generic=arcCommand(.2,.3);
  const RangerControlDecision stop{PlannerState::GOAL_REACHED,0,0,0,0,"goal"};
  limiter.reset();seedCommand(limiter,generic);motion={true,.169,0,.294,.002,.004};
  bool checked=false;
  output=limiter.limitAndValidate(stop,4.1,motion,
      [&](double vx,double wz){checked=true;assert(vx>0 && wz>0);return true;},[] {return true;});
  assert(checked && output.state==PlannerState::FOLLOW_ARC && !limiter.commandStopped());
  assertStrictBounds(output,generic,motion,config,4.1-4.);
  limiter.reset();seedCommand(limiter,generic);checked=false;
  output=limiter.limitAndValidate(arcCommand(.22,.32),4.1,motion,
      [&](double vx,double wz){checked=true;assert(vx>0 && wz>0);return false;},[] {return true;});
  assert(checked && output.state==PlannerState::BLOCKED && output.linear_x_mps==0);
  assert(!limiter.commandStopped());

  // Time pause grants no latitude to reshape when measured bounds contract.
  limiter.reset();seedCommand(limiter,generic);motion={true,.2,0,.3,.002,.004};
  output=limiter.limit(arcCommand(.22,.32),4.,motion);
  assert(output.linear_x_mps==.2 && output.angular_z_radps==.3);
  motion.forward_mps=.169;
  output=limiter.limit(generic,4.,motion);
  assert(output.state==PlannerState::BLOCKED && output.linear_x_mps==0);

  // With no prior positive-v curvature, a first arc uses only the radius cone.
  limiter.reset();motion={true,.1,0,.1,.002,.004};
  limiter.limit(arcCommand(.1,.1),1.,motion);
  output=limiter.limit(arcCommand(.1,.1),1.1,motion);
  assert(output.state!=PlannerState::BLOCKED && output.linear_x_mps>0);
  assert(std::abs(output.angular_z_radps/output.linear_x_mps-1.)<1e-15);

  // A spin tail below vendor epsilon cannot be emitted as a nonzero spin.
  auto tail_config=config;tail_config.max_angular_accel_radps2=1.;tail_config.rate_limit_max_dt_s=1.;
  RangerCommandLimiter tail(tail_config);
  const RangerControlDecision spin{PlannerState::SPIN_ALIGN,0,0,.25,0,"spin"};
  motion={true,0,0,.25,.002,.004};tail.limit(spin,0,motion);
  auto accepted=tail.limit(spin,.25,motion);assert(accepted.angular_z_radps==.25);
  const double tick=std::nextafter(.5,0.);
  output=tail.limit(stop,tick,motion);
  assertStrictBounds(output,accepted,motion,tail_config,tick-.25);
  assert(output.angular_z_radps>std::numeric_limits<double>::epsilon());
}

void arithmeticAndModeRegressions(const RangerPlannerConfig& config) {
  // Exact binary64 subtraction can exceed the debit used to construct a target.
  auto c=config;c.max_linear_decel_mps2=.24;
  RangerCommandLimiter linear(c);
  auto prior=arcCommand(.36676194022777253,0);seedCommand(linear,prior);
  const double requested=prior.linear_x_mps-.24*.125;
  assert(prior.linear_x_mps-requested>.24*.125);
  CanonicalMotionProgress motion{true,prior.linear_x_mps,0,0,.002,.004};
  auto output=linear.limit(arcCommand(requested,0),4.125,motion);
  assertStrictBounds(output,prior,motion,c,.125);

  // The constructed curvature endpoint rounds beyond its original rate bound.
  c=config;c.max_curvature_rate_per_m_s=.4;
  RangerCommandLimiter curve(c);
  const double v=.04976927113749628,k=.9119795388930996;
  prior=arcCommand(v,k*v);seedCommand(curve,prior);
  const double target_w=(k+.4*.125)*v;
  assert(target_w/v-k>.4*.125);
  motion={true,v,0,target_w,.002,.004};
  output=curve.limit(arcCommand(v,target_w),4.125,motion);
  assertStrictBounds(output,prior,motion,c,.125);

  // Division by v can construct a yaw whose rounded product exceeds lateral A.
  c=config;RangerCommandLimiter lateral(c);
  const double lv=.3131960237831032,lw=.2/lv;
  assert(lv*lw>.2);
  prior=arcCommand(lv,lw-.005);seedCommand(lateral,prior);
  motion={true,lv,0,lw,.002,.004};
  output=lateral.limit(arcCommand(lv,lw),4.125,motion);
  assertStrictBounds(output,prior,motion,c,.125);

  // Exact zero curvature coefficients, mirrored without a sign shortcut.
  for(double sign:{1.,-1.}) {
    RangerCommandLimiter zero(config);
    prior=arcCommand(.25,sign*.015625);seedCommand(zero,prior);
    motion={true,.3,0,sign*.015625,.002,.004};
    output=zero.limit(arcCommand(.3,-sign*.015625),4.125,motion);
    assertStrictBounds(output,prior,motion,config,.125);
    assert(output.angular_z_radps==0 && output.linear_x_mps>0);
  }
  // Ackermann yaw can cross zero without selecting a new source motion mode.
  RangerCommandLimiter crossing(config);
  prior=arcCommand(.2,.002);seedCommand(crossing,prior);
  motion={true,.2,0,.002,.002,.004};
  output=crossing.limit(arcCommand(.2,-.002),4.125,motion);
  assert(output.linear_x_mps==.2 && output.angular_z_radps==-.002);
  assertStrictBounds(output,prior,motion,config,.125);

  // Source spin reversal policy is unchanged; the final arbiter owns its
  // measured-stop handoff. Do not silently add a new source reversal policy.
  RangerCommandLimiter spin(config);
  const RangerControlDecision positive{PlannerState::SPIN_ALIGN,0,0,.01,0,"spin"};
  const RangerControlDecision negative{PlannerState::SPIN_ALIGN,0,0,-.01,0,"spin"};
  seedCommand(spin,positive);motion={true,0,0,-.01,.002,.004};
  output=spin.limit(negative,4.125,motion);
  assert(output.linear_x_mps==0 && output.angular_z_radps==-.01);

  // Nonzero singleton yaw intersection and a one-ulp source-time increment.
  c=config;c.max_angular_accel_radps2=.032;
  RangerCommandLimiter singleton(c);
  prior=arcCommand(.1,.01);seedCommand(singleton,prior);
  motion={true,.1,0,.002,.002,.004};
  output=singleton.limit(arcCommand(.1,.005),4.125,motion);
  assertStrictBounds(output,prior,motion,c,.125);
  assert(output.angular_z_radps==.006);
  RangerCommandLimiter tiny(config);prior=arcCommand(.2,.3);seedCommand(tiny,prior);
  motion={true,.2,0,.3,.002,.004};
  const double next=std::nextafter(4.,5.);
  output=tiny.limit(arcCommand(.21,.31),next,motion);
  assertStrictBounds(output,prior,motion,config,next-4.);

  // Finite inputs whose derived limits overflow fail closed, with no history.
  RangerCommandLimiter overflow(config);
  motion={true,std::numeric_limits<double>::max(),0,0,std::numeric_limits<double>::max(),.004};
  overflow.limit(arcCommand(.1,0),0,motion);
  output=overflow.limit(arcCommand(.1,0),.1,motion);
  assert(output.state==PlannerState::BLOCKED && !overflow.commandStopped());
  c=config;c.max_linear_accel_mps2=std::numeric_limits<double>::max();
  c.rate_limit_max_dt_s=std::numeric_limits<double>::max();
  RangerCommandLimiter elapsed(c);motion={true,.1,0,0,.002,.004};
  elapsed.limit(arcCommand(.1,0),0,motion);
  output=elapsed.limit(arcCommand(.1,0),std::numeric_limits<double>::max(),motion);
  assert(output.state==PlannerState::BLOCKED && !elapsed.commandStopped());
  c=config;c.max_curvature_rate_per_m_s=std::numeric_limits<double>::quiet_NaN();
  bool rejected=false;
  try {RangerCommandLimiter invalid(c);} catch(const std::invalid_argument&) {rejected=true;}
  assert(rejected);
}
}  // namespace

int main() {
  RangerPlannerConfig config{};
  config.min_turn_radius_m=.4764;config.max_linear_speed_mps=.5;
  config.max_linear_accel_mps2=.25;config.max_linear_decel_mps2=.3;
  config.max_angular_accel_radps2=.4;config.max_lateral_accel_mps2=.2;
  config.max_spin_speed_radps=.4;config.rate_limit_max_dt_s=.2;
  feasibleSetRegressions(config);
  feasibilityBoundaryRegressions(config);
  arithmeticAndModeRegressions(config);
  CanonicalMotionProgress motion{true,0,0,0,.002,.004};
  RangerCommandLimiter limiter(config);
  RangerControlDecision arc{PlannerState::FOLLOW_ARC,.5,0,.3,0,"arc"};
  auto previous=limiter.limit(arc,1,motion);
  assert(previous.linear_x_mps==0 && previous.angular_z_radps==0);
  double now=1;
  auto advance=[&](const RangerControlDecision& desired,int ticks,bool tracking) {
    for(int i=0;i<ticks;++i) {
      if(tracking) {motion.forward_mps=previous.linear_x_mps;motion.yaw_radps=previous.angular_z_radps;}
      now+=.05;const auto current=limiter.limit(desired,now,motion);
      assert(current.state!=PlannerState::BLOCKED);
      const double a=current.linear_x_mps>=previous.linear_x_mps?.25:.3;
      assert(std::abs(current.linear_x_mps-previous.linear_x_mps)<=a*.05+1e-12);
      assert(std::abs(current.angular_z_radps-previous.angular_z_radps)<=.4*.05+1e-12);
      assert(current.linear_x_mps<=std::max(0.,motion.forward_mps)+.002+1e-12);
      assert(current.angular_z_radps<=std::max(0.,motion.yaw_radps)+.004+1e-12);
      assert(current.angular_z_radps>=std::min(0.,motion.yaw_radps)-.004-1e-12);
      assert(current.linear_y_mps==0);
      assert(current.linear_x_mps==0 || std::abs(current.angular_z_radps)<=current.linear_x_mps/.4764+1e-12);
      assert(std::abs(current.linear_x_mps*current.angular_z_radps)<=.2+1e-12);
      previous=current;
    }
  };
  advance(arc,1000,false);assert(previous.linear_x_mps<=.002+1e-12);
  advance(arc,300,true);assert(std::abs(previous.linear_x_mps-.5)<1e-12);
  // Archived changing-curvature case is shaped before collision checking.
  motion.forward_mps=.5;motion.yaw_radps=.3;arc.angular_z_radps=.325;
  previous=limiter.limit(arc,now+=.05,motion);
  assert(previous.state==PlannerState::FOLLOW_ARC);
  assert(previous.linear_x_mps==.5 && std::abs(previous.angular_z_radps-.304)<1e-12);
  // Paused source time cannot accrue motion even if the desired ray changes.
  arc.angular_z_radps=.1;
  auto paused=limiter.limit(arc,now,motion);
  assert(paused.linear_x_mps==previous.linear_x_mps && paused.angular_z_radps==previous.angular_z_radps);
  // Ordinary arc-to-straight braking is valid while measured yaw is nonzero.
  arc.angular_z_radps=0;advance(arc,30,true);
  assert(previous.angular_z_radps==0 && previous.linear_x_mps==.5);
  RangerControlDecision stopped{PlannerState::GOAL_REACHED,0,0,0,0,"goal"};
  assert(!limiter.commandStopped());
  advance(stopped,1,true);assert(!limiter.commandStopped());
  advance(stopped,100,true);assert(limiter.commandStopped());
  assert(previous.state==PlannerState::GOAL_REACHED && previous.linear_x_mps==0);
  // Spin stationary history is bounded independently, including negative yaw.
  RangerControlDecision spin{PlannerState::SPIN_ALIGN,0,0,-.4,0,"spin"};
  motion={true,0,0,0,.002,.004};advance(spin,1000,false);
  assert(std::abs(previous.angular_z_radps)<=.004+1e-12);
  advance(spin,150,true);assert(previous.angular_z_radps==-.4);
  advance(arc,600,true);assert(previous.linear_x_mps==.5);
  // Sudden contraction has no feasible normal slew solution: explicit fail-zero.
  motion={true,0,0,0,.002,.004};
  auto contraction=limiter.limit(arc,now+=.05,motion);
  assert(contraction.state==PlannerState::BLOCKED && contraction.linear_x_mps==0);
  assert(!limiter.commandStopped());
  previous=limiter.limit(arc,now+=.05,motion);assert(previous.linear_x_mps==0);
  advance(arc,300,true);
  motion.valid=false;
  assert(limiter.limit(arc,now,motion).state==PlannerState::BLOCKED);
  motion.valid=true;previous=limiter.limit(arc,now+=.05,motion);
  advance(arc,300,true);
  const auto regressed=limiter.limit(arc,now-1,motion);
  assert(regressed.state==PlannerState::BLOCKED && regressed.linear_x_mps==0);
  // The production post-interpolation validation path sees the retained curve.
  limiter.reset();motion={true,.5,0,.3,.002,.004};
  previous=limiter.limit({PlannerState::FOLLOW_ARC,.5,0,.3,0,"arc"},now+=.05,motion);
  advance({PlannerState::FOLLOW_ARC,.5,0,.3,0,"arc"},300,true);
  bool checked=false;
  auto blocked=limiter.limitAndValidate(arc,now+=.05,motion,
      [&](double vx,double wz){checked=true;assert(vx>0 && wz>0);return false;},
      []{return true;});
  assert(checked && blocked.state==PlannerState::BLOCKED && blocked.linear_x_mps==0);
  assert(!limiter.commandStopped());
  // The archived ordinary terminal command must remain a smooth source step.
  limiter.reset();
  RangerControlDecision terminal{PlannerState::FOLLOW_ARC,.094375,0,0,0,"terminal"};
  motion={true,.094375,0,0,.002,.004};
  limiter.limit(terminal,now,motion);
  for(int i=0;i<10;++i) limiter.limit(terminal,now+=.1,motion);
  terminal.linear_x_mps=.064375;
  auto braking=limiter.limit(terminal,now+=.1,motion);
  assert(braking.state==PlannerState::FOLLOW_ARC);
  assert(std::abs(braking.linear_x_mps-.064375)<1e-12);
  // Opposing signed motion grants only the fixed lead, never abs(speed).
  limiter.reset();motion={true,-.3,0,-.2,.002,.004};
  arc.angular_z_radps=.3;limiter.limit(arc,now,motion);
  auto opposing=limiter.limit(arc,now+=.1,motion);
  assert(opposing.linear_x_mps<=.002+1e-12 && opposing.angular_z_radps<=.004+1e-12);
  // A blocked spin check is an immediate reset, including normal braking.
  limiter.reset();motion={true,0,0,-.4,.002,.004};
  limiter.limit(spin,now,motion);limiter.limit(spin,now+=.1,motion);
  bool rotation_checked=false;
  blocked=limiter.limitAndValidate(stopped,now+=.01,motion,
      [](double,double){return true;},[&]{rotation_checked=true;return false;});
  assert(rotation_checked && blocked.state==PlannerState::BLOCKED && blocked.angular_z_radps==0);
  RangerControlDecision fault{PlannerState::BLOCKED,0,0,0,0,"cancelled"};
  auto cancelled=limiter.limit(fault,now,motion);
  assert(cancelled.state==PlannerState::BLOCKED && !limiter.commandStopped());
  // An interpolated boundary rounded one ulp above the fixed measured lead
  // previously made the next identical sample look infeasible (seeded replay).
  limiter.reset();motion={true,.011765688466090066,0,.049653656726776373,.002,.004};
  RangerControlDecision rounding{PlannerState::FOLLOW_ARC,.41415555268871218,0,
      .21705576594818538,0,"rounding"};
  for(int i=0;i<100;++i) {
    const auto bounded=limiter.limit(rounding,now+=.1,motion);
    assert(bounded.state!=PlannerState::BLOCKED);
    assert(bounded.linear_x_mps<=motion.forward_mps+motion.linear_lead_mps);
  }
  // Repeated desired speed cannot accumulate past original measured motion.
  limiter.reset();motion={true,0,0,0,.002,.004};
  for(int i=0;i<500;++i) {
    const auto command=limiter.limit(arcCommand(.5,0),now+=.1,motion);
    assert(command.state!=PlannerState::BLOCKED);
    assert(command.linear_x_mps<=.002);
  }
}
