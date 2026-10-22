#include <agx_workbench_navigation/motion_policy_io.hpp>
#include <cassert>
#include <fstream>
#include <string>
#include <unistd.h>

namespace nav=agx_workbench_navigation;
int main(int argc,char** argv) {
  assert(argc==2);
  assert(nav::MotionBytesDigest("abc")=="sha256:ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  const auto policy=nav::LoadInstalledMotionPolicy(argv[1]);
  assert(!policy.execution_enabled && policy.preflight.frame_id=="map");
  assert(policy.preflight.geometry.footprint.size()==4);
  assert(policy.preflight.geometry.hard_padding_m==.25);
  assert(policy.preflight.limits.max_speed_mps==.5);
  assert(policy.preflight.model_digest==nav::MotionFileDigest(argv[1]));
  std::ifstream stream(argv[1]);const std::string raw((std::istreambuf_iterator<char>(stream)),{});
  char path[]="/tmp/motion-policy-test-XXXXXX";
  const int file=mkstemp(path);assert(file>=0);close(file);
  const auto rejects=[&](std::string candidate) {
    std::ofstream out(path);out<<candidate;out.close();
    bool rejected=false;
    try {nav::LoadInstalledMotionPolicy(path);}catch(const std::exception&) {rejected=true;}
    assert(rejected);
  };
  const auto change=[&](std::string from,std::string to) {
    auto candidate=raw;const auto pos=candidate.find(from);assert(pos!=std::string::npos);
    candidate.replace(pos,from.size(),to);rejects(candidate);
  };
  change("execution_enabled: false","execution_enabled: true");
  change("qualification: unqualified","qualification: qualified");
  change("frame_id: base_link","frame_id: odom");
  change("geometry: 0.025","geometry: -0.025");
  change("geometry: 0.025","geometry: '0.025'");
  {
    auto candidate=raw;
    for(const auto& field: {std::string("geometry: 0.025"),std::string("localization: 0.10")}) {
      const auto pos=candidate.find(field);assert(pos!=std::string::npos);
      candidate.replace(pos,field.size(),field.substr(0,field.find(':')+1)+" 1e308");
    }
    rejects(candidate);
  }
  change("max_speed_mps: 0.50","max_speed_mps: 0.51");
  {
    auto candidate=raw;const auto pos=candidate.find("max_yaw_rate_radps: 0.40");
    assert(pos!=std::string::npos);
    candidate.replace(pos,std::string("max_yaw_rate_radps: 0.40").size(),"max_yaw_rate_radps: 0.55");
    std::ofstream out(path);out<<candidate;out.close();
    const auto tuned=nav::LoadInstalledMotionPolicy(path);
    assert(tuned.preflight.limits.max_yaw_rate_radps==.55 && !tuned.execution_enabled);
  }
  change("max_yaw_rate_radps: 0.40","max_yaw_rate_radps: 0.550001");
  change("min_turn_radius_m: 0.4764","min_turn_radius_m: 0.1");
  rejects(raw+"unknown_option: true\n");
  rejects(raw+"frame_id: base_link\n");
  rejects(std::string(1024*1024+1,'x'));
  unlink(path);
}
