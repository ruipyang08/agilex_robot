#include <cassert>
#include <functional>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <yaml-cpp/yaml.h>
#include "travel_test_support.hpp"
using namespace travel_test;
namespace {
bool Rejects(const std::function<void()>& f) { try { f(); } catch (const std::exception&) { return true; } return false; }
}
int main() {
  const auto fixture_path=std::filesystem::path(__FILE__).parent_path()/"travel_profile_fixture.yaml";
  std::ifstream fixture(fixture_path);
  assert(fixture);
  const std::string fixture_text{std::istreambuf_iterator<char>(fixture),{}};
  const auto parsed=ParseTravelProfile(fixture_text,fixture_path.string());
  assert(parsed.source_path == fixture_path.string());
  assert(parsed.robot.arm_group == "synthetic_arm" && parsed.robot.observer.fully_open_command == 1000);
  assert(parsed.arm.size() == 2 && parsed.starts.size() == 2 && parsed.base.footprint.size() == 4);
  assert(parsed.starts[1].arm_tolerances_rad.at("joint2") == .01);
  assert(parsed.simulation.enabled && !parsed.hardware.enabled);
  assert(parsed.held_attachment_link == "test_gripper_base" && parsed.held_touch_links.size() == 2);
  assert(parsed.held_center_xyz_m[2] == .1 && parsed.held_orientation_xyzw[3] == 1.);
  for (const auto& field : {"center_xyz_m","orientation_xyzw"}) {
    auto bad=YAML::Load(fixture_text); bad["parameters"]["held_load"][field]=YAML::Load("[0, 0]");
    assert(Rejects([&] { LoadTravelProfile(bad); }));
  }
  for (const auto& path : {"robot","parameters"}) {
    auto bad=YAML::Load(fixture_text); bad[path]["typo"]=1;
    assert(Rejects([&] { LoadTravelProfile(bad); }));
  }
  auto bad_number=YAML::Load(fixture_text); bad_number["parameters"]["lift"]["tolerance_m"]="not-a-number";
  assert(Rejects([&] { LoadTravelProfile(bad_number); }));
  auto p = Profile();
  assert(TravelEnabled(p,TravelMode::Simulation));
  assert(!TravelEnabled(p,TravelMode::Hardware));
  ValidateTravelProfile(p);
  auto empty_only = p;
  for (auto& start : empty_only.starts) start.single_held_allowed = false;
  ValidateTravelProfile(empty_only);
  auto held_only = p;
  for (auto& start : held_only.starts) {
    start.empty_allowed = false;
    start.single_held_allowed = true;
  }
  ValidateTravelProfile(held_only);
  for (int fault = 0; fault < 28; ++fault) {
    auto bad = p;
    if (fault == 0) bad.simulation.evidence_id.clear();
    if (fault == 1) bad.arm[0].tolerance_rad = 0;
    if (fault == 2) bad.arm[0].position_rad = std::numeric_limits<double>::infinity();
    if (fault == 3) bad.arm[1].name = bad.arm[0].name;
    if (fault == 4) bad.lift_tolerance_m = -1;
    if (fault == 5) bad.base.footprint.clear();
    if (fault == 6) bad.base.linear_velocity_mps = 0;
    if (fault == 7) bad.starts.clear();
    if (fault == 8) bad.starts[0].arm_positions_rad.erase("joint1");
    if (fault == 9) bad.starts[0].path_id.clear();
    if (fault == 10) bad.starts[1].id = bad.starts[0].id;
    if (fault == 11) bad.settle_s = bad.preparation_timeout_s;
    if (fault == 12) bad.container_profile_id.clear();
    if (fault == 13) bad.held_max_mass_kg = 0;
    if (fault == 14) for (auto& start : bad.starts) start.empty_allowed = start.single_held_allowed = false;
    if (fault == 15) bad.robot.lift_service.clear();
    if (fault == 16) bad.robot.observer.gripper_max_age_s = 0;
    if (fault == 17) bad.starts[0].arm_tolerances_rad.erase("joint1");
    if (fault == 18) bad.starts[0].arm_tolerances_rad["joint2"] = -1;
    if (fault == 19) bad.base.footprint={{0,2},{1.1756,-1.618},{-1.902,.618},{1.902,.618},{-1.1756,-1.618}};
    if (fault == 20) bad.held_attachment_link = "invalid link";
    if (fault == 21) bad.held_center_xyz_m[0] = std::numeric_limits<double>::quiet_NaN();
    if (fault == 22) bad.held_orientation_xyzw = {0,0,0,0};
    if (fault == 23) bad.held_orientation_xyzw = {0,0,0,2};
    if (fault == 24) bad.held_touch_links.clear();
    if (fault == 25) bad.held_touch_links = {"finger","finger"};
    if (fault == 26) bad.held_touch_links = {"finger","*"};
    if (fault == 27) bad.preparation_timeout_s = 3600.1;
    assert(Rejects([&] { ValidateTravelProfile(bad); }));
  }
  const std::string disabled = R"(
schema_version: 1
profile_id: unqualified
robot: null
simulation: {enabled: false, evidence_id: unqualified}
hardware: {enabled: false, evidence_id: unqualified}
parameters: null
)";
  auto d = ParseTravelProfile(disabled);
  assert(d.config_id.size() == 71 && d.config_id.substr(0,7) == "sha256:");
  assert(ParseTravelProfile(disabled,"/one/path").config_id == ParseTravelProfile(disabled,"/other/path").config_id);
  assert(ParseTravelProfile(disabled + "# changed\n").config_id != d.config_id);
  assert(!TravelEnabled(d,TravelMode::Hardware));
  assert(!TravelEnabled(d,TravelMode::Simulation));
  auto node = YAML::Load(disabled); node["hardware"]["enabled"] = true;
  assert(Rejects([&] { LoadTravelProfile(node); }));
  assert(Rejects([&] { ParseTravelProfile(disabled + "misspelled: true\n"); }));
  assert(Rejects([&] { ParseTravelProfile(disabled + "profile_id: duplicate\n"); }));
  assert(Rejects([&] { ParseTravelProfile(disabled + "---\n{}\n"); }));
  assert(Rejects([&] { TravelEnabled(p,static_cast<TravelMode>(99)); }));
}
