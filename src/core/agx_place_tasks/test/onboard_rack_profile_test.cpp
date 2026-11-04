#include "agx_place_tasks/onboard_rack_profile.hpp"

#include <cassert>
#include <cmath>
#include <filesystem>
#include <string>

#include <yaml-cpp/yaml.h>

using namespace agx_place_tasks;

namespace {

const char* kRack = R"(schema_version: 2
onboard_rack_profiles:
  onboard_rack_v1:
    parent_frame: link_base_lift0
    tip_link: tcp_link
    tool_rpy_deg: [180.00, 0.00, 1.32]
    descend_axis: [0.0, 0.0, -1.0]
    approach_rise_m: 0.150
    slots:
      1: {name: a, center_m: [0.327302, 0.082902, -0.078002]}
      2: {name: b, center_m: [0.329030, 0.007922, -0.078002]}
      3: {name: c, center_m: [0.330758, -0.067058, -0.078002]}
    qualified: true
    evidence_id: fixture-not-a-measurement
)";

bool Close(double a, double b) { return std::fabs(a - b) < 1e-9; }

template <class F>
void Throws(F&& f) {
  bool thrown = false;
  try { f(); } catch (const std::invalid_argument&) { thrown = true; }
  assert(thrown);
}

}  // namespace

int main() {
  const auto parsed = ParseRackProfile(YAML::Load(kRack), "onboard_rack_v1");
  assert(parsed.slots.size() == 3 && parsed.qualified);
  assert(parsed.tip_link == "tcp_link");
  assert(parsed.tool_rpy_deg && Close((*parsed.tool_rpy_deg)[2], 1.32));
  assert(!parsed.slot_tolerance_m);
  assert(Close(parsed.SlotCenter(2).y, 0.007922));
  assert(Close(parsed.ApproachPoint(2).z, 0.071998));
  assert(parsed.Find(3) && parsed.Find(3)->name == "c");
  assert(parsed.Find(9) == nullptr);

  bool out_of_range = false;
  try { parsed.SlotCenter(9); } catch (const std::out_of_range&) { out_of_range = true; }
  assert(out_of_range);

  // Every slot is individually required; a gap would make "slot: N" ambiguous
  // between the teaching record and the recipe.
  {
    auto document = YAML::Load(kRack);
    document["onboard_rack_profiles"]["onboard_rack_v1"]["slots"].remove(2);
    Throws([&] { ParseRackProfile(document, "onboard_rack_v1"); });
  }
  {
    auto document = YAML::Load(kRack);
    document["onboard_rack_profiles"]["onboard_rack_v1"]["slots"][1].remove("name");
    Throws([&] { ParseRackProfile(document, "onboard_rack_v1"); });
  }
  {
    auto document = YAML::Load(kRack);
    document["onboard_rack_profiles"]["onboard_rack_v1"]["slots"][1]["center_m"][0] = 9.0;
    Throws([&] { ParseRackProfile(document, "onboard_rack_v1"); });
  }
  {
    // A quoted number is a string, not a measurement.
    std::string text = kRack;
    const std::string target = "approach_rise_m: 0.150";
    text.replace(text.find(target), target.size(), "approach_rise_m: \"0.150\"");
    Throws([&] { ParseRackProfile(YAML::Load(text), "onboard_rack_v1"); });
  }
  {
    auto document = YAML::Load(kRack);
    document["onboard_rack_profiles"]["onboard_rack_v1"]["approach_rise_m"] = 0.0;
    Throws([&] { ParseRackProfile(document, "onboard_rack_v1"); });
  }
  {
    auto document = YAML::Load(kRack);
    document["onboard_rack_profiles"]["onboard_rack_v1"]["descend_axis"] = YAML::Load("[0.0, 2.0, 0.0]");
    Throws([&] { ParseRackProfile(document, "onboard_rack_v1"); });
  }
  {
    auto document = YAML::Load(kRack);
    document["onboard_rack_profiles"]["onboard_rack_v1"]["evidence_id"] = "unqualified";
    Throws([&] { ParseRackProfile(document, "onboard_rack_v1"); });
  }
  {
    auto document = YAML::Load(kRack);
    document["onboard_rack_profiles"]["onboard_rack_v1"]["unexpected"] = 1;
    Throws([&] { ParseRackProfile(document, "onboard_rack_v1"); });
  }
  Throws([&] { ParseRackProfile(YAML::Load(kRack), "missing_rack"); });

  // The shipped profile is the measured M6 layout: six taught slots in a 3x2
  // grid. This assertion is the guard against an accidental edit.
  const auto shipped = std::filesystem::path(__FILE__).parent_path().parent_path() /
                       "config/onboard_rack_profiles.yaml";
  const auto measured = LoadRackProfile(shipped.string(), "onboard_rack_v1");
  assert(measured.qualified && measured.slots.size() == 6);
  assert(Close(measured.SlotCenter(2).x, 0.329030) && Close(measured.SlotCenter(2).y, 0.007922));
  assert(Close(measured.SlotCenter(6).x, 0.255778) && Close(measured.SlotCenter(6).y, -0.068786));
  assert(Close(measured.ApproachPoint(1).z, 0.071998));
  assert(measured.tool_rpy_deg && Close((*measured.tool_rpy_deg)[0], 180.0));
  // The slow descent touched a neighbouring cap on 2026-11-04; the profile
  // now carries the decided 45 degree tool-axis offset.
  assert(measured.tool_yaw_offset_deg && Close(*measured.tool_yaw_offset_deg, 45.0));

  // Structure can be valid while qualification is still an explicit false.
  OnboardRackProfile unqualified = parsed;
  unqualified.qualified = false;
  unqualified.evidence_id = "unqualified";
  std::string detail;
  assert(Validate(unqualified, &detail));
  assert(!unqualified.qualified);
}
