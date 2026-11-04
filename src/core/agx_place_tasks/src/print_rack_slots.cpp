#include <iomanip>
#include <iostream>
#include <stdexcept>

#include "agx_place_tasks/onboard_rack_profile.hpp"

// Prints the taught slot poses of one rack profile as JSON so that planning
// tools do not re-implement (and drift from) the C++ profile validation.
int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: print_rack_slots <profiles.yaml> <profile_id>\n";
    return 2;
  }
  try {
    const auto rack = agx_place_tasks::LoadRackProfile(argv[1], argv[2]);
    std::cout << std::setprecision(9);
    std::cout << "{\"profile_id\":\"" << rack.profile_id << "\",\"parent_frame\":\""
              << rack.parent_frame << "\",\"tip_link\":\"" << rack.tip_link << "\",\"qualified\":" << (rack.qualified ? "true" : "false")
              << ",\"evidence_id\":\"" << rack.evidence_id << "\",\"approach_rise_m\":"
              << rack.approach_rise_m << ",\"descend_axis\":[" << rack.descend_axis.x << ","
              << rack.descend_axis.y << "," << rack.descend_axis.z << "],\"tool_rpy_deg\":";
    if (rack.tool_rpy_deg)
      std::cout << "[" << (*rack.tool_rpy_deg)[0] << "," << (*rack.tool_rpy_deg)[1] << ","
                << (*rack.tool_rpy_deg)[2] << "]";
    else
      std::cout << "null";
    std::cout << ",\"tool_yaw_offset_deg\":";
    if (rack.tool_yaw_offset_deg) std::cout << *rack.tool_yaw_offset_deg;
    else std::cout << "null";
    std::cout << ",\"slot_tolerance_m\":";
    if (rack.slot_tolerance_m) std::cout << *rack.slot_tolerance_m;
    else std::cout << "null";
    std::cout << ",\"slots\":[";
    bool first = true;
    for (const auto& entry : rack.slots) {
      const auto approach = rack.ApproachPoint(entry.slot);
      if (!first) std::cout << ",";
      first = false;
      std::cout << "{\"slot\":" << entry.slot << ",\"name\":\"" << entry.name
                << "\",\"center\":[" << entry.center_m.x << "," << entry.center_m.y << ","
                << entry.center_m.z << "],\"approach\":[" << approach.x << "," << approach.y << ","
                << approach.z << "]}";
    }
    std::cout << "]}\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
  return 0;
}
