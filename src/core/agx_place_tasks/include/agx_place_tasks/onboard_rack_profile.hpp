#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace agx_place_tasks {

struct Point3 {
  double x = 0.0, y = 0.0, z = 0.0;
};

// One taught slot. The slot number is the same number used by the recipe
// ("slot: 3") and by the teaching record, so the two can never drift apart.
struct RackSlot {
  std::size_t slot = 0;
  std::string name;
  Point3 center_m;
};

// Measured geometry of the rack mounted on the robot itself. The external rack
// is recognised with vision because its pose changes with every stop; this rack
// does not move relative to the arm, so it is addressed by taught poses and
// never by recognition or visual alignment.
//
// Slot centres are listed explicitly instead of being derived from a pitch and
// an axis: the taught layout is a 3x2 grid, so a linear model would not describe
// it. Nothing here is inferred from the external rack.
struct OnboardRackProfile {
  std::string profile_id;
  std::string parent_frame;
  // Link the taught centres refer to (the M6 record used tcp_link).
  std::string tip_link;
  std::vector<RackSlot> slots;
  // Unit vector the closed fingers travel along while lowering a tube.
  Point3 descend_axis{0.0, 0.0, -1.0};
  // Straight-line distance between the slot centre and the approach point.
  double approach_rise_m = 0.0;
  // Tool orientation recorded together with the taught points (sxyz, degrees).
  // Absent means the profile was taught without a recorded attitude.
  std::optional<std::array<double, 3>> tool_rpy_deg;
  // Extra rotation about the tool axis used for the descent, in degrees.
  // The taught attitude alone lets the AG95 fingers sweep an adjacent cap,
  // so the descent is performed with this offset. Absent means zero.
  std::optional<double> tool_yaw_offset_deg;
  // Measured repeatability of one taught slot, when somebody measured it.
  std::optional<double> slot_tolerance_m;
  // A rack is usable only with a physical measurement behind it.
  bool qualified = false;
  std::string evidence_id;

  const RackSlot* Find(std::size_t slot) const;
  // Throws std::out_of_range for a slot that was never taught.
  Point3 SlotCenter(std::size_t slot) const;
  Point3 ApproachPoint(std::size_t slot) const;
};

// Structural validation only. Qualification is a separate explicit claim and is
// never inferred from well-formed numbers.
bool Validate(const OnboardRackProfile& profile, std::string* detail);

OnboardRackProfile ParseRackProfile(const YAML::Node& document, const std::string& profile_id);
OnboardRackProfile LoadRackProfile(const std::string& path, const std::string& profile_id);

}  // namespace agx_place_tasks
