#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>

#include "agx_place_tasks/onboard_rack_profile.hpp"

namespace agx_place_tasks {

// A pose without a ROS dependency, so the placement geometry can be unit
// tested without a ROS graph.
struct PlacementPose {
  Point3 position;
  std::array<double, 4> orientation_xyzw{{0.0, 0.0, 0.0, 1.0}};
};

// RPY (sxyz convention, degrees) to quaternion xyzw. The taught record stores
// the tool attitude in exactly this convention.
std::array<double, 4> QuaternionFromRpyDeg(const std::array<double, 3>& rpy_deg);

// Slot centre with the taught tool attitude. Throws std::invalid_argument when
// the profile has no recorded attitude: a placement orientation is never
// invented from the rack axes.
// Rotate a tool attitude about its own Z axis (the insertion direction).
std::array<double, 4> RotateAboutToolZ(const std::array<double, 4>& quaternion,
                                       double radians);

PlacementPose SlotPose(const OnboardRackProfile& rack, std::size_t slot);

// Same attitude, at the approach point one rise above the slot centre.
PlacementPose ApproachPose(const OnboardRackProfile& rack, std::size_t slot);

}  // namespace agx_place_tasks
