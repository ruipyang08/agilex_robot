#include "agx_place_tasks/place_pose.hpp"

#include <cmath>

namespace agx_place_tasks {
namespace {

std::array<double, 4> MultiplyQuaternion(const std::array<double, 4>& a,
                                         const std::array<double, 4>& b) {
  return {{a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
           a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
           a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
           a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]}};
}

}  // namespace

std::array<double, 4> QuaternionFromRpyDeg(const std::array<double, 3>& rpy_deg) {
  const double half_roll = rpy_deg[0] * M_PI / 360.0;
  const double half_pitch = rpy_deg[1] * M_PI / 360.0;
  const double half_yaw = rpy_deg[2] * M_PI / 360.0;
  const std::array<double, 4> roll{{std::sin(half_roll), 0.0, 0.0, std::cos(half_roll)}};
  const std::array<double, 4> pitch{{0.0, std::sin(half_pitch), 0.0, std::cos(half_pitch)}};
  const std::array<double, 4> yaw{{0.0, 0.0, std::sin(half_yaw), std::cos(half_yaw)}};
  return MultiplyQuaternion(MultiplyQuaternion(yaw, pitch), roll);
}

std::array<double, 4> RotateAboutToolZ(const std::array<double, 4>& quaternion,
                                       double radians) {
  const double half = radians / 2.0;
  const std::array<double, 4> twist{{0.0, 0.0, std::sin(half), std::cos(half)}};
  return MultiplyQuaternion(quaternion, twist);
}

PlacementPose SlotPose(const OnboardRackProfile& rack, std::size_t slot) {
  if (!rack.tool_rpy_deg)
    throw std::invalid_argument("rack profile has no tool_rpy_deg; the placement attitude "
                                "was never recorded");
  PlacementPose pose;
  pose.position = rack.SlotCenter(slot);
  pose.orientation_xyzw = QuaternionFromRpyDeg(*rack.tool_rpy_deg);
  // The taught attitude alone sweeps an adjacent cap on this rack; the descent
  // is deliberately rotated about the tool axis.
  if (rack.tool_yaw_offset_deg && *rack.tool_yaw_offset_deg != 0.0)
    pose.orientation_xyzw = RotateAboutToolZ(pose.orientation_xyzw,
                                            *rack.tool_yaw_offset_deg * M_PI / 180.0);
  return pose;
}

PlacementPose ApproachPose(const OnboardRackProfile& rack, std::size_t slot) {
  PlacementPose pose = SlotPose(rack, slot);
  pose.position = rack.ApproachPoint(slot);
  return pose;
}

}  // namespace agx_place_tasks
