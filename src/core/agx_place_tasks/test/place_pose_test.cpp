#include "agx_place_tasks/place_pose.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <string>

using namespace agx_place_tasks;

namespace {

bool Close(double a, double b, double tolerance = 1e-9) {
  return std::fabs(a - b) <= tolerance;
}

OnboardRackProfile Rack(bool with_attitude = true) {
  OnboardRackProfile rack;
  rack.profile_id = "onboard_rack_v1";
  rack.parent_frame = "link_base_lift0";
  rack.tip_link = "tcp_link";
  rack.descend_axis = {0.0, 0.0, -1.0};
  rack.approach_rise_m = 0.150;
  if (with_attitude) rack.tool_rpy_deg = std::array<double, 3>{{180.0, 0.0, 1.32}};
  rack.slots.push_back(RackSlot{1, "a", {0.327302, 0.082902, -0.078002}});
  rack.slots.push_back(RackSlot{2, "b", {0.329030, 0.007922, -0.078002}});
  rack.qualified = true;
  rack.evidence_id = "fixture";
  return rack;
}

void TestIdentity() {
  const auto q = QuaternionFromRpyDeg({{0.0, 0.0, 0.0}});
  assert(Close(q[0], 0.0) && Close(q[1], 0.0) && Close(q[2], 0.0) && Close(q[3], 1.0));
}

// The taught attitude is tool +Z pointing along link_base_lift0 -Z, i.e. a
// 180 degree rotation about X. With no yaw that is exactly (1, 0, 0, 0).
void TestTaughtAttitude() {
  const auto q = QuaternionFromRpyDeg({{180.0, 0.0, 0.0}});
  assert(Close(std::fabs(q[0]), 1.0) && Close(q[1], 0.0) && Close(q[2], 0.0) && Close(q[3], 0.0));
  const auto taught = QuaternionFromRpyDeg({{180.0, 0.0, 1.32}});
  const double norm = std::sqrt(taught[0] * taught[0] + taught[1] * taught[1] +
                                taught[2] * taught[2] + taught[3] * taught[3]);
  assert(Close(norm, 1.0, 1e-12));
  // M6 records this attitude as xyzw = (0.999934, 0.011519, 0, 0): with a 180
  // degree roll the 1.32 degree yaw lands in y, not z. Matching the recorded
  // quaternion is the check that the convention is the robot's own.
  assert(Close(std::fabs(taught[0]), 0.999934, 1e-6));
  assert(Close(std::fabs(taught[1]), 0.011519, 1e-6));
  assert(Close(taught[2], 0.0, 1e-9));
  assert(Close(taught[3], 0.0, 1e-9));
}

void TestPoses() {
  const auto rack = Rack();
  const auto slot = SlotPose(rack, 2);
  assert(Close(slot.position.x, 0.329030) && Close(slot.position.y, 0.007922));
  const auto approach = ApproachPose(rack, 2);
  assert(Close(approach.position.z, 0.071998));
  // The attitude is identical at both points; only the position changes.
  for (int i = 0; i < 4; ++i) assert(Close(slot.orientation_xyzw[i], approach.orientation_xyzw[i]));

  bool thrown = false;
  try { SlotPose(rack, 9); } catch (const std::out_of_range&) { thrown = true; }
  assert(thrown);

  // The descent yaw offset rotates the tool about its own Z axis. Two unit
  // quaternions that differ by an angle t have dot product cos(t/2).
  auto rotated = rack;
  rotated.tool_yaw_offset_deg = 45.0;
  const auto offset_slot = SlotPose(rotated, 2);
  double product = 0.0;
  for (int i = 0; i < 4; ++i) product += slot.orientation_xyzw[i] * offset_slot.orientation_xyzw[i];
  assert(Close(product, std::cos(M_PI / 8.0), 1e-9));
  // The descent direction is unchanged: a tool twist must not tilt the tool.
  const auto tool_z = [](const std::array<double, 4>& q) {
    return std::array<double, 3>{{2.0 * (q[0] * q[2] + q[3] * q[1]),
                                  2.0 * (q[1] * q[2] - q[3] * q[0]),
                                  1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1])}};
  };
  const auto before_twist = tool_z(slot.orientation_xyzw);
  const auto after_twist = tool_z(offset_slot.orientation_xyzw);
  for (int i = 0; i < 3; ++i) assert(Close(before_twist[i], after_twist[i], 1e-9));
  assert(Close(after_twist[2], -1.0, 1e-9));

  // Without a recorded attitude the orientation is refused, not invented.
  const auto no_attitude = Rack(false);
  thrown = false;
  try { SlotPose(no_attitude, 1); } catch (const std::invalid_argument&) { thrown = true; }
  assert(thrown);
}

}  // namespace

int main() {
  TestIdentity();
  TestTaughtAttitude();
  TestPoses();
}
