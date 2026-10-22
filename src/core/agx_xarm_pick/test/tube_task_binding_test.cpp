#include "agx_xarm_pick/resolved_tube_config.hpp"
#include <cassert>
#include <stdexcept>

int main(int argc, char** argv) {
  assert(argc == 2);
  const auto frozen = agx_xarm_pick::ResolveTubeTaskConfig(argv[1]);
  const auto bound = agx_xarm_pick::BindTubeTaskMotion(*frozen, "green");
  assert(bound.target_cap_color == "green");
  assert(bound.profile_target_cap_color == "green");
  assert(bound.target_hsv_lower == frozen->behavior.color_classifiers.at("green").hsv_lower);
  assert(frozen->motion.target_cap_color.empty());
  assert(bound.phase_one_joint_positions_rad == frozen->motion.phase_one_joint_positions_rad);
  assert(bound.link6_to_grasp_tcp.rotation.w == frozen->motion.link6_to_grasp_tcp.rotation.w);
  auto rejected = [](auto&& callback) {
    try { callback(); } catch (const std::invalid_argument&) { return true; }
    return false;
  };
  assert(rejected([&] { agx_xarm_pick::BindTubeTaskMotion(*frozen, "unknown-color"); }));
  auto tagged = *frozen;
  tagged.motion.tag_registration_required = true;
  assert(rejected([&] { agx_xarm_pick::BindTubeTaskMotion(tagged, "green"); }));
  auto wrong_frame = *frozen;
  wrong_frame.motion.planning_frame = "map";
  assert(rejected([&] { agx_xarm_pick::BindTubeTaskMotion(wrong_frame, "green"); }));
  auto tilted = *frozen;
  tilted.installation.measured_transform.rotation.x = 0.1;
  assert(rejected([&] { agx_xarm_pick::BindTubeTaskMotion(tilted, "green"); }));
}
