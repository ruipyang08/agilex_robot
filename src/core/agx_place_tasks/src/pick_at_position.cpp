#include <cstdlib>
#include <iostream>
#include <string>

#include <ros/ros.h>

#include "agx_place_tasks/onboard_positions.hpp"
#include "agx_place_tasks/place_motion.hpp"
#include "agx_place_tasks/place_pose.hpp"

// Single-pick check for one measured position, driving the same motion the
// workflow host uses (RunPickMotionAt). It exists so the position path can be
// exercised without a preceding navigation task. It drives the arm only and
// does not touch robot custody, so it cannot be used in a recipe.
int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_at_position");
  ros::NodeHandle private_node("~");

  std::string positions_file;
  int position = 0;
  bool execute = false, release_back = false;
  private_node.param<std::string>("positions_file", positions_file, "");
  private_node.param("position", position, 0);
  private_node.param("execute", execute, false);
  // Put the tube straight back into the same hole, so a verification run
  // leaves the rack as it found it.
  private_node.param("release_back", release_back, false);
  if (positions_file.empty() || position < 1) {
    std::cerr << "usage: pick_at_position _positions_file:=<yaml> _position:=<n> _execute:=true"
              << std::endl;
    return 2;
  }

  agx_place_tasks::PositionTable table;
  try {
    table = agx_place_tasks::LoadPositions(positions_file);
  } catch (const std::exception& error) {
    std::cerr << "cannot load position table: " << error.what() << std::endl;
    return 2;
  }
  const auto* target_entry = table.Find(std::to_string(position));
  if (target_entry == nullptr) {
    std::cerr << "position " << position << " is not in the table" << std::endl;
    return 2;
  }

  agx_place_tasks::PlaceMotionConfig config;
  config.velocity_scaling = table.velocity_scaling;
  config.acceleration_scaling = table.acceleration_scaling;
  config.eef_step_m = table.eef_step_m;
  config.descent_open_position = table.open_before_descent;
  config.gripper_open_position = table.open_before_descent;
  config.grasp_close_position = table.close_on_cap;
  config.grasp_held_margin = table.held_margin;
  config.gripper_force = table.gripper_force;
  config.gripper_speed = table.gripper_speed;

  agx_place_tasks::PickTarget target;
  target.frame = table.frame;
  target.tip_link = "tcp_link";
  target.approach_m = {target_entry->x, target_entry->y, target_entry->approach_z};
  target.grasp_m = {target_entry->x, target_entry->y, target_entry->grasp_z};
  // Travel at the taught yaw, twist only above the target.
  target.approach_orientation_xyzw = agx_place_tasks::QuaternionFromRpyDeg(table.tool_rpy_deg);
  target.orientation_xyzw = agx_place_tasks::RotateAboutToolZ(
      agx_place_tasks::QuaternionFromRpyDeg(table.tool_rpy_deg),
      table.tool_yaw_offset_deg * M_PI / 180.0);

  ROS_INFO("pick_at_position: %d at (%.4f, %.4f), approach %.3f, grasp %.3f, close %.0f",
           position, target_entry->x, target_entry->y, target_entry->approach_z,
           target_entry->grasp_z, table.close_on_cap);
  if (!execute) {
    std::cout << "{\"execute\":false}" << std::endl;
    std::cerr << "refusing to move: pass _execute:=true to run the sequence" << std::endl;
    return 2;
  }

  ros::AsyncSpinner spinner(1);
  spinner.start();
  const auto outcome = agx_place_tasks::RunPickMotionAt(target, config, [] { return !ros::ok(); });

  bool released = false;
  if (outcome.grasped && release_back) {
    agx_place_tasks::PickTarget back = target;
    back.approach_m = target.approach_m;
    back.grasp_m = {target_entry->x, target_entry->y, target_entry->place_z};
    // Already at the twisted yaw; keep it so the release needs no extra turn.
    back.approach_orientation_xyzw = target.orientation_xyzw;
    const auto place = agx_place_tasks::RunPlaceMotionAt(back, config, [] { return !ros::ok(); });
    released = place.released;
    std::cout << "{\"released\":" << (place.released ? "true" : "false")
              << ",\"release_uncertain\":" << (place.release_uncertain ? "true" : "false")
              << ",\"place_detail\":\"" << place.detail << "\"}" << std::endl;
  }

  std::cout << "{\"grasped\":" << (outcome.grasped ? "true" : "false")
            << ",\"grasp_uncertain\":" << (outcome.grasp_uncertain ? "true" : "false")
            << ",\"stop_confirmed\":" << (outcome.measured_stop_confirmed ? "true" : "false")
            << ",\"detail\":\"" << outcome.detail << "\"}" << std::endl;
  if (release_back && outcome.grasped && !released) return 1;
  return (outcome.grasped && outcome.measured_stop_confirmed) ? 0 : 1;
}
