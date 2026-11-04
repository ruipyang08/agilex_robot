#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include <ros/ros.h>

#include "agx_place_tasks/onboard_rack_profile.hpp"
#include "agx_place_tasks/place_motion.hpp"

// Single-slot motion check for the onboard rack. It drives the arm through the
// placement sequence only: it does not touch robot custody, and it reports a
// refusal instead of inventing a placement. Use it to validate one slot at low
// speed before the workflow is wired end to end.
int main(int argc, char** argv) {
  ros::init(argc, argv, "place_one_slot");
  ros::NodeHandle private_node("~");

  std::string rack_file, rack_id;
  int slot = 0;
  double velocity = 0.05, acceleration = 0.05, eef_step = 0.005;
  bool execute = false;
  bool expects_object = false;
  double descent_width = 750.0;
  private_node.param<std::string>("rack_file", rack_file, "");
  private_node.param<std::string>("rack_id", rack_id, "");
  private_node.param("slot", slot, 0);
  private_node.param("velocity", velocity, velocity);
  private_node.param("acceleration", acceleration, acceleration);
  private_node.param("eef_step", eef_step, eef_step);
  private_node.param("execute", execute, execute);
  private_node.param("expects_object", expects_object, expects_object);
  private_node.param("descent_width", descent_width, descent_width);
  if (rack_file.empty() || rack_id.empty() || slot < 1) {
    std::cerr << "usage: place_one_slot _rack_file:=<yaml> _rack_id:=<id> _slot:=<n> _execute:=true"
              << std::endl;
    return 2;
  }
  if (velocity <= 0.0 || velocity > 0.5 || acceleration <= 0.0 || acceleration > 0.5) {
    std::cerr << "velocity/acceleration must be inside (0, 0.5]" << std::endl;
    return 2;
  }
  if (!execute) {
    std::cerr << "refusing to move: pass _execute:=true to run the sequence" << std::endl;
    return 2;
  }

  // MoveGroupInterface needs a live callback queue for its action clients.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  agx_place_tasks::OnboardRackProfile rack;
  try {
    rack = agx_place_tasks::LoadRackProfile(rack_file, rack_id);
  } catch (const std::exception& error) {
    std::cerr << "cannot load rack profile: " << error.what() << std::endl;
    return 2;
  }
  if (!rack.Find(static_cast<std::size_t>(slot))) {
    std::cerr << "slot " << slot << " was never taught" << std::endl;
    return 2;
  }

  agx_place_tasks::PlaceMotionConfig config;
  config.velocity_scaling = velocity;
  config.acceleration_scaling = acceleration;
  config.eef_step_m = eef_step;
  config.expects_held_object = expects_object;
  config.descent_open_position = descent_width;
  ROS_INFO("place_one_slot: slot %d, %.2f speed scaling, %s rack", slot, velocity,
           rack.qualified ? "qualified" : "UNQUALIFIED");

  const auto outcome = agx_place_tasks::RunPlaceMotion(
      rack, static_cast<std::size_t>(slot), config, [] { return !ros::ok(); });

  std::cout << "{\"released\":" << (outcome.released ? "true" : "false")
            << ",\"release_uncertain\":" << (outcome.release_uncertain ? "true" : "false")
            << ",\"slot_occupancy_measured\":" << (outcome.slot_occupancy_measured ? "true" : "false")
            << ",\"slot_occupancy_confirmed\":" << (outcome.slot_occupancy_confirmed ? "true" : "false")
            << ",\"stop_confirmed\":" << (outcome.measured_stop_confirmed ? "true" : "false")
            << ",\"worker_joined\":" << (outcome.worker_joined ? "true" : "false")
            << ",\"detail\":\"" << outcome.detail << "\"}" << std::endl;
  return (outcome.worker_joined && outcome.measured_stop_confirmed) ? 0 : 1;
}
