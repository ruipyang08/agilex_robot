#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>

#include "agx_place_tasks/onboard_rack_profile.hpp"
#include "agx_place_tasks/place_execution.hpp"

namespace agx_place_tasks {

struct PlaceMotionConfig {
  std::string group = "xarm6";
  double planning_time_s = 5.0;
  // Slow by default: the descent ends a few hundredths of a radian away from
  // the wrist singularity, so tracking errors there are amplified.
  double velocity_scaling = 0.10;
  double acceleration_scaling = 0.10;
  double eef_step_m = 0.005;
  double cartesian_jump_threshold = 0.0;
  std::string gripper_command_topic = "/gripper/ctrl";
  std::string gripper_state_topic = "/gripper/states";
  std::string joint_state_topic = "/joint_states";
  double gripper_open_position = 1000.0;
  double gripper_open_tolerance = 100.0;
  double gripper_force = 50.0;
  double gripper_speed = 50.0;
  double gripper_timeout_s = 4.0;
  // Narrow the fingers before the descent. At the fully open 95 mm the pads
  // sweep the neighbouring caps on a 75 mm grid; 750 counts is about 75 mm on
  // the AG95 (0 = closed, 1000 = fully open). Only ever closes: a gripper that
  // is already narrower (holding a tube) is left alone.
  bool narrow_before_descent = true;
  double descent_open_position = 750.0;
  double descent_open_tolerance = 30.0;
  double descent_narrow_timeout_s = 3.0;
  // The caller knows from measured custody whether a tube is held. When it is
  // false the release step is skipped instead of reporting an empty gripper
  // opening as a released object.
  bool expects_held_object = true;
  // Pick sequence: command a gap clearly narrower than the tube, then judge by
  // how far the fingers stop ABOVE that command. An empty gripper reaches the
  // command (margin ~0); a 35 mm cap stops tens of counts higher. Comparing
  // against the command instead of an absolute floor keeps the test valid when
  // the commanded gap changes.
  // Default for the taught-slot path, which grips the 20 mm tube body:
  // the command must stay clearly below that width or the margin is zero.
  double grasp_close_position = 100.0;
  double grasp_held_margin = 60.0;
  double grasp_timeout_s = 4.0;
  double settle_velocity_rad_s = 0.02;
  double settle_timeout_s = 3.0;
  // Below holds the arm at the approach point. Returning to a named joint
  // target is opt-in because it is a free-space move the caller must validate.
  bool return_to_start = false;
  std::string start_named_target = "initial";
  // Optional independent slot-occupancy measurement. Empty means no sensor
  // exists, and the caller must not claim a verified placement.
  std::function<std::optional<bool>()> slot_occupancy_confirmed;
};

// approach (planned) -> straight descent -> release -> straight retreat -> settle.
// Never claims more than it measured: no occupancy sensor means no verified
// placement, and a release that cannot be confirmed is reported as uncertain.
PlaceMotionOutcome RunPlaceMotion(const OnboardRackProfile& rack, std::size_t slot,
                                  const PlaceMotionConfig& config,
                                  const std::function<bool()>& stop_requested);


struct PickMotionOutcome {
  bool grasped = false;
  bool grasp_uncertain = false;
  bool measured_stop_confirmed = false;
  bool worker_joined = false;
  std::string detail;
  PlaceEvidence evidence;
};

// An explicit pair of poses, used when the target comes from a measured
// position table instead of a taught rack slot.
struct PickTarget {
  std::string frame;
  std::string tip_link;
  Point3 approach_m;
  Point3 grasp_m;
  // Orientation of the free-space approach. When it differs from
  // `orientation_xyzw` the tool twists in place above the target before the
  // straight descent, so the widened pads travel without a yaw offset.
  std::array<double, 4> approach_orientation_xyzw{{1.0, 0.0, 0.0, 0.0}};
  std::array<double, 4> orientation_xyzw{{1.0, 0.0, 0.0, 0.0}};
};

PickMotionOutcome RunPickMotionAt(const PickTarget& target, const PlaceMotionConfig& config,
                                  const std::function<bool()>& stop_requested);
// Same release sequence, but with the poses given explicitly (measured
// positions instead of a taught slot). Reuses PickTarget because the shape is
// identical: an approach pose, a lower pose and the two orientations.
PlaceMotionOutcome RunPlaceMotionAt(const PickTarget& target, const PlaceMotionConfig& config,
                                    const std::function<bool()>& stop_requested);

// approach -> narrow -> straight descent -> close and measure -> straight retreat.
// The grasp is only reported when the fingers stopped on something.
PickMotionOutcome RunPickMotion(const OnboardRackProfile& rack, std::size_t slot,
                                const PlaceMotionConfig& config,
                                const std::function<bool()>& stop_requested);

}  // namespace agx_place_tasks
