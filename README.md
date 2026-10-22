# AgileX Mobile Manipulator ROS1 Onboarding Guide

This repository is a ROS1 workspace for an AgileX mobile manipulator: a Ranger mobile base, a lift module, an xArm6 manipulator, a DH gripper, lidar, camera, MoveIt, SLAM, and navigation.

This README is written for someone who is new to ROS1. It starts with the concepts you need in order to understand what is happening, then gives operator commands, then maps the codebase and physical model files.

For the current reusable task runtime, read the [onboard source architecture
index](src/core/WORKFLOW_ARCHITECTURE.md) and [workflow module guide](
src/core/agx_robot_workflow/README.md). They document task configuration,
measured custody, guarded travel and continuation. The vendor bringup/navigation
examples below describe their own launch paths; the workbench application uses
its operation-bound command arbiter. New multi-station physical qualification is
pending and its default travel profile remains disabled.

## Manual Index

### Core Topics

- [How To Read This README](#1-how-to-read-this-readme)
- [ROS1 Mental Model](#2-ros1-mental-model)
- [High-Level System Architecture](#3-high-level-system-architecture)
- [Lidar, Mapping, And Navigation Pipeline](#4-lidar-mapping-and-navigation-pipeline)
- [Arm, Lift, Gripper, And MoveIt Pipeline](#5-arm-lift-gripper-and-moveit-pipeline)
- [Operator Manual](#6-operator-manual)
- [Developer Map](#7-developer-map)
- [Physical Model, Mesh, URDF, And MoveIt Files](#8-physical-model-mesh-urdf-and-moveit-files)
- [Troubleshooting Checklist](#9-troubleshooting-checklist)

### Operator Actions

- [Lift Test](#lift-test)
- [Arm And MoveIt Test](#arm-and-moveit-test)
- [Gripper Test](#gripper-test)
- [Camera / AR Marker Test](#camera--ar-marker-test)
- [Pick Demo](#pick-demo)
- [Rover Base-Only Test](#rover-base-only-test)
- [Lidar Test](#lidar-test)
- [Mapping Test](#mapping-test)
- [Navigation Test](#navigation-test)

### Code And Model

- [Important Launch Files](#important-launch-files)
- [Important Demo Code](#important-demo-code)
- [Combined Mobile Manipulator Description](#combined-mobile-manipulator-description)
- [Base Model And Meshes](#base-model-and-meshes)
- [xArm6 Model And Meshes](#xarm6-model-and-meshes)
- [Gripper Model And Meshes](#gripper-model-and-meshes)
- [Lift Model And Meshes](#lift-model-and-meshes)
- [MoveIt Config](#moveit-config)

## 1. How To Read This README

You will see these labels:

- **From the codebase:** verified by reading files in this workspace.
- **From onboard testing:** observed during our real robot tests.
- **Needs robot verification:** likely correct, but should be checked on the onboard machine.

Important safety note: commands in this README can move real hardware. Keep the robot clear of people, cables, tables, and loose objects before testing the base, lift, arm, or gripper.

## 2. ROS1 Mental Model

### Why `roscore` Always Comes First

**From the codebase and ROS1 design**

`roscore` starts the ROS master. The ROS master is the name service that lets ROS nodes find each other.

Without `roscore`:

- nodes cannot register themselves
- topics cannot be discovered
- services cannot be found
- launch files cannot reliably load and connect a system
- tools like `rostopic list`, `rosservice list`, RViz, and MoveIt will fail or show very little

So the first terminal in almost every ROS1 session is:

```bash
roscore
```

Leave it running.

### The Basic ROS1 Words

**From ROS1 design**

`package`
: A folder that ROS can find. Packages contain launch files, source code, config, meshes, scripts, and dependencies.

`node`
: A running program in ROS. For example, a lidar driver node, a base driver node, a MoveIt node, or an RViz node.

`topic`
: A streaming data channel. Examples: `/scan`, `/odom`, `/cmd_vel`, `/joint_states`.

`service`
: A request/response call. Services are useful for actions that should happen once, such as sending a lift command.

`parameter`
: A value stored in the ROS parameter server. Launch files often load YAML files into parameters.

`TF`
: The transform tree. TF tells ROS where frames are relative to each other, such as `world`, `odom`, `base_link`, `rslidar`, `link6`, and camera frames.

`launch file`
: A ROS XML file that starts multiple nodes and loads parameters together.

`URDF/Xacro`
: Robot model files. URDF describes robot links, joints, visuals, collision geometry, and inertial properties. Xacro is a macro language that generates URDF.

`SRDF`
: MoveIt semantic robot description. It defines planning groups, named poses, end effectors, and allowed collision pairs.

### Why Every Terminal Needs `source`

**From ROS1 design**

Before running ROS commands in a new terminal, source ROS and the workspace:

```bash
source /opt/ros/<your_ros_distro>/setup.bash
source ~/agilex_ws/devel/setup.bash
```

This teaches the terminal where ROS is installed and where this workspace's packages are.

If a package exists but `roslaunch` says it cannot find it, sourcing is one of the first things to check.

## 3. High-Level System Architecture

### Robot Hardware Layers

**From the codebase**

The robot stack contains these major hardware pieces:

- Ranger mobile base
- lift module
- xArm6 manipulator
- DH gripper
- lidar
- wrist camera / AR marker detection

### Software Layers

**From the codebase**

The stack is easiest to understand as layers:

```text
roscore
  |
  v
hardware drivers
  |-- Ranger base driver
  |-- lift motor service
  |-- xArm hardware server
  |-- DH gripper driver
  |-- lidar driver
  |-- camera / marker detector
  |
  v
state, model, and TF
  |-- /joint_states
  |-- /odom
  |-- /scan
  |-- TF tree
  |-- robot_description from URDF/Xacro
  |
  v
higher-level systems
  |-- MoveIt for arm planning and collision checking
  |-- gmapping for SLAM
  |-- AMCL for localization
  |-- move_base for navigation
  |
  v
operator tools and demos
  |-- RViz
  |-- MoveIt UI
  |-- demo_pick / demo_place
```

### Why RViz Is Not The Robot Controller

**From ROS1 design and this codebase**

RViz is a visualization and interaction tool. It can display the robot, TF, lidar scans, maps, costmaps, and MoveIt planning results. It does not directly drive motors by itself.

For arm motion:

```text
RViz MoveIt panel
  -> MoveIt move_group planner
  -> joint trajectory controller
  -> xArm hardware server
  -> real arm motion
```

For navigation:

```text
RViz 2D Nav Goal
  -> /move_base_simple/goal
  -> move_base
  -> /cmd_vel
  -> Ranger base driver
  -> real base motion
```

## 4. Lidar, Mapping, And Navigation Pipeline

This is the part that usually feels mysterious at first. The navigation stack is not one program. It is a chain of perception, localization, planning, and control.

### Full Navigation Pipeline

**From the codebase**

The intended pipeline is:

```text
lidar hardware
  -> lidar driver
  -> point cloud
  -> pointcloud_to_laserscan
  -> /scan
  -> laser odometry
  -> /odom and TF
  -> map / localization
  -> costmaps
  -> global planner and local planner
  -> /cmd_vel
  -> Ranger base driver
```

### Lidar Bring-Up

**From the codebase**

The lidar launch file is:

```bash
roslaunch agilexpro open_lidar.launch
```

File:

```text
src/start/agilexpro/launch/open_lidar.launch
```

It starts:

- `rslidar_sdk`, the lidar driver
- `pointcloud_to_laserscan`, which converts 3D lidar points into a 2D laser scan
- a static TF publisher involving `rslidar` and `dummy_base_link`
- `rf2o_laser_odometry`, which estimates odometry from laser scans

Important topics from this launch:

- `/scan`: 2D laser scan used by mapping and navigation
- `/odom`: laser-odometry estimate from `rf2o_laser_odometry`

### Why `/scan` Matters

**From ROS1 navigation design**

Most classic ROS1 navigation examples use a 2D `sensor_msgs/LaserScan`. The navigation stack uses it to detect obstacles around the robot.

This robot's lidar likely produces point clouds first. The `pointcloud_to_laserscan` node turns that into `/scan`, which is easier for `gmapping`, `amcl`, and `costmap_2d` to consume.

### Why `/odom` Matters

**From ROS1 navigation design**

Odometry is the robot's estimate of how it moved over time. It is not the same thing as global position. It drifts.

In this workspace, `open_lidar.launch` starts `rf2o_laser_odometry`, configured to:

- subscribe to `/scan`
- publish `/odom`
- publish TF
- use `rslidar` as its base frame
- use `odom` as its odom frame

This is why `/odom` appears during lidar/navigation bring-up.

### Base-Only Odometry Versus Navigation Odometry

**From onboard testing**

During base-only tests, we saw the base odometry under:

```text
/ranger_base_node/odom
```

During lidar/navigation tests, `/odom` can exist because `open_lidar.launch` starts laser odometry.

So both observations can be true:

- base-only bring-up: check `/ranger_base_node/odom`
- lidar/navigation bring-up: check `/odom`

Do not assume one topic name works for every bring-up mode.

### Mapping With gmapping

**From the codebase**

Mapping uses:

```bash
roslaunch agilexpro gmapping.launch
```

File:

```text
src/start/agilexpro/launch/gmapping.launch
```

This launch starts `slam_gmapping`. In this workspace it is configured with:

- `odom_frame: odom`
- `base_frame: dummy_base_link`
- `map_frame: map`
- `/scan` as the laser input

Conceptually:

```text
/scan + /odom
  -> gmapping
  -> map
```

You drive the robot while gmapping watches laser scans and odometry. It builds an occupancy grid map.

### Navigation With move_base

**From the codebase**

Navigation uses:

```bash
roslaunch agilexpro navigation_4wd.launch
```

File:

```text
src/start/agilexpro/launch/navigation_4wd.launch
```

It starts:

- `move_base`
- `map_server`
- `amcl`
- RViz
- Ranger base driver through `ranger_mini_v2.launch`

The main planner configuration is loaded from:

```text
src/start/agilexpro/param/4wd/
```

Important files:

- `move_base_params.yaml`
- `global_costmap_params.yaml`
- `local_costmap_params.yaml`
- `costmap_common_params.yaml`
- `base_local_planner_params.yaml`

### What `map_server` Does

**From ROS1 navigation design and this codebase**

`map_server` loads a saved map from:

```text
src/start/agilexpro/maps/map.yaml
```

In this repo, `navigation_4wd.launch` sets the map frame to:

```text
world
```

That is a workspace-specific detail. Many ROS tutorials use `map`; this robot's navigation config uses `world` in several places.

### What AMCL Does

**From ROS1 navigation design and this codebase**

AMCL localizes the robot on a known map. It uses:

- the saved map
- the laser scan
- odometry
- a particle filter

In this workspace, AMCL is configured with:

- `odom_frame_id: odom`
- `base_frame_id: rslidar`
- `global_frame_id: world`

This means you should inspect the TF tree when debugging navigation. The frames are not the clean textbook set of only `map`, `odom`, and `base_link`.

### What Costmaps Do

**From ROS1 navigation design and this codebase**

Costmaps are 2D grids where obstacles are marked as expensive or blocked.

This workspace has:

- a global costmap for long-range planning on the map
- a local costmap for nearby obstacles and short-term motion

The costmap config uses `/scan` as an observation source:

```text
observation_sources: scan
scan topic: scan
```

The robot footprint is configured in:

```text
src/start/agilexpro/param/4wd/costmap_common_params.yaml
```

That footprint is how the navigation stack knows the approximate body size to avoid collisions.

### What move_base Does

**From ROS1 navigation design and this codebase**

`move_base` receives a goal and computes velocity commands.

This repo configures:

- global planner: `global_planner/GlobalPlanner`
- local planner: `dwa_local_planner/DWAPlannerROS`

The global planner finds a path through the map. The local planner chooses short-term velocity commands that follow the path while respecting obstacles and velocity limits.

The final output is:

```text
/cmd_vel
```

That topic is consumed by the base driver to move the robot.

## 5. Arm, Lift, Gripper, And MoveIt Pipeline

### Arm Bring-Up

**From the codebase**

The arm launch command is:

```bash
roslaunch agx_xarm_bringup setup_arm.launch
```

File:

```text
src/start/agx_xarm_bringup/launch/setup_arm.launch
```

This launch includes:

```text
src/moveit_config/ranger_xarm_lifting_moveit_config/launch/realMove_exec.launch
```

`realMove_exec.launch` starts:

- xArm hardware server through `xarm_bringup`
- xArm trajectory controller
- MoveIt common/RViz launch
- real controller remapping for `/follow_joint_trajectory`

Conceptually:

```text
MoveIt plan
  -> trajectory controller
  -> xArm hardware server
  -> real xArm motion
```

### What MoveIt Adds

**From ROS1 MoveIt design and this codebase**

MoveIt is the motion planning layer. It uses:

- URDF/Xacro for robot links, joints, and collision geometry
- SRDF for planning groups and allowed collision pairs
- current joint states
- planning scene
- planning algorithms
- controller configuration

MoveIt helps answer:

- Can the arm reach this target pose?
- Which joint trajectory should get there?
- Does the planned path collide with the robot model or known obstacles?
- Which controller should execute the trajectory?

### Collision Prevention In MoveIt

**From ROS1 MoveIt design and this codebase**

MoveIt collision checking depends on the robot model and planning scene.

The robot description includes visual and collision geometry. The SRDF includes allowed collision pairs. For example, adjacent links often have collisions disabled because they are physically connected or always close.

Important file:

```text
src/moveit_config/ranger_xarm_lifting_moveit_config/config/ranger_with_xarm6.srdf
```

This file defines:

- the `xarm6` planning group
- named poses such as `home` and `initial`
- many disabled collision pairs

Collision checking is only as good as the model, TF, current joint states, and planning scene. If the world contains a table but MoveIt does not know about the table, MoveIt cannot avoid it by magic.

### Is The Lift Integrated As A 7-DOF Arm?

**From the codebase**

Physically, the robot is a lift-mounted mobile manipulator. The arm is mounted on the lift structure in the combined robot model.

But in the current MoveIt config in this repo, the active MoveIt planning group is:

```text
xarm6
```

It contains:

```text
joint1
joint2
joint3
joint4
joint5
joint6
```

The SRDF group chain is:

```text
base_link: link_base
tip_link: link6
```

So the current MoveIt planning group is the 6-joint xArm, not a full 7-DOF lift-plus-arm planning group.

That means:

- the lift is represented in the overall robot model
- the xArm is planned by MoveIt as a 6-DOF arm group
- the lift is controlled separately through its ROS service path
- making lift plus arm into one 7-DOF planning group would require MoveIt config changes, joint state/controller integration, and careful collision/limits setup

### Gripper Bring-Up

**From the codebase**

The gripper launch command is:

```bash
roslaunch agx_xarm_bringup open_gripper.launch
```

File:

```text
src/start/agx_xarm_bringup/launch/open_gripper.launch
```

It includes:

```text
dh_gripper_driver/launch/dh_gripper.launch
```

### Camera And AR Marker Bring-Up

**From the codebase**

The camera launch command is:

```bash
roslaunch agx_xarm_bringup open_camera.launch
```

File:

```text
src/start/agx_xarm_bringup/launch/open_camera.launch
```

It includes `agx_ar_pose.launch` with:

- camera namespace `cam1`
- a specific camera serial number
- a transform from `link6`
- AR marker pose processing
- `mark_listenner`

This is why the pick demo is marker-based. It expects marker frames/poses, not general object recognition.

## 6. Operator Manual

Run these on the onboard Ubuntu machine connected to the robot.

### Common Terminal Setup

**From onboard testing and ROS1 design**

In every new terminal:

```bash
source /opt/ros/<your_ros_distro>/setup.bash
source ~/agilex_ws/devel/setup.bash
```

If the onboard workspace path is different, use the real path.

### Terminal 1: Start ROS Master

```bash
roscore
```

Keep this terminal open.

### Lift Test

**From onboard testing**

Terminal 2:

```bash
roslaunch lifting_ctrl start_motor.launch
```

This starts the lift motor control service.

Terminal 3:

```bash
rosservice call /lifter_1/LiftingMotorService "val: 100
mode: 0"
```

This sends a lift command. The exact meaning and safe range of `val` should be verified on the robot before using large values.

**Needs robot verification:** confirm the safe lift limits, direction convention, and whether `val` means millimeters, encoder units, or a controller-specific position value on the current onboard setup.

### Arm And MoveIt Test

**From onboard testing and the codebase**

Terminal 2:

```bash
roslaunch agx_xarm_bringup setup_arm.launch
```

This starts the xArm hardware path, controller path, MoveIt, and RViz.

Expected behavior:

- the arm connects using the configured robot IP
- RViz / MoveIt UI opens
- MoveIt can show the robot model and planning group

If the MoveIt `MotionPlanning` panel shows a warning such as:

```text
PlanningScene: Requesting initial scene failed
Unable to connect to move_group action server 'move_group' within allotted time
```

keep Terminal 2 running, close only the RViz UI window, then open a new terminal and relaunch RViz:

```bash
roslaunch ranger_xarm_lifting_moveit_config moveit_rviz.launch
```

**From onboard testing:** this can happen when RViz starts before `move_group` is fully ready. The arm backend may still be healthy, so restarting only RViz is often enough.

Useful note:

```text
Package name is agx_xarm_bringup, not agx_arm_bringup.
```

### Gripper Test

**From onboard testing and the codebase**

Terminal 3:

```bash
roslaunch agx_xarm_bringup open_gripper.launch
```

This starts the DH gripper driver.

### Camera / AR Marker Test

**From onboard testing and the codebase**

Terminal 4:

```bash
roslaunch agx_xarm_bringup open_camera.launch
```

This starts the wrist camera and AR marker pose pipeline.

### Pick Demo

**From onboard testing and the codebase**

After arm, gripper, and camera are running:

```bash
rosrun agx_xarm_pick demo_pick
```

The demo code:

- initializes the arm
- moves to a ready/home pose
- opens the gripper
- looks for marker information
- runs a search motion if marker data is missing
- executes the pick mission if a valid target is found

The default target marker in the demo code is marker ID `0`.

To generate marker ID `0`:

```bash
rosrun ar_track_alvar createMarker 0
```

### Rover Base-Only Test

**From onboard testing and the codebase**

Terminal 2:

```bash
roslaunch ranger_bringup ranger_mini_v2.launch
```

This starts the Ranger base driver.

Check odometry:

```bash
rostopic list
rostopic echo /ranger_base_node/odom
```

During our base-only test, `/ranger_base_node/odom` was the useful odometry topic.

Send a very slow forward command:

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist \
'{linear: {x: 0.10, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Stop the robot:

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist \
'{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

Optional CAN setup from the old onboard notes:

```bash
rosrun ranger_bringup setup_can2usb.bash
```

`bringup_can2usb.bash` and `setup_can2usb.bash` are the usual recovery steps when `can0` is missing or stale.

Why you may need this again today:

- the robot or base power was cycled
- the USB-CAN adapter was unplugged/replugged
- the Linux CAN interface was not brought back up automatically

In other words, the CAN interface is not always persistent across sessions, so you often need to re-run the bring-up script before `roslaunch ranger_bringup ranger_mini_v2.launch`.

**Needs robot verification:** use this only if the CAN interface is not already available.

### Lidar Test

**From onboard testing and the codebase**

Terminal 2:

```bash
roslaunch agilexpro open_lidar.launch
```

Check topics:

```bash
rostopic list
rostopic echo /scan
rostopic echo /odom
```

Expected result:

- `/scan` should publish laser scan data
- `/odom` should publish laser odometry from `rf2o_laser_odometry`

Inspect TF:

```bash
rosrun tf view_frames
```

or use RViz TF display.

### Mapping Test

**From onboard testing and the codebase**

Terminal 2:

```bash
roslaunch agilexpro open_lidar.launch
```

Terminal 3:

```bash
roslaunch agilexpro gmapping.launch
```

Drive the robot slowly while the map is built.

Save the map:

```bash
roscd agilexpro/maps
rosrun map_server map_saver -f map
```

This should create or update map files named like:

```text
map.yaml
map.pgm
```

### Navigation Test

**From onboard testing and the codebase**

Terminal 2:

```bash
roslaunch agilexpro open_lidar.launch
```

Terminal 3:

```bash
roslaunch agilexpro navigation_4wd.launch
```

This starts:

- base driver
- map server
- AMCL localization
- move_base
- RViz

In RViz:

- use `2D Pose Estimate` if localization needs an initial pose
- use `2D Nav Goal` to send a navigation goal

Useful checks:

```bash
rostopic echo /odom
rostopic echo /scan
rostopic echo /cmd_vel
rostopic echo /move_base/status
```

If `/cmd_vel` publishes after a goal is sent, `move_base` is trying to command the base.

## 7. Developer Map

### Top-Level Workspace Areas

**From the codebase**

```text
src/start
```

Launch files and bring-up packages used most often by operators.

```text
src/car_base
```

Ranger mobile base packages.

```text
src/core
```

Higher-level application logic, including pick/place demos.

```text
src/moveit_config
```

Robot descriptions, meshes, URDF/Xacro, SRDF, and MoveIt config.

```text
src/sensors
```

Camera, AR marker, lidar, and sensor-related packages.

```text
src/SLAM
```

Additional SLAM/navigation notes and packages.

### Important Launch Files

**From the codebase**

Arm:

```text
src/start/agx_xarm_bringup/launch/setup_arm.launch
src/moveit_config/ranger_xarm_lifting_moveit_config/launch/realMove_exec.launch
```

Gripper:

```text
src/start/agx_xarm_bringup/launch/open_gripper.launch
```

Camera / AR marker:

```text
src/start/agx_xarm_bringup/launch/open_camera.launch
```

Base:

```text
src/car_base/ranger_ros/ranger_bringup/launch/ranger_mini_v2.launch
```

Lidar:

```text
src/start/agilexpro/launch/open_lidar.launch
```

Mapping:

```text
src/start/agilexpro/launch/gmapping.launch
```

Navigation:

```text
src/start/agilexpro/launch/navigation_4wd.launch
```

### Important Demo Code

**From the codebase**

Pick demo entrypoint:

```text
src/core/agx_xarm_pick/demo/demo_pick.cpp
```

Pick logic:

```text
src/core/agx_xarm_pick/src/agx_xarm_pick_node.cpp
```

Useful functions to inspect:

- `Setup`
- `SetHomePose`
- `OpenGripper`
- `SearchMaker`
- `ExecuteMission`

## 8. Physical Model, Mesh, URDF, And MoveIt Files

### Combined Mobile Manipulator Description

**From the codebase**

Main combined robot description package:

```text
src/moveit_config/ranger_xarm/ranger_xarm_description/
```

Important combined Xacro files:

```text
src/moveit_config/ranger_xarm/ranger_xarm_description/ranger_xarm6.xacro
src/moveit_config/ranger_xarm/ranger_xarm_description/ranger_xarm6_lifting.xacro
src/moveit_config/ranger_xarm/ranger_xarm_description/ranger_xarm6_lifting_2431.xacro
```

`ranger_xarm6_lifting.xacro` includes:

- Ranger mini model
- xArm6 robot model
- lift/base adapter model
- DH AG95 gripper model

### Base Model And Meshes

**From the codebase**

```text
src/moveit_config/ranger_mini_v3/
src/moveit_config/ranger_mini_v3_gazebo/
src/moveit_config/ranger_xarm/ranger_xarm_base/
```

Look in:

```text
urdf/
meshes/
xacro/
```

### xArm6 Model And Meshes

**From the codebase**

```text
src/moveit_config/ranger_xarm/ranger_xarm_description/xarm6_urdf/
src/moveit_config/ranger_xarm/ranger_xarm_description/xarm6_meshes/
```

### Gripper Model And Meshes

**From the codebase**

```text
src/moveit_config/dh_robotics_ag95_model/
```

Look in:

```text
urdf/
meshes/
```

### Lift Model And Meshes

**From the codebase**

```text
src/moveit_config/c1997/
```

Look in:

```text
urdf/
meshes/
```

### MoveIt Config

**From the codebase**

Main MoveIt config for the lift-mounted xArm setup:

```text
src/moveit_config/ranger_xarm_lifting_moveit_config/
```

Important files:

```text
config/ranger_with_xarm6.srdf
config/kinematics.yaml
config/joint_limits.yaml
config/ompl_planning.yaml
config/controllers.yaml
config/xarm6_traj_controller.yaml
launch/realMove_exec.launch
launch/move_group.launch
launch/moveit_rviz.launch
```

The SRDF is where you confirm the current planning group:

```text
group name: xarm6
joints: joint1 to joint6
chain: link_base to link6
```

## 9. Troubleshooting Checklist

### If A Package Cannot Be Found

Check:

```bash
source /opt/ros/<your_ros_distro>/setup.bash
source ~/agilex_ws/devel/setup.bash
rospack find agx_xarm_bringup
```

### If Topics Are Missing

Check:

```bash
roscore
rostopic list
rosnode list
```

Then confirm the correct launch file is running.

### If Navigation Does Not Move

Check:

```bash
rostopic echo /scan
rostopic echo /odom
rostopic echo /move_base/status
rostopic echo /cmd_vel
```

Then inspect RViz:

- does the map appear?
- does the laser scan appear?
- does the robot pose appear in the right place?
- are costmaps showing obstacles?
- did you provide an initial pose for AMCL?

### If MoveIt Cannot Plan

Check:

```bash
rostopic echo /joint_states
rosnode list
```

In RViz:

- confirm the robot model is visible
- confirm the planning group is `xarm6`
- check for red collision state
- check whether the target pose is reachable

### If TF Looks Wrong

Use:

```bash
rosrun tf view_frames
```

or RViz TF display.

Pay special attention to these frames in this workspace:

- `world`
- `odom`
- `dummy_base_link`
- `rslidar`
- `base_link`
- `link_base`
- `link6`

This workspace does not use only the textbook `map -> odom -> base_link` frame names, so debugging TF by inspection is important.
