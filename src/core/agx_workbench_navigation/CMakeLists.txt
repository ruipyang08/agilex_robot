cmake_minimum_required(VERSION 3.0.2)
project(agx_workbench_navigation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  angles
  ar_track_alvar
  ar_track_alvar_msgs
  costmap_2d
  geometry_msgs
  message_generation
  message_filters
  move_base_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)

add_message_files(
  FILES
  BaseAuthorityState.msg
  AuthorizedBaseCommand.msg
  CoarseNavigationState.msg
  LidarSafetyState.msg
  TravelReadiness.msg
  PlannedMotionSample.msg
  PlannedMotionSegment.msg
  NavigationRoute.msg
  LidarObstacleSnapshot.msg
)

add_service_files(
  FILES
  SetBaseAuthority.srv
  BindBaseCommandSource.srv
  AcquireBaseHold.srv
  PrepareBaseDeparture.srv
  ReleaseBaseHold.srv
  ResetBaseFault.srv
  PlanNavigationRoute.srv
  PrepareNavigationRoute.srv
  ManageNavigationRoute.srv
)

add_action_files(
  FILES
  NavigateToWorkbench.action
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ranger_local_planner
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  angles
  ar_track_alvar
  ar_track_alvar_msgs
  costmap_2d
  geometry_msgs
  message_runtime
  message_filters
  move_base_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Bounded motion search needs optimized geometry checks. Respect explicitly
# selected Debug/Release builds; avoid an unoptimized default Catkin build.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

add_library(ranger_local_planner
  src/motion_command_sweep.cpp
  src/motion_route_follower.cpp
  src/motion_sensor_evidence.cpp
  src/motion_evidence_ros.cpp
  src/motion_policy_io.cpp
  src/motion_geometry.cpp
  src/motion_route.cpp
  src/motion_speed_profile.cpp
  src/motion_search.cpp
  src/motion_route_store.cpp
  src/motion_preflight.cpp
  src/motion_departure.cpp
  src/motion_plan_service.cpp
  src/ranger_control.cpp
  src/ranger_global_planner.cpp
  src/ranger_local_planner.cpp
)
add_dependencies(ranger_local_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ranger_local_planner
  ${catkin_LIBRARIES}
  yaml-cpp
  OpenSSL::Crypto
)

add_executable(motion_preview_cli src/motion_preview_cli.cpp src/motion_geometry.cpp
  src/motion_route.cpp src/motion_speed_profile.cpp src/motion_search.cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_executable(motion_sensor_evidence_test test/motion_sensor_evidence_test.cpp)
  target_compile_options(motion_sensor_evidence_test PRIVATE -UNDEBUG)
  target_link_libraries(motion_sensor_evidence_test ranger_local_planner)
  add_dependencies(tests motion_sensor_evidence_test)
  add_test(NAME motion_sensor_evidence_contract COMMAND motion_sensor_evidence_test)
  add_rostest_gtest(motion_evidence_ros test/motion_evidence_ros.test test/motion_evidence_ros.cpp)
  if(TARGET motion_evidence_ros)
    add_dependencies(motion_evidence_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(motion_evidence_ros ranger_local_planner ${catkin_LIBRARIES})
  endif()
  add_executable(motion_policy_io_test test/motion_policy_io_test.cpp)
  target_compile_options(motion_policy_io_test PRIVATE -UNDEBUG)
  target_link_libraries(motion_policy_io_test ranger_local_planner)
  add_dependencies(tests motion_policy_io_test)
  add_test(NAME motion_policy_io_contract COMMAND motion_policy_io_test
    ${CMAKE_CURRENT_SOURCE_DIR}/config/navigation_motion_policy.yaml)
  add_dependencies(tests motion_preview_cli)
  foreach(unit motion_geometry motion_route motion_speed_profile motion_search motion_route_store motion_preflight motion_route_follower motion_command_sweep motion_departure)
    add_executable(${unit}_test test/${unit}_test.cpp src/motion_geometry.cpp src/motion_route.cpp src/motion_speed_profile.cpp src/motion_search.cpp src/motion_route_store.cpp src/motion_preflight.cpp src/motion_departure.cpp src/motion_route_follower.cpp src/motion_command_sweep.cpp)
    target_compile_options(${unit}_test PRIVATE -UNDEBUG)
    add_dependencies(tests ${unit}_test)
    add_test(NAME ${unit}_contract COMMAND ${unit}_test)
  endforeach()
  add_executable(motion_follower_response_test test/motion_follower_response_test.cpp
    src/motion_geometry.cpp src/motion_route.cpp src/motion_speed_profile.cpp
    src/motion_route_follower.cpp src/ranger_control.cpp)
  target_compile_options(motion_follower_response_test PRIVATE -UNDEBUG)
  add_dependencies(tests motion_follower_response_test)
  add_test(NAME motion_follower_response_contract COMMAND motion_follower_response_test)
  find_package(Threads REQUIRED)
  add_executable(navigation_progress_guard_test test/navigation_progress_guard_test.cpp)
  target_compile_options(navigation_progress_guard_test PRIVATE -UNDEBUG)
  target_link_libraries(navigation_progress_guard_test Threads::Threads)
  foreach(scenario unbound config stall late_progress rotation mirrored_rotation translation wrap
      translation_boundary rotation_boundary below_boundary duplicate jitter source_regression
      receipt_regression bad_clock paused_source future_sample original_stamp bad_pose regressed_sample
      conflicting_repeat nonadvancing_sample gate_lifetime)
    add_test(NAME navigation_progress_${scenario} COMMAND navigation_progress_guard_test ${scenario})
  endforeach()
  add_executable(command_source_gate_test test/command_source_gate_test.cpp)
  target_compile_options(command_source_gate_test PRIVATE -UNDEBUG)
  target_link_libraries(command_source_gate_test Threads::Threads)
  add_test(NAME command_source_gate_test COMMAND command_source_gate_test)
  add_executable(canonical_motion_feedback_test test/canonical_motion_feedback_test.cpp)
  target_compile_options(canonical_motion_feedback_test PRIVATE -UNDEBUG)
  add_test(NAME canonical_motion_feedback_test COMMAND canonical_motion_feedback_test)
  add_executable(ranger_command_limiter_test test/ranger_command_limiter_test.cpp src/ranger_control.cpp)
  target_compile_options(ranger_command_limiter_test PRIVATE -UNDEBUG)
  add_test(NAME ranger_command_limiter_test COMMAND ranger_command_limiter_test)
  add_executable(effective_planner_config_test test/effective_planner_config_test.cpp)
  target_compile_options(effective_planner_config_test PRIVATE -UNDEBUG)
  add_test(NAME effective_planner_config_test COMMAND effective_planner_config_test)
  add_rostest_gtest(route_preflight_ros test/route_preflight_ros.test test/route_preflight_ros.cpp)
  if(TARGET route_preflight_ros)
    add_dependencies(route_preflight_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(route_preflight_ros ranger_local_planner ${catkin_LIBRARIES})
  endif()
  add_rostest(test/rover_execution_ros.test)
  add_rostest(test/route_mission_preflight_ros.test)
  add_rostest(test/lidar_snapshot_ros.test)
  add_rostest(test/navigation_progress_move_base.test)
  add_rostest_gtest(navigation_progress_ros test/navigation_progress_ros.test
    test/navigation_progress_ros.cpp)
  if(TARGET navigation_progress_ros)
    add_dependencies(navigation_progress_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(navigation_progress_ros ranger_local_planner ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_ranger_local_planner
    test/test_ranger_local_planner.cpp
  )
  if(TARGET test_ranger_local_planner)
    add_dependencies(test_ranger_local_planner
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(test_ranger_local_planner
      ranger_local_planner
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS ranger_local_planner motion_preview_cli
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES ranger_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/base_command_arbiter.py
  scripts/check_cam2_ready.py
  scripts/lidar_safety_monitor.py
  scripts/register_workbench.py
  scripts/workbench_mission.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
