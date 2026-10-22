cmake_minimum_required(VERSION 3.0.2)
project(agx_xarm_pick)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(yaml-cpp REQUIRED)
find_package(OpenSSL REQUIRED)
find_path(yaml-cpp_INCLUDE_DIRS yaml-cpp/yaml.h HINTS ${YAML_CPP_INCLUDE_DIR})
if(NOT yaml-cpp_INCLUDE_DIRS)
  message(FATAL_ERROR "yaml-cpp headers were not found")
endif()
set(yaml-cpp_LIBRARIES ${YAML_CPP_LIBRARIES})
set(OpenSSL_INCLUDE_DIRS ${OPENSSL_INCLUDE_DIR})
set(OpenSSL_LIBRARIES ${OPENSSL_CRYPTO_LIBRARY})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  move_base_msgs
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  message_generation
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf
  tf2_ros
  tf2_geometry_msgs
  shape_msgs
  agx_pick_msg
  ar_track_alvar_msgs
  bt_task_msgs
  dh_gripper_msgs
)
#add_service_files(
#  DIRECTORY srv
#  FILES PickStateSrv.srv
#)

#generate_messages(
#    DEPENDENCIES
#        std_msgs
#)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    agx_xarm_pick_node
    tube_grasp_mission
    tube_top_down_planner
    visual_alignment
    tube_target_provider
    visual_alignment_motion
    tube_task_config
    tube_effects
  CATKIN_DEPENDS
    agx_pick_msg
    ar_track_alvar_msgs
    bt_task_msgs
    geometry_msgs
    message_runtime
    moveit_msgs
    shape_msgs
    tf2_geometry_msgs
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    move_base_msgs
    dh_gripper_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_visual_tools
    tf
  DEPENDS yaml-cpp OpenSSL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories("/usr/include/eigen3")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
add_library(agx_xarm_pick_node STATIC src/agx_xarm_pick_node.cpp)
add_dependencies(agx_xarm_pick_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(agx_xarm_pick_node ${catkin_LIBRARIES})
set_property(TARGET agx_xarm_pick_node PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(agx_xarm_pick src/agx_xarm_pick.cpp )
add_dependencies(agx_xarm_pick ${catkin_EXPORTED_TARGETS})
target_link_libraries(agx_xarm_pick agx_xarm_pick_node ${catkin_LIBRARIES})

add_executable(viewtest src/test.cpp )
add_dependencies(viewtest ${catkin_EXPORTED_TARGETS})
target_link_libraries(viewtest ${catkin_LIBRARIES})

add_executable(order src/order.cpp )
target_link_libraries(order ${catkin_LIBRARIES})

add_executable(pub_pick_state tool/PubPickState.cpp )
add_dependencies(pub_pick_state ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_pick_state ${catkin_LIBRARIES})

add_executable(control_gripper tool/control_gripper.cpp )
add_dependencies(control_gripper ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_gripper ${catkin_LIBRARIES})

add_executable(record tool/record.cpp )
target_link_libraries(record ${catkin_LIBRARIES})

add_executable(demo_pick demo/demo_pick.cpp )
target_link_libraries(demo_pick agx_xarm_pick_node ${catkin_LIBRARIES})

add_executable(demo_place demo/demo_place.cpp )
target_link_libraries(demo_place agx_xarm_pick_node ${catkin_LIBRARIES})

add_executable(mission_contract_test test/mission_contract_test.cpp)
target_link_libraries(mission_contract_test ${catkin_LIBRARIES})

add_executable(tube_grasp_contract_test test/tube_grasp_contract_test.cpp)
target_link_libraries(tube_grasp_contract_test ${catkin_LIBRARIES})

add_executable(tube_search_contract_test test/tube_search_contract_test.cpp)
target_link_libraries(tube_search_contract_test ${catkin_LIBRARIES})

add_executable(
  visual_alignment_contract_test
  test/visual_alignment_contract_test.cpp
)

add_library(visual_alignment STATIC src/visual_alignment.cpp)

add_library(tube_effects STATIC src/tube_effects.cpp src/tube_observation_buffer.cpp)
set_property(TARGET tube_effects PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(tube_task_config STATIC src/resolved_tube_config.cpp
  src/tube_task_binding.cpp src/tube_perception_projection.cpp)
target_link_libraries(tube_task_config yaml-cpp OpenSSL::Crypto)
set_property(TARGET tube_task_config PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(tube_target_provider STATIC src/tube_target_provider.cpp)
add_dependencies(tube_target_provider ${catkin_EXPORTED_TARGETS})
target_link_libraries(tube_target_provider tube_task_config ${catkin_LIBRARIES})
set_property(TARGET tube_target_provider PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(visual_alignment_motion STATIC src/visual_alignment_motion.cpp)
add_dependencies(visual_alignment_motion ${catkin_EXPORTED_TARGETS})
target_link_libraries(visual_alignment_motion ${catkin_LIBRARIES})
set_property(TARGET visual_alignment_motion PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(
  visual_target_aligner_test
  test/visual_target_aligner_test.cpp
)
target_link_libraries(visual_target_aligner_test visual_alignment)

add_executable(arm_execution_test test/arm_execution_test.cpp)

add_executable(tube_lift_contract_test test/tube_lift_contract_test.cpp)

add_executable(tube_preparation_contract_test test/tube_preparation_contract_test.cpp)

add_executable(tube_view_region_test test/tube_view_region_test.cpp)

add_library(tube_top_down_planner STATIC src/tube_top_down_planner.cpp)
add_dependencies(tube_top_down_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(tube_top_down_planner ${catkin_LIBRARIES})
set_property(TARGET tube_top_down_planner PROPERTY POSITION_INDEPENDENT_CODE ON)

add_executable(
  tube_top_down_planner_contract_test
  test/tube_top_down_planner_contract_test.cpp
)
add_dependencies(
  tube_top_down_planner_contract_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  tube_top_down_planner_contract_test
  tube_top_down_planner
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GTest REQUIRED)
  add_test(NAME mission_contract_test COMMAND mission_contract_test)
  add_test(NAME tube_grasp_contract_test COMMAND tube_grasp_contract_test)
  add_test(NAME tube_search_contract_test COMMAND tube_search_contract_test)
  add_test(NAME tube_lift_contract_test COMMAND tube_lift_contract_test)
  add_test(NAME tube_preparation_contract_test COMMAND tube_preparation_contract_test)
  add_test(NAME tube_view_region_test COMMAND tube_view_region_test)
  add_test(
    NAME visual_alignment_contract_test
    COMMAND visual_alignment_contract_test
  )
  add_test(NAME visual_target_aligner_test COMMAND visual_target_aligner_test)
  add_executable(
    tube_target_provider_rostest
    test/tube_target_provider_rostest.cpp
  )
  target_include_directories(
    tube_target_provider_rostest PRIVATE ${GTEST_INCLUDE_DIRS}
  )
  add_dependencies(tube_target_provider_rostest ${catkin_EXPORTED_TARGETS})
  target_link_libraries(
    tube_target_provider_rostest
    tube_target_provider
    ${GTEST_LIBRARIES}
    ${catkin_LIBRARIES}
  )
  add_rostest(test/tube_target_provider.test)
  add_rostest(test/speed_config_rejection.test)
  add_test(
    NAME tube_top_down_planner_contract_test
    COMMAND tube_top_down_planner_contract_test
  )
endif()

add_executable(tube_grasp_geometry_test test/tube_grasp_geometry_test.cpp)
target_link_libraries(tube_grasp_geometry_test ${catkin_LIBRARIES})
if(CATKIN_ENABLE_TESTING)
  add_test(NAME tube_grasp_geometry_test COMMAND tube_grasp_geometry_test)
endif()

add_library(
  tube_grasp_mission STATIC
  src/tube_grasp_config.cpp
  src/tube_grasp_mission.cpp
  src/tube_live_observer.cpp
)
add_dependencies(tube_grasp_mission ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  tube_grasp_mission
  tube_top_down_planner
  visual_alignment
  tube_target_provider
  visual_alignment_motion
  tube_task_config
  tube_effects
  ${catkin_LIBRARIES}
)
set_property(TARGET tube_grasp_mission PROPERTY POSITION_INDEPENDENT_CODE ON)

if(CATKIN_ENABLE_TESTING)
  add_executable(tube_task_binding_test test/tube_task_binding_test.cpp)
  target_compile_options(tube_task_binding_test PRIVATE -UNDEBUG)
  target_link_libraries(tube_task_binding_test tube_grasp_mission)
  add_test(NAME tube_task_binding_test COMMAND tube_task_binding_test
    ${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/tube_task_config/taskA.yaml)
  add_executable(tube_config_snapshot_test test/tube_config_snapshot_test.cpp)
  target_compile_options(tube_config_snapshot_test PRIVATE -UNDEBUG)
  target_include_directories(tube_config_snapshot_test PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(tube_config_snapshot_test tube_grasp_mission ${GTEST_LIBRARIES})
  add_rostest(test/tube_config_snapshot.test)
  add_executable(tube_motion_window_test test/tube_motion_window_test.cpp)
  target_compile_options(tube_motion_window_test PRIVATE -UNDEBUG)
  target_link_libraries(tube_motion_window_test tube_effects ${catkin_LIBRARIES})
  add_test(NAME tube_motion_window_test COMMAND tube_motion_window_test)
endif()

add_executable(tube_speed_config_rejection_test test/tube_speed_config_rejection_test.cpp)
add_dependencies(tube_speed_config_rejection_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(tube_speed_config_rejection_test tube_grasp_mission ${catkin_LIBRARIES})

add_executable(demo_tube_pick demo/demo_tube_pick.cpp)
add_dependencies(demo_tube_pick ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  demo_tube_pick
  tube_grasp_mission
  ${catkin_LIBRARIES}
)

add_executable(demo_tube_search demo/demo_tube_search.cpp)
add_dependencies(demo_tube_search ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  demo_tube_search
  tube_grasp_mission
  ${catkin_LIBRARIES}
)

install(TARGETS
  agx_xarm_pick_node tube_grasp_mission tube_top_down_planner visual_alignment
  tube_target_provider visual_alignment_motion tube_task_config tube_effects
  agx_xarm_pick viewtest order pub_pick_state control_gripper record
  demo_pick demo_place demo_tube_pick demo_tube_search
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config launch docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
