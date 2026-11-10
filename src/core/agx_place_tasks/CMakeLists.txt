cmake_minimum_required(VERSION 3.10)
project(agx_place_tasks)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS agx_task_workflow agx_robot_task_runtime roscpp
  geometry_msgs moveit_core moveit_msgs moveit_ros_planning_interface dh_gripper_msgs)
find_package(yaml-cpp REQUIRED)
catkin_package(INCLUDE_DIRS include LIBRARIES place_tasks
  CATKIN_DEPENDS agx_task_workflow agx_robot_task_runtime roscpp geometry_msgs moveit_core
  moveit_msgs moveit_ros_planning_interface dh_gripper_msgs DEPENDS yaml-cpp)
add_library(place_tasks
  src/onboard_rack_profile.cpp
  src/onboard_positions.cpp
  src/place_pose.cpp
  src/place_task.cpp
  src/pick_task.cpp
  src/pick_execution.cpp
  src/place_execution.cpp
  src/place_motion.cpp)
target_include_directories(place_tasks PUBLIC include ${catkin_INCLUDE_DIRS})
add_dependencies(place_tasks ${catkin_EXPORTED_TARGETS})
target_link_libraries(place_tasks PUBLIC yaml-cpp ${catkin_LIBRARIES})
add_executable(place_one_slot src/place_one_slot.cpp)
target_link_libraries(place_one_slot place_tasks)
add_dependencies(place_one_slot ${catkin_EXPORTED_TARGETS})

add_executable(pick_one_slot src/pick_one_slot.cpp)
target_link_libraries(pick_one_slot place_tasks)
add_dependencies(pick_one_slot ${catkin_EXPORTED_TARGETS})
target_link_libraries(place_one_slot place_tasks)
add_dependencies(place_one_slot ${catkin_EXPORTED_TARGETS})

# Planning tooling: the C++ helper keeps the profile validation single-sourced.
add_executable(print_rack_slots src/print_rack_slots.cpp)
target_link_libraries(print_rack_slots place_tasks)
add_dependencies(print_rack_slots ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS scripts/check_rack_reach.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  foreach(test onboard_rack_profile onboard_positions place_pose place_task place_execution pick_task pick_execution)
    add_executable(${PROJECT_NAME}_${test}_test test/${test}_test.cpp)
    target_link_libraries(${PROJECT_NAME}_${test}_test place_tasks)
    target_compile_options(${PROJECT_NAME}_${test}_test PRIVATE -UNDEBUG)
    add_test(NAME ${PROJECT_NAME}_${test}_test COMMAND ${PROJECT_NAME}_${test}_test)
  endforeach()
  set_tests_properties(agx_place_tasks_place_execution_test PROPERTIES TIMEOUT 45)
endif()
install(TARGETS place_tasks print_rack_slots place_one_slot pick_one_slot
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
