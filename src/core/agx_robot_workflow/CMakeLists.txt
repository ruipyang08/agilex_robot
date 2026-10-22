cmake_minimum_required(VERSION 3.0.2)
project(agx_robot_workflow)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(catkin REQUIRED COMPONENTS agx_navigation_tasks agx_tube_tasks agx_task_workflow agx_robot_task_runtime roscpp std_msgs std_srvs nav_msgs sensor_msgs geometry_msgs actionlib actionlib_msgs move_base_msgs agx_workbench_navigation agx_xarm_pick moveit_core moveit_msgs moveit_ros_planning_interface geometric_shapes bt_task_msgs dh_gripper_msgs)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
# This package is an application, not a second task API. Consumers depend on
# the framework and selected peer modules directly.
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
add_executable(run_robot_workflow src/workflow_node.cpp)
add_dependencies(run_robot_workflow ${catkin_EXPORTED_TARGETS})
target_link_libraries(run_robot_workflow yaml-cpp Threads::Threads ${catkin_LIBRARIES})
catkin_install_python(PROGRAMS scripts/workflow_watchdog.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(workflow_grasp_host_binding_test
    test/grasp_host_binding_ros.test test/grasp_host_binding_ros.cpp)
  add_dependencies(workflow_grasp_host_binding_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(workflow_grasp_host_binding_test yaml-cpp ${catkin_LIBRARIES})
  target_compile_options(workflow_grasp_host_binding_test PRIVATE -UNDEBUG)
  # Linux native fixture only: exact symbol/archive ownership is review evidence.
  set_property(TARGET workflow_grasp_host_binding_test APPEND_STRING PROPERTY
    LINK_FLAGS " -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/workflow_grasp_host_binding.link.map")

  add_rostest_gtest(workflow_task_extension_ros_test
    test/task_extension_ros.test test/task_extension_ros.cpp)
  add_dependencies(workflow_task_extension_ros_test ${catkin_EXPORTED_TARGETS})
  target_link_libraries(workflow_task_extension_ros_test yaml-cpp ${catkin_LIBRARIES})
  target_compile_options(workflow_task_extension_ros_test PRIVATE -UNDEBUG)

  add_rostest(test/grasp_unjoined_process_ros.test DEPENDENCIES workflow_grasp_host_binding_test)
  add_rostest(test/task_workflow_ros.test)
  add_executable(workflow_registered_tasks_test test/registered_tasks_test.cpp)
  target_link_libraries(workflow_registered_tasks_test yaml-cpp ${catkin_LIBRARIES})
  target_compile_options(workflow_registered_tasks_test PRIVATE -UNDEBUG)
  add_test(NAME workflow_registered_tasks_test COMMAND workflow_registered_tasks_test)

endif()
install(TARGETS run_robot_workflow
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
