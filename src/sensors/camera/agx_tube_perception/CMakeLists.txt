cmake_minimum_required(VERSION 3.0.2)
project(agx_tube_perception)

find_package(catkin REQUIRED COMPONENTS
  agx_pick_msg
  bt_task_msgs
  cv_bridge
  geometry_msgs
  message_filters
  rospy
  sensor_msgs
  tf2_ros
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    agx_pick_msg
    bt_task_msgs
    cv_bridge
    geometry_msgs
    message_filters
    rospy
    sensor_msgs
    tf2_ros
)

catkin_install_python(
  PROGRAMS
    scripts/tube_cap_detector_node.py
    test/test_detector_layout_service.py
    test/test_target_2d_service.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/test_detector_layout_service.test)
  add_rostest(test/test_target_2d_service.test)
endif()
