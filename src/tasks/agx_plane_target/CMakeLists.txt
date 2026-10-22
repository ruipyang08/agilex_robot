cmake_minimum_required(VERSION 3.0.2)
project(agx_plane_target)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  visualization_msgs
  interactive_markers
  message_generation
)

add_service_files(FILES
  PutPoint.srv
  PickPoint.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  scripts/plane_target_marker.py
  scripts/put
  scripts/pick
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
