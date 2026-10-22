cmake_minimum_required(VERSION 3.0.2)
project(agx_pick_msg)


find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation

)
add_message_files(
  DIRECTORY msg
  FILES
  VisualTarget2D.msg
)
add_message_files(DIRECTORY msg FILES TubePerceptionConfig.msg)
add_service_files(
    DIRECTORY srv
    FILES
    PickStateSrv.srv
    AgxPickSrv.srv
    OrderSrv.srv
    TaskCmd.srv
    DetectTubeCap.srv
    SelectTubeTarget2D.srv
    DetectTubeCapConfigured.srv
    SelectTubeTarget2DConfigured.srv
)


generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    message_runtime

)
include_directories(
    #include
    ${catkin_INCLUDE_DIRS}
)
