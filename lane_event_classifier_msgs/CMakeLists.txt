cmake_minimum_required(VERSION 3.14)
project(lane_event_classifier_msgs)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DrivingState.msg"
  "msg/DrivingFactor.msg"
  DEPENDENCIES std_msgs
)

ament_auto_package()
