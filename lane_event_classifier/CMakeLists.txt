cmake_minimum_required(VERSION 3.14)
project(lane_event_classifier)

find_package(autoware_cmake REQUIRED)
autoware_package()

unset(PYTHON_EXECUTABLE CACHE)
unset(PYTHON_EXECUTABLE)

generate_parameter_library(lane_event_classifier_parameters
  schema/lane_event_classifier.schema.yaml
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/lane_event_classifier_node.cpp
  src/debug.cpp
  src/detail/lane_tracker.cpp
  src/lane_following/checker.cpp
  src/lane_change/geometry.cpp
  src/lane_change/classifier.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  lane_event_classifier_parameters
)

# Suppress deprecated-declarations from generate_parameter_library internals
target_compile_options(${PROJECT_NAME}_lib PRIVATE -Wno-deprecated-declarations)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "lane_event_classifier::LaneEventClassifierNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_lane_tracker
    test/test_lane_tracker.cpp
  )
  target_link_libraries(test_lane_tracker ${PROJECT_NAME}_lib)
  target_compile_options(test_lane_tracker PRIVATE -Wno-deprecated-declarations)

  ament_add_ros_isolated_gtest(test_lane_following
    test/test_lane_following.cpp
  )
  target_link_libraries(test_lane_following ${PROJECT_NAME}_lib)
  target_compile_definitions(test_lane_following PRIVATE
    "TEST_MAP_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/test/map/lanelet2_map.osm\"")
  target_compile_options(test_lane_following PRIVATE -Wno-deprecated-declarations)

  ament_add_ros_isolated_gtest(test_lane_change_classifier
    test/test_lane_change_classifier.cpp
  )
  target_link_libraries(test_lane_change_classifier ${PROJECT_NAME}_lib)
  target_compile_definitions(test_lane_change_classifier PRIVATE
    "TEST_MAP_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/test/map/lanelet2_map.osm\"")
  target_compile_options(test_lane_change_classifier PRIVATE -Wno-deprecated-declarations)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  param
  schema
)
