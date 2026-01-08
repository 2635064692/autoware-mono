cmake_minimum_required(VERSION 3.14)
project(autoware_following_goal_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

file(GLOB_RECURSE SOURCE_FILES src/*.cpp)
ament_auto_add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "autoware::following_goal_publisher::TargetTrackerNode"
  EXECUTABLE ${PROJECT_NAME}_target_tracker_node)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "autoware::following_goal_publisher::FollowingGoalPublisherNode"
  EXECUTABLE ${PROJECT_NAME}_node)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_target_tracker
    test/test_target_tracker.cpp)
  target_link_libraries(test_${PROJECT_NAME}_target_tracker ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_goal_generator
    test/test_goal_generator.cpp)
  target_link_libraries(test_${PROJECT_NAME}_goal_generator ${PROJECT_NAME})
endif()

ament_auto_package()
