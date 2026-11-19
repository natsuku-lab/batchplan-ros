cmake_minimum_required(VERSION 3.22)
project(batchplan_moveit LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

add_library(batchplan_moveit SHARED src/batchplan_planner.cpp)
target_compile_features(batchplan_moveit PUBLIC cxx_std_17)
target_link_libraries(batchplan_moveit PUBLIC
  moveit_core::moveit_planning_interface
  moveit_core::moveit_planning_scene
  moveit_core::moveit_robot_state
  moveit_core::moveit_robot_trajectory
  ${moveit_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action)

pluginlib_export_plugin_description_file(moveit_core batchplan_moveit_plugin.xml)

install(TARGETS batchplan_moveit LIBRARY DESTINATION lib)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
