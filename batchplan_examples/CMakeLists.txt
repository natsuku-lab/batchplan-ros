cmake_minimum_required(VERSION 3.16)
project(batchplan_examples NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/best_grasp.py DESTINATION lib/${PROJECT_NAME})

ament_package()
