cmake_minimum_required(VERSION 3.16)
project(batchplan_examples NONE)

find_package(ament_cmake REQUIRED)

# tiago_pro holds what scripts/fetch_tiago_pro.py fetches, empty until it runs.
install(DIRECTORY config launch tiago_pro urdf DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/best_grasp.py DESTINATION lib/${PROJECT_NAME})

ament_package()
