cmake_minimum_required(VERSION 3.8)
project(robomaster_teleop)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
