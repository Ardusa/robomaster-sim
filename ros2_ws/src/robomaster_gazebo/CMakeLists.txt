cmake_minimum_required(VERSION 3.8)
project(robomaster_gazebo)
find_package(ament_cmake REQUIRED)

set(AWS_SMALL_HOUSE_WORLD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/aws_robomaker_small_house_world)

install(DIRECTORY launch worlds models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${AWS_SMALL_HOUSE_WORLD_DIR}/models/ DESTINATION share/${PROJECT_NAME})
install(FILES ${AWS_SMALL_HOUSE_WORLD_DIR}/worlds/small_house.world DESTINATION share/${PROJECT_NAME}/worlds)
ament_package()