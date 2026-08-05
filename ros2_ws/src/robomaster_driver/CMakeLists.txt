cmake_minimum_required(VERSION 3.8)
project(robomaster_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# --- shared TCP client, linked into both the plugin and the test binary ---
add_library(tcp_client SHARED
  src/tcp_client.cpp
)
target_include_directories(tcp_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tcp_client rclcpp)

# --- ros2_control hardware_interface plugin ---
add_library(robomaster_driver SHARED
  src/hardware_interface.cpp
)
target_include_directories(robomaster_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robomaster_driver tcp_client)
ament_target_dependencies(robomaster_driver
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)
pluginlib_export_plugin_description_file(hardware_interface robomaster_driver.xml)

# --- standalone connectivity smoke test (no ROS2 node, no controller_manager) ---
add_executable(connection_test src/connection_test.cpp)
target_link_libraries(connection_test tcp_client)

install(
  TARGETS tcp_client robomaster_driver
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS connection_test DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(FILES robomaster_driver.xml DESTINATION share/${PROJECT_NAME})
# No urdf/ here: the <ros2_control> block for this hardware plugin lives in
# robomaster_description (it names the plugin by string, so no dep is needed).
# No config/ either: the controller yamls live in robomaster_drivetrain, next to
# the sim's, since both backends load the same controllers.
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(tcp_client robomaster_driver)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()