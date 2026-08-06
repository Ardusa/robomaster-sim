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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ArmSdk.srv"
  "srv/GripperSdk.srv"
)

# --- shared TCP client, linked into both the plugin and the test binary ---
add_library(tcp_client SHARED
  src/tcp_client.cpp
)
target_include_directories(tcp_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tcp_client rclcpp)

# --- SDK service facade (shared by HI and standalone bridge node) ---
add_library(sdk_bridge SHARED
  src/sdk_bridge.cpp
)
target_include_directories(sdk_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sdk_bridge tcp_client)
ament_target_dependencies(sdk_bridge rclcpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(sdk_bridge "${cpp_typesupport_target}")

# --- ros2_control hardware_interface plugin ---
# Named differently from the rosidl target (${PROJECT_NAME}).
add_library(robomaster_driver_hw SHARED
  src/hardware_interface.cpp
)
target_include_directories(robomaster_driver_hw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robomaster_driver_hw tcp_client sdk_bridge)
ament_target_dependencies(robomaster_driver_hw
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)
pluginlib_export_plugin_description_file(hardware_interface robomaster_driver.xml)

# --- standalone connectivity smoke test (no ROS2 node, no controller_manager) ---
add_executable(connection_test src/connection_test.cpp)
target_link_libraries(connection_test tcp_client)

# --- arm/gripper services when wheels are not holding the control port ---
add_executable(sdk_bridge_node src/sdk_bridge_node.cpp)
target_link_libraries(sdk_bridge_node tcp_client sdk_bridge)
ament_target_dependencies(sdk_bridge_node rclcpp)

install(
  TARGETS tcp_client sdk_bridge robomaster_driver_hw
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS connection_test sdk_bridge_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(FILES robomaster_driver.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(tcp_client sdk_bridge robomaster_driver_hw)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
