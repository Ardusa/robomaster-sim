cmake_minimum_required(VERSION 3.8)
project(robomaster_arm)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveArm.action"
  "action/SetGripper.action"
  "msg/ArmState.msg"
  "msg/GripperState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# No ament_python_install_package: rosidl already owns the Python package name.
install(PROGRAMS
  scripts/arm_node.py
  scripts/arm_kinematics.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
