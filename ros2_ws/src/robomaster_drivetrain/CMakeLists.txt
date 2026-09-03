cmake_minimum_required(VERSION 3.8)
project(robomaster_drivetrain)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToPose.srv"
)

ament_get_python_install_dir(python_install_dir)
install(
  DIRECTORY robomaster_drivetrain
  DESTINATION ${python_install_dir}/robomaster_drivetrain
  FILES_MATCHING PATTERN "*.py"
)

install(PROGRAMS
  scripts/cmd_vel_mux.py
  scripts/go_to_pose_node.py
  scripts/odom_tf_relay.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/generate_map.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
