cmake_minimum_required(VERSION 3.8)
project(robomaster_command)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActionPrimitive.msg"
  "srv/GenerateActionSequence.srv"
  "srv/ExecuteActionSequence.srv"
  DEPENDENCIES std_msgs
)

# No ament_python_install_package: rosidl already owns the Python package name.
install(PROGRAMS
  scripts/grounding_node.py
  scripts/command_generator.py
  scripts/translator_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
