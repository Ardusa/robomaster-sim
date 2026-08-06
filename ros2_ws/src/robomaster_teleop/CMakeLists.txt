cmake_minimum_required(VERSION 3.8)
project(robomaster_teleop)
find_package(ament_cmake REQUIRED)

# No launch/ directory: the keyboard node has to be run directly. ros2 launch
# gives its children /dev/null on stdin, so termios/raw-mode reads fail with
# "Inappropriate ioctl for device". The Makefile runs `ros2 run` in the
# foreground instead.
install(PROGRAMS scripts/teleop_node.py DESTINATION lib/${PROJECT_NAME})

ament_package()
