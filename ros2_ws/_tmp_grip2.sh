#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
echo "=== nodes ==="
ros2 node list 2>&1 | sort
echo "=== controllers ==="
ros2 control list_controllers 2>&1
echo "=== grip hw ==="
ros2 control list_hardware_interfaces 2>&1 | grep -i grip
echo "=== installed gripper cmd ==="
grep -n "Float64MultiArray(data=" /root/ros2_ws/install/robomaster_arm/lib/robomaster_arm/arm_node.py | head
echo "=== urdf grip joints ==="
# from robot_description if available
timeout 3 ros2 topic echo /robot_description --once 2>/dev/null | tr ' ' '\n' | grep -E 'gripper_[mr]_joint' | head
echo "=== action ==="
ros2 action list 2>&1
