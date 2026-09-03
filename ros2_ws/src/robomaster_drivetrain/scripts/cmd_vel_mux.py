#!/usr/bin/env python3
"""Priority mux for Twist producers: highest-priority active input wins."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelMux(Node):
    def __init__(self):
        super().__init__("cmd_vel_mux")

        self.declare_parameter("topics_names", rclpy.Parameter.Type.STRING_ARRAY)
        names = self.get_parameter("topics_names").value

        self._inputs = []
        for name in names:
            self.declare_parameter(f"topics.{name}.topic", rclpy.Parameter.Type.STRING)
            self.declare_parameter(f"topics.{name}.timeout", rclpy.Parameter.Type.DOUBLE)
            self.declare_parameter(f"topics.{name}.priority", rclpy.Parameter.Type.INTEGER)
            topic = self.get_parameter(f"topics.{name}.topic").value
            timeout = self.get_parameter(f"topics.{name}.timeout").value
            priority = self.get_parameter(f"topics.{name}.priority").value

            state = {
                "name": name,
                "priority": priority,
                "timeout": timeout,
                "last": None,
                "msg": None,
            }
            self.create_subscription(
                Twist, topic, lambda msg, s=state: self._on_input(msg, s), 10
            )
            self._inputs.append(state)
            self.get_logger().info(
                f"input '{name}': {topic} (priority {priority}, timeout {timeout}s)"
            )

        self._inputs.sort(key=lambda s: s["priority"], reverse=True)
        self._pub = self.create_publisher(Twist, "cmd_vel_out", 10)
        self._winner = None
        self.create_timer(0.02, self._tick)

    def _on_input(self, msg, state):
        state["last"] = self.get_clock().now()
        state["msg"] = msg

    def _tick(self):
        now = self.get_clock().now()
        for state in self._inputs:
            if state["last"] is None:
                continue
            age = (now - state["last"]).nanoseconds / 1e9
            if age > state["timeout"]:
                continue
            if self._winner != state["name"]:
                self.get_logger().info(f"switching to '{state['name']}'")
                self._winner = state["name"]
            self._pub.publish(state["msg"])
            return
        self._winner = None


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
