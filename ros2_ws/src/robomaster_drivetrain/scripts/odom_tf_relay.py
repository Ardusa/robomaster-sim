#!/usr/bin/env python3
"""Publish odom->base_link on /tf from mecanum odometry (Nav2/AMCL expect /tf)."""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfRelay(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_relay")
        self._br = TransformBroadcaster(self)
        self.create_subscription(
            Odometry,
            "/mecanum_drive_controller/odometry",
            self._on_odom,
            10,
        )
        self.get_logger().info(
            "publishing odom->base_link on /tf from mecanum odometry"
        )

    def _on_odom(self, msg: Odometry) -> None:
        t = TransformStamped()
        # Use the odometry stamp so TF stays aligned with sim time / scan data.
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id or "odom"
        t.child_frame_id = msg.child_frame_id or "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
