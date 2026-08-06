#!/usr/bin/env python3
"""Draws COCO object detections onto the camera image for eyeballing.

Purely a debug view — nothing control-related subscribes to it. Publishes
/camera/image_annotated; view with web_video_server:

    http://localhost:8080/stream?topic=/camera/image_annotated

Keeps the latest Detection2DArray and paints it onto every new frame so the
annotated stream stays smooth even when the detector runs at a few Hz.
"""
from __future__ import annotations

from typing import Optional, Sequence

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

_GREEN = (0, 255, 0)
_LABEL = (255, 255, 255)


class DetectionOverlayNode(Node):
    def __init__(self):
        super().__init__("detection_overlay_node")

        self._bridge = CvBridge()
        self._latest: Optional[Detection2DArray] = None
        self._pub = self.create_publisher(Image, "/camera/image_annotated", 10)

        self.create_subscription(
            Detection2DArray,
            "/object_detections",
            self._on_detections,
            10,
        )
        self.create_subscription(
            Image,
            "/camera/image_raw",
            self._on_image,
            qos_profile_sensor_data,
        )

    def _on_detections(self, msg: Detection2DArray) -> None:
        self._latest = msg

    def _on_image(self, image_msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(image_msg, "bgr8")
        dets = self._latest
        if dets is not None:
            for det in dets.detections:
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                w = det.bbox.size_x
                h = det.bbox.size_y
                x1 = int(cx - w / 2.0)
                y1 = int(cy - h / 2.0)
                x2 = int(cx + w / 2.0)
                y2 = int(cy + h / 2.0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), _GREEN, 2)

                score = det.results[0].hypothesis.score if det.results else 0.0
                label = det.id or (
                    det.results[0].hypothesis.class_id if det.results else "?"
                )
                text = f"{label} {score:.2f}"
                (tw, th), _ = cv2.getTextSize(
                    text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
                )
                cv2.rectangle(
                    frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), _GREEN, -1
                )
                cv2.putText(
                    frame,
                    text,
                    (x1 + 2, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    _LABEL,
                    1,
                    cv2.LINE_AA,
                )

        annotated = self._bridge.cv2_to_imgmsg(frame, "bgr8")
        annotated.header = image_msg.header
        self._pub.publish(annotated)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = DetectionOverlayNode()
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
