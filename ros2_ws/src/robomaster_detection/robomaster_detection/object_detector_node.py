#!/usr/bin/env python3
"""COCO object detection via YOLOv8n ONNX (CPU).

Consumes /camera/image_raw — same contract as the old AprilTag stack — and
publishes vision_msgs/Detection2DArray on /object_detections. Inference is
rate-limited so it stays usable on Mac/WSL CPU Docker.
"""
from __future__ import annotations

import time
from typing import List, Optional, Sequence, Set, Tuple

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from robomaster_detection.coco_names import COCO_NAMES


def _letterbox(
    image: np.ndarray, imgsz: int
) -> Tuple[np.ndarray, float, int, int]:
    """Resize with unchanged aspect ratio, pad to square (Ultralytics-style)."""
    h, w = image.shape[:2]
    scale = min(imgsz / h, imgsz / w)
    nh, nw = int(round(h * scale)), int(round(w * scale))
    resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((imgsz, imgsz, 3), 114, dtype=np.uint8)
    top = (imgsz - nh) // 2
    left = (imgsz - nw) // 2
    canvas[top : top + nh, left : left + nw] = resized
    return canvas, scale, left, top


def _nms_xyxy(
    boxes: np.ndarray, scores: np.ndarray, iou_thresh: float
) -> List[int]:
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes.T
    areas = (x2 - x1).clip(0) * (y2 - y1).clip(0)
    order = scores.argsort()[::-1]
    keep: List[int] = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = (xx2 - xx1).clip(0) * (yy2 - yy1).clip(0)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        order = order[1:][iou <= iou_thresh]
    return keep


class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__("object_detector_node")

        self.declare_parameter(
            "model_path", "/opt/robomaster/models/yolov8n.onnx"
        )
        self.declare_parameter("confidence", 0.35)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 320)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("class_allowlist", [])

        model_path = self.get_parameter("model_path").value
        self._confidence = float(self.get_parameter("confidence").value)
        self._iou = float(self.get_parameter("iou").value)
        self._imgsz = int(self.get_parameter("imgsz").value)
        self._min_period = 1.0 / max(
            float(self.get_parameter("rate_hz").value), 0.1
        )
        allowlist = list(self.get_parameter("class_allowlist").value)
        self._allowlist: Optional[Set[str]] = (
            set(allowlist) if allowlist else None
        )

        self._bridge = CvBridge()
        self._last_infer = 0.0

        try:
            self._session = ort.InferenceSession(
                model_path,
                providers=["CPUExecutionProvider"],
            )
        except Exception as exc:
            self.get_logger().fatal(
                f"Failed to load ONNX model at '{model_path}': {exc}"
            )
            raise

        self._input_name = self._session.get_inputs()[0].name
        self.get_logger().info(
            f"Loaded {model_path} (imgsz={self._imgsz}, "
            f"rate={1.0 / self._min_period:.1f} Hz)"
        )

        self._pub = self.create_publisher(
            Detection2DArray, "/object_detections", 10
        )
        self.create_subscription(
            Image,
            "/camera/image_raw",
            self._on_image,
            qos_profile_sensor_data,
        )

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self._last_infer < self._min_period:
            return
        self._last_infer = now

        frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        detections = self._infer(frame)

        out = Detection2DArray()
        out.header = msg.header
        out.detections = detections
        self._pub.publish(out)

    def _infer(self, frame: np.ndarray) -> List[Detection2D]:
        blob, scale, pad_x, pad_y = _letterbox(frame, self._imgsz)
        tensor = blob[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
        tensor = np.expand_dims(tensor, axis=0)

        outputs = self._session.run(None, {self._input_name: tensor})
        preds = np.squeeze(outputs[0])
        # YOLOv8 export: (84, N) or (N, 84)
        if preds.ndim != 2:
            self.get_logger().warning(
                f"Unexpected ONNX output shape {preds.shape}"
            )
            return []
        if preds.shape[0] < preds.shape[1] and preds.shape[0] in (84, 85):
            preds = preds.T

        boxes_xyxy, scores, class_ids = self._decode(preds, scale, pad_x, pad_y)
        if len(boxes_xyxy) == 0:
            return []

        keep = _nms_xyxy(boxes_xyxy, scores, self._iou)
        h, w = frame.shape[:2]
        results: List[Detection2D] = []
        for i in keep:
            cls_id = int(class_ids[i])
            if cls_id < 0 or cls_id >= len(COCO_NAMES):
                continue
            name = COCO_NAMES[cls_id]
            if self._allowlist is not None and name not in self._allowlist:
                continue

            x1, y1, x2, y2 = boxes_xyxy[i]
            x1 = float(np.clip(x1, 0, w - 1))
            y1 = float(np.clip(y1, 0, h - 1))
            x2 = float(np.clip(x2, 0, w - 1))
            y2 = float(np.clip(y2, 0, h - 1))
            bw = max(x2 - x1, 0.0)
            bh = max(y2 - y1, 0.0)
            if bw < 1.0 or bh < 1.0:
                continue

            det = Detection2D()
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = x1 + bw / 2.0
            det.bbox.center.position.y = y1 + bh / 2.0
            det.bbox.center.theta = 0.0
            det.bbox.size_x = bw
            det.bbox.size_y = bh
            det.id = name

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = float(scores[i])
            det.results = [hyp]
            results.append(det)

        return results

    def _decode(
        self,
        preds: np.ndarray,
        scale: float,
        pad_x: int,
        pad_y: int,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        # preds: (N, 4+nc) — cx, cy, w, h, class scores
        boxes = preds[:, :4]
        class_scores = preds[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        scores = class_scores[np.arange(class_scores.shape[0]), class_ids]
        mask = scores >= self._confidence
        boxes = boxes[mask]
        scores = scores[mask]
        class_ids = class_ids[mask]
        if len(boxes) == 0:
            return (
                np.zeros((0, 4), dtype=np.float32),
                np.zeros((0,), dtype=np.float32),
                np.zeros((0,), dtype=np.int32),
            )

        cx, cy, bw, bh = boxes.T
        x1 = cx - bw / 2.0
        y1 = cy - bh / 2.0
        x2 = cx + bw / 2.0
        y2 = cy + bh / 2.0
        # Undo letterbox
        x1 = (x1 - pad_x) / scale
        y1 = (y1 - pad_y) / scale
        x2 = (x2 - pad_x) / scale
        y2 = (y2 - pad_y) / scale
        xyxy = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)
        return xyxy, scores.astype(np.float32), class_ids.astype(np.int32)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = ObjectDetectorNode()
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
