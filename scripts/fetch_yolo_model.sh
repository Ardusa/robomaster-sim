#!/usr/bin/env bash
# Export YOLOv8n ONNX for local (non-Docker) use.
# Prefer the Docker image path: /opt/robomaster/models/yolov8n.onnx
set -euo pipefail

out_dir="${1:-ros2_ws/src/robomaster_detection/models}"
mkdir -p "${out_dir}"

python3 - <<PY
from ultralytics import YOLO
m = YOLO("yolov8n.pt")
m.export(format="onnx", imgsz=320, simplify=True)
PY

mv -f yolov8n.onnx "${out_dir}/yolov8n.onnx"
echo "wrote ${out_dir}/yolov8n.onnx"
