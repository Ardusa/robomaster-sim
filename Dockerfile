# Separate slim stage: ultralytics + the ROS desktop image fight over numpy /
# matplotlib, so export ONNX here and copy the weights into the runtime image.
# CPU torch only — we just need export, not CUDA.
FROM python:3.10-slim AS yolo-model

RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 libxcb1 \
    && rm -rf /var/lib/apt/lists/*

# PyPI first for deps (flit_core, jinja2, typing-extensions); PyTorch index is extra only.
# --index-url alone breaks when pip rejects PyTorch-hosted wheels and tries to build from sdist.
RUN pip3 install --no-cache-dir jinja2 typing-extensions \
    && pip3 install --no-cache-dir torch torchvision --extra-index-url https://download.pytorch.org/whl/cpu \
    && pip3 install --no-cache-dir ultralytics \
    && python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt').export(format='onnx', imgsz=320, simplify=True)" \
    && mkdir -p /opt/robomaster/models \
    && mv yolov8n.onnx /opt/robomaster/models/yolov8n.onnx


FROM osrf/ros:humble-desktop-full

# NOTE: no ros-humble-twist-mux — its binary links libdiagnostic_updater.so,
# which no ros-humble-diagnostic-updater in the repos ships. robomaster_bringup
# has a small cmd_vel_mux instead.
# Video is watched in a browser (web_video_server), not an X11 GUI — no rqt.
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-gz-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joint-state-publisher \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-web-video-server \
    python3-colcon-common-extensions \
    python3-aiohttp \
    python3-av \
    python3-opencv \
    python3-pip \
    ffmpeg \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# Keep numpy 1.x: system python3-opencv was built against it; pip's default
# numpy 2.x breaks cv2 ("_ARRAY_API not found").
RUN pip3 install --no-cache-dir "numpy<2" onnxruntime

COPY --from=yolo-model /opt/robomaster/models/yolov8n.onnx /opt/robomaster/models/yolov8n.onnx

# Mac/Windows: Gazebo's Ogre still needs a display for GLX even with
# --headless-rendering. Xvfb provides one inside the container (see
# docker-compose.mac.yml XVFB=1). WSL2/Linux with a real DISPLAY skip it.
COPY scripts/docker-entrypoint.sh /usr/local/bin/robomaster-entrypoint.sh
RUN chmod +x /usr/local/bin/robomaster-entrypoint.sh \
    && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws
ENTRYPOINT ["/usr/local/bin/robomaster-entrypoint.sh"]
CMD ["bash"]
