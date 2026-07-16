FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-gz-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joint-state-publisher \
# NOTE: no ros-humble-twist-mux — its binary links libdiagnostic_updater.so,
# which no ros-humble-diagnostic-updater in the repos ships. robomaster_bringup
# has a small cmd_vel_mux instead.

    ros-humble-apriltag-ros \
    ros-humble-apriltag-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-proc \
    # Video is watched in a browser (web_video_server), not an X11 GUI — no rqt.
    ros-humble-web-video-server \
    python3-colcon-common-extensions \
    python3-av \
    python3-opencv \
    ffmpeg \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws