FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-lc"]

# 1) Install base tools + camera drivers + utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    git curl nano vim x11-apps \
    python3-pip \
    ros-humble-usb-cam \
    ros-humble-openni2-camera \
    ros-humble-realsense2-camera \
    ros-humble-topic-tools \
    ros-humble-cv-bridge \
    python3-opencv \
    libgl1-mesa-dri libgl1-mesa-glx mesa-utils \
    ros-humble-web-video-server \
    && rm -rf /var/lib/apt/lists/*

# 2) Create ROS 2 workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# 3) Copy camera_launch package
COPY camera_launch/ /ros2_ws/src/camera_launch/

# 4) Copy entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 5) Build workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
