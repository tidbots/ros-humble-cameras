# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 Humble multi-camera input system supporting Webcam (UVC), ASUS Xtion (OpenNI2), and Intel RealSense cameras with Docker containerization. All cameras output to unified topics `/camera/image_raw` and `/camera/camera_info`.

## Build Commands

```bash
# Build Docker image
docker compose build

# Start with specific camera profile
docker compose --profile webcam up
docker compose --profile xtion up
docker compose --profile realsense up

# Stop
docker compose down
```

## Debug Utilities

```bash
# OpenCV viewer (q or ESC to exit)
docker compose --profile webcam --profile view up

# Web browser viewer (http://localhost:8080/)
docker compose --profile webcam --profile debug up
```

## Architecture

- **camera_launch/** - ROS 2 package with launch files and cv_viewer node
- **launch/** - Host-mounted launch files (editable without rebuild)
- **persist/camera_info/** - Persistent calibration storage per camera
- **compose.yaml** - Docker Compose profiles for each camera type
- ROS 2 uses DDS (no roscore needed)
- Topic relay nodes unify different camera outputs to `/camera/image_raw`

## Key Differences from ROS 1 (Noetic) Version

- Launch files are Python (`.launch.py`) instead of XML
- Uses `rclpy` instead of `rospy`
- Uses `ament_cmake` instead of `catkin`
- Uses `ros2 launch` instead of `roslaunch`
- Uses `ros2 topic` instead of `rostopic`
