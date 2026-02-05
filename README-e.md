# Camera Input
ros-humble-cameras

## Purpose
- Operate multiple camera inputs (Webcam / Xtion / RealSense) on ROS 2 Humble using Docker Compose
- Persist camera_info (calibration yaml) on the host

## Supported Cameras
- Webcam (UVC: /dev/video*)
- ASUS Xtion (OpenNI2)
- Intel RealSense (optional: if installed)

## Directory Structure
```
ros-humble-cameras/
  compose.yaml
  Dockerfile
  entrypoint.sh
  launch/
    webcam.launch.py
    xtion.launch.py
    realsense.launch.py
    web_video_server.launch.py
    cv_viewer.launch.py
  persist/
    camera_info/
      webcam/
      xtion/
      realsense/
```

## 1. Prerequisites
```
git clone https://github.com/tidbots/ros-humble-cameras.git
cd ros-humble-cameras
docker compose build
```

### 1-1. Create Host Directories (for camera_info persistence)
```
$ mkdir -p persist/camera_info/webcam
$ mkdir -p persist/camera_info/xtion
$ mkdir -p persist/camera_info/realsense
```

### 1-2. Device Verification
#### Webcam
```
$ ls -l /dev/video*
crw-rw----+ 1 root video 81, 0  Jan 22 11:14 /dev/video0
crw-rw----+ 1 root video 81, 1  Jan 22 11:14 /dev/video1
crw-rw----+ 1 root video 81, 2  Jan 22 11:14 /dev/video2
crw-rw----+ 1 root video 81, 3  Jan 22 11:14 /dev/video3
```

#### Xtion / RealSense
Verify USB device is visible (Linux)
```
$ lsusb
...
Bus 003 Device 005: ID 1d27:0601 ASUS Xtion
...
```

## 2. Startup Methods
### 2-1. Start Webcam
```
docker compose --profile webcam up
```
Change webcam parameters at startup
```
CAM_DEV=/dev/video2 CAM_W=640 CAM_H=480 CAM_FPS=15 docker compose --profile webcam up
```
Change output topics (usually not needed)
```
export OUT_IMAGE=/camera/image_raw OUT_INFO=/camera/camera_info
docker compose -f compose.yaml --profile webcam up --build
```


### 2-2. Start Xtion
```
docker compose --profile xtion up
```
If Xtion's "source topic" differs in your environment:
```
export XTION_IN_IMAGE=/xtion/rgb/image_raw
export XTION_IN_INFO=/xtion/rgb/camera_info
docker compose -f compose.yaml --profile xtion up --build
```

### 2-3. Start RealSense
```
docker compose --profile realsense up
```

### Stop:
```
docker compose down
```

## 3. Debugging
### Check with OpenCV Viewer (Recommended)
```
docker compose --profile webcam --profile view up
```
- Press `q` or `ESC` to exit

### Check in Browser
```
docker compose --profile webcam --profile debug up
```

On the same machine:
```
http://localhost:8080/
```
```
Direct link: http://localhost:8080/stream?topic=/camera/image_raw
```

From another PC, replace localhost with the hostname


## 4. Persisting camera_info (Calibration)
### 4-1. How Persistence Works
Each camera container mounts the following to the host:
- Container: /root/.ros/camera_info
- Host: ./persist/camera_info/<camera>/

This keeps calibration yaml files on the host, preserved across restarts.

### 4-2. Loading Webcam camera_info
Webcam loads yaml via camera_info_url specified in webcam.launch.py.

#### Example (in compose.yaml):
```
"camera_info_url:=file:///root/.ros/camera_info/webcam.yaml"
```

#### Actual file on host:
```
./persist/camera_info/webcam/webcam.yaml
```

## 5. How to Change Camera Parameters
### Basic Approach:

- Define DeclareLaunchArgument in launch/*.launch.py
- Pass arg:=value in compose.yaml command

### 5-1. Webcam Example
Modify the command for camera_webcam in compose.yaml.

#### Example: Change resolution and FPS
```
"width:=640", "height:=480", "fps:=15"
```

#### Example: Change device
```
"device:=/dev/video2"
```

### 5-2. Xtion Example
Modify the command for camera_xtion.

## 6. Common Issues and Solutions
### 6-1. Topics Not Publishing
- Is the correct profile specified?
- Is the device visible? (/dev/video0, /dev/bus/usb)

#### Verify:
```
ros2 topic list
```

### 6-2. RealSense Topic Names Differ
RealSense topic names may vary by environment.

Check actual names with the following and adjust realsense.launch.py arguments accordingly.
```
ros2 topic list | grep realsense
```

## 7. Recommended Usage (for downstream YOLO, etc.)
- Downstream should subscribe only to /camera/image_raw (and Depth topics if needed)
- Camera switching is handled by relay
- Fix camera_info with persist for reproducibility
