# BFMC SimulatorROS2 — Complete Setup Guide

Complete guide for installing, configuring and running the BFMC simulator with ROS2 Jazzy and Gazebo Harmonic, including the simulated OAK-D camera integration.

---

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [Installing Dependencies](#2-installing-dependencies)
3. [Workspace Setup](#3-workspace-setup)
4. [Gazebo Configuration](#4-gazebo-configuration)
5. [Building the Workspace](#5-building-the-workspace)
6. [Launching the Simulation](#6-launching-the-simulation)
7. [Vehicle Control](#7-vehicle-control)
8. [Simulated OAK-D Camera](#8-simulated-oak-d-camera)
9. [PointCloud2](#9-pointcloud2)
10. [Visualization with RViz2 and rqt](#10-visualization-with-rviz2-and-rqt)
---

## 1. System Requirements

- **OS:** Ubuntu 24.04 LTS (Noble)
- **ROS2:** Jazzy Jalisco
- **RAM:** 8GB minimum (16GB recommended)
- **GPU:** any (Intel Iris Xe or better supported)

---

## 2. Installing Dependencies

### 2.1 ROS2 Jazzy

Follow the official guide: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debians.html

```bash
# Add the ROS2 repository
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-jazzy-desktop -y
```

### 2.2 Gazebo Harmonic (via ROS2 Jazzy vendor packages)

> Gazebo Harmonic is available through the ROS2 Jazzy vendor packages.

```bash
sudo apt install ros-jazzy-ros-gz -y
```

### 2.3 Additional ROS2 dependencies

```bash
sudo apt install \
  ros-jazzy-depth-image-proc \
  ros-jazzy-rqt-image-view \
  ros-jazzy-rqt \
  python3-colcon-common-extensions \
  python3-rosdep \
  -y
```

### 2.4 Just (task runner)

```bash
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh \
  | bash -s -- --to ~/.local/bin

# Add to PATH
echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc
source ~/.bashrc

# Verify
just --version
```

---

## 3. Workspace Setup

```bash
# Create the working directory
mkdir -p ~/BFMC/
cd ~/BFMC/

# Clone the repository
git clone https://github.com/UniPD-DriveOps-BFMC/SimulatorROS2.git
```

---

## 4. Gazebo Configuration

### 4.1 Configuration directory

```bash
mkdir -p ~/.gz/sim/8
```

### 4.2 gui.config file

The `gui.config` file is already included in the cloned repository:

```bash
cp ~/BFMC/SimulatorROS2/"aditional files"/gui.config ~/.gz/sim/8/gui.config
```

### 4.3 Fix permissions and Ogre cache

```bash
sudo chown -R $USER:$USER ~/.gz
rm -rf ~/.gz/rendering/ogre-rtshader/*
mkdir -p ~/.gz/rendering/ogre-rtshader/hisham-rtshaderlibcache
```

---

## 5. Building the Workspace

> ⚠️ **IMPORTANT:** The build must be run from the **parent** directory of `SimulatorROS2` (i.e. `~/BFMC`), not from inside the repo.

```bash
cd ~/BFMC/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

The build creates the `~/BFMC/install/` directory which is used by all launch files.

---

## 6. Launching the Simulation

Open a **fresh terminal** (important: no prior `source` commands active):

```bash
cd ~/BFMC/SimulatorROS2
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash
```

### 6.1 Map and robot only 

```bash
just car
```

Starts: Gazebo, robot, ROS2↔Gazebo bridge, OAK-D mono converter, PointCloud2.

### 6.2 Map + all objects (signs, traffic lights, pedestrians, obstacle cars)

```bash
just all-objects
```

Note: all the objects are in the wrong positions, since they are referred to the old map

### 6.3 Monitor RAM usage

In a second terminal:

```bash
watch -n 2 free -h
```

### 6.4 Free RAM if needed

```bash
pkill -9 gz; pkill -9 ruby; pkill -9 ros2; pkill -9 python3
sudo sync && sudo sysctl vm.drop_caches=3
free -h
```

---

## 7. Vehicle Control

With the simulation running, open a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/BFMC/install/setup.bash
```

### 7.1 Steering

```bash
# Steer right (positive angle in degrees, range -25/+25)
ros2 topic pub /automobile/command std_msgs/msg/String \
  "data: '{\"action\": \"2\", \"steerAngle\": 15.0}'" --once

# Steer left
ros2 topic pub /automobile/command std_msgs/msg/String \
  "data: '{\"action\": \"2\", \"steerAngle\": -15.0}'" --once

# Center steering
ros2 topic pub /automobile/command std_msgs/msg/String \
  "data: '{\"action\": \"2\", \"steerAngle\": 0.0}'" --once
```

### 7.2 Speed

```bash
# Move forward (speed in m/s)
ros2 topic pub /automobile/command std_msgs/msg/String \
  "data: '{\"action\": \"1\", \"speed\": 0.05}'" --once

# Brake / stop
ros2 topic pub /automobile/command std_msgs/msg/String \
  "data: '{\"action\": \"1\", \"speed\": 0.0}'" --once

# Reverse
ros2 topic pub /automobile/command std_msgs/msg/String \
  "data: '{\"action\": \"1\", \"speed\": -0.05}'" --once
```

### 7.3 Keyboard control (built-in example)

```bash
just control_example
# W/A/S/D to move, ESC to exit
```

### 7.4 Vehicle topics reference

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/automobile/command` | `std_msgs/String` | ROS2 → Gazebo | Steering / speed commands |
| `/automobile/feedback` | `std_msgs/String` | Gazebo → ROS2 | Vehicle feedback |
| `/automobile/IMU` | `geometry_msgs/Vector3` | Gazebo → ROS2 | Roll, pitch, yaw |
| `/automobile/localisation` | `geometry_msgs/Pose` | Gazebo → ROS2 | GPS position |

---

## 8. Simulated OAK-D Camera

The simulation includes a full OAK-D camera with the following sensors:

| Topic | Encoding | Resolution | Hz | Description |
|---|---|---|---|---|
| `/automobile/image_raw` | rgb8 | 640×480 | ~19 | Original camera (backward compat.) |
| `/oak/rgb/image_raw` | rgb8 | 1920×1080 | ~19 | RGB (simulated IMX378) |
| `/oak/rgb/camera_info` | CameraInfo | — | ~19 | RGB calibration |
| `/oak/stereo/image_raw` | 32FC1 | 1280×800 | ~20 | Depth in meters (float32) |
| `/oak/stereo/image_fixed` | 32FC1 | 1280×800 | ~20 | Depth with corrected frame_id |
| `/oak/stereo/camera_info` | CameraInfo | — | ~20 | Depth calibration |
| `/oak/left/image_raw` | rgb8 | 640×400 | ~23 | Left mono raw |
| `/oak/left/image_mono` | mono8 | 640×400 | ~23 | Left mono grayscale (OV9282) |
| `/oak/left/camera_info` | CameraInfo | — | ~23 | Left mono calibration |
| `/oak/right/image_raw` | rgb8 | 640×400 | ~23 | Right mono raw |
| `/oak/right/image_mono` | mono8 | 640×400 | ~23 | Right mono grayscale (OV9282) |
| `/oak/right/camera_info` | CameraInfo | — | ~23 | Right mono calibration |

### 8.1 Check active topics

```bash
ros2 topic list | grep oak
```

### 8.2 Check encoding and resolution

```bash
# RGB
ros2 topic echo /oak/rgb/image_raw --once | grep -E "height|width|encoding"

# Depth
ros2 topic echo /oak/stereo/image_raw --once | grep -E "height|width|encoding"

# Left mono (should be mono8)
ros2 topic echo /oak/left/image_mono --once | grep -E "height|width|encoding"
```

### 8.3 Check frame rates

```bash
ros2 topic hz /oak/rgb/image_raw &
ros2 topic hz /oak/stereo/image_raw &
ros2 topic hz /oak/left/image_mono &
wait
```

---

## 9. PointCloud2

The XYZRGB PointCloud2 is generated automatically by combining depth + RGB using `depth_image_proc`.

| Topic | Type | Hz | Description |
|---|---|---|---|
| `/oak/points` | `sensor_msgs/PointCloud2` | ~12 | XYZRGB PointCloud |

### 9.1 Verify

```bash
ros2 topic info /oak/points
ros2 topic hz /oak/points
```

### 9.2 Visualize in RViz2

```bash
rviz2
```

Inside RViz2:
1. Click **Add** → **By topic** → `/oak/points` → **PointCloud2** → OK
2. Set **Fixed Frame** by typing manually: `automobile/camera/link_camera/oak_rgb`
3. In the PointCloud2 panel set **Color Transformer** → `RGB8`

---

## 10. Visualization with RViz2 and rqt

### 10.1 View all cameras with rqt

```bash
ros2 run rqt_image_view rqt_image_view
```

Select the desired topic from the dropdown:
- `/oak/rgb/image_raw` — color RGB image
- `/oak/stereo/image_raw` — depth map (grayscale)
- `/oak/left/image_mono` — left grayscale mono
- `/oak/right/image_mono` — right grayscale mono

### 10.2 Gazebo navigation controls

| Action | Control |
|---|---|
| Zoom | Mouse scroll wheel |
| Rotate view | Left click + drag |
| Pan | Middle click + drag / Shift + left click + drag |
| Focus on object | Click object in Entity Tree → press F |
| Reset view | Home key |

---

## Package Structure

```
~/BFMC/SimulatorROS2/src/
├── sim_pkg/                  # Launch files and world files
│   ├── launch/
│   │   ├── map_with_car.launch          # just car
│   │   ├── map_with_car_light.launch    # lightweight version
│   │   ├── all_objects.launch           # all objects
│   │   └── bridge.launch                # ROS2↔Gazebo bridge
│   └── worlds/
│       └── world_with_separators.world  # main world
├── models_pkg/               # 3D models (signs, vehicle, traffic lights...)
├── car_plugin/               # C++ vehicle control plugin
├── bno055_plugin/            # IMU plugin
├── gps_plugin/               # GPS plugin
├── traffic_light_plugin/     # Traffic light plugin
├── traffic_light_pkg/        # ROS2 node for traffic light logic
├── oak_mono_converter/       # Python node: RGB→mono8 + depth frame fix
├── example/                  # Control examples (keyboard, camera)
└── utils/                    # Utilities
```

---

## Quick Reference

```bash
# Launch simulation (map + robot only)
cd ~/BFMC/SimulatorROS2 && source /opt/ros/jazzy/setup.bash && source ../install/setup.bash && just car

# Launch full simulation (all objects, lightweight)
just all-objects-light

# Move robot forward
ros2 topic pub /automobile/command std_msgs/msg/String "data: '{\"action\": \"1\", \"speed\": 0.05}'" --once

# Steer right
ros2 topic pub /automobile/command std_msgs/msg/String "data: '{\"action\": \"2\", \"steerAngle\": 15.0}'" --once

# Stop
ros2 topic pub /automobile/command std_msgs/msg/String "data: '{\"action\": \"1\", \"speed\": 0.0}'" --once

# View camera feed
ros2 run rqt_image_view rqt_image_view

# List all active topics
ros2 topic list

# Check camera frame rate
ros2 topic hz /oak/rgb/image_raw

# Free RAM and restart
pkill -9 gz; pkill -9 ruby; pkill -9 ros2; pkill -9 python3
sudo sync && sudo sysctl vm.drop_caches=3
```

