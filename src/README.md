# Forklift AMR - Source Code Workspace (`src/`)

This directory contains all ROS 2 packages required to operate the **Forklift AMR**. The workspace is organized into custom core packages developed for the robot and external third-party hardware drivers/libraries.

---

## 📁 Workspace Directory Tree

```text
src/
├── car_assets/             # Saved 2D occupancy grid maps (.yaml, .pgm) generated via SLAM
├── car_controller/         # High-level control scripts, teleoperation, and pallet docking logic
├── car_main/               # Core bringup launch files and YAML configs (Nav2, AMCL, EKF, SLAM)
├── mecanum_odom/           # Forward Mecanum kinematics node converting ESP32 ticks to /odom
├── view_robot_model/       # Robot URDF/Xacro description, 3D meshes, and RViz2 model viewer
│
├── zed_display_rviz2/      # RViz2 visualization plugins for ZED2 (extracted from zed-ros2-examples)
└── [External Submodules]   # Hardware drivers & middleware (Micro-ROS, ZED2, LiDAR, etc.)
```

---

## 🛠️ Core Custom Packages

### 1. `car_main`
* **Role:** The central orchestration and configuration package of the AMR.
* **Contents:**
  * Master launch files to bring up sensors, sensor fusion, localization, and navigation stacks simultaneously.
  * Configuration `.yaml` files for **Nav2** (controllers, planners, STVL 3D costmaps), **AMCL**, **SLAM Toolbox**, and `robot_localization` (**EKF**).

### 2. `mecanum_odom`
* **Role:** Low-level odometry computation bridge.
* **Contents:**
  * Subscribes to raw encoder feedback (`/wheel_ticks`) from the ESP32 Micro-ROS node.
  * Computes 4-wheel Mecanum forward kinematics to publish raw wheel odometry (`/odom`) and wheel velocities for EKF sensor fusion.

### 3. `car_controller`
* **Role:** Robot motion control, hardware testing, and task execution.
* **Contents:**
  * Autonomous pallet docking routines and precision alignment test scripts.
  * Manual teleoperation nodes for hardware debugging and velocity validation.

### 4. `view_robot_model`
* **Role:** Digital twin and TF tree visualization.
* **Contents:**
  * URDF/Xacro files defining the forklift chassis, Mecanum wheel joints, lifting mast, and sensor frames (`base_link`, `laser`, `zed2_camera_center`).
  * Launch files to inspect the kinematic model and TF transforms in RViz2.

### 5. `car_assets`
* **Role:** Environment map storage.
* **Contents:**
  * Pre-built 2D occupancy grid maps (`.pgm` and `.yaml` metadata) recorded during SLAM mapping runs for static AMCL localization.

---

## 📦 Third-Party Drivers & Submodules

To keep the repository modular and lightweight, external hardware drivers are integrated as Git submodules or standalone packages:

| Package / Repository | Source / Type | Role in System |
| :--- | :--- | :--- |
| **`micro_ros_setup`** | Git Submodule | Establishes serial transport (`921600 baud`) between Jetson Orin Nano and ESP32 MCU. |
| **`zed-ros2-wrapper`** | Git Submodule | Official Stereolabs ROS 2 driver publishing ZED2 RGB-D point clouds, depth, and IMU data. |
| **`zed_display_rviz2`** | Standalone Package | Custom RViz2 display plugins for ZED2 objects/point clouds *(sourced from `stereolabs/zed-ros2-examples`)*. |
| **`sllidar_ros2`** | Git Submodule | Driver for the $360^\circ$ 2D planar LiDAR publishing `/scan` messages. |
| **`pointcloud_crop_box`** | Git Submodule | Filters and crops raw 3D point clouds to remove self-collisions (forklift mast/forks) before STVL costmap projection. |
| **`web_video_server`** | Git Submodule | Streams ROS 2 camera image topics over HTTP for remote web dashboard monitoring. |

---

## ⚙️ Cloning & Building the Workspace

Because this workspace relies on Git submodules for hardware drivers, clone with `--recursive` (or initialize submodules after cloning):

```bash
# 1. Initialize and update external submodules
git submodule update --init --recursive

# 2. Navigate to workspace root and install dependencies
cd ~/forklift-amr-ros2
rosdep install --from-paths src --ignore-src -r -y

# 3. Compile all packages
colcon build --symlink-install

# 4. Source the environment
source install/setup.bash
```

---

[< Back to Root README](../README.md)