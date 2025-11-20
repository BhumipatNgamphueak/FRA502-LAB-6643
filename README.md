# LAB4: 3R Manipulator Control System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)

ROS2 implementation for controlling a 3-DOF robotic manipulator with three control modes.

## Installation
```bash
mkdir -p ~/lab4_ws/src
cd ~/lab4_ws/src
# Place lab4_controller_rviz and lab4_description packages here
# lab4_controller_rviz,lab4_description packages and workspace_finder.py should be inside /src 
```

**Verify packages are in place:**
```bash
ls ~/lab4_ws/src/
# Expected output: lab4_controller_rviz  lab4_description
```

**Build workspace:**
```bash
cd ~/lab4_ws
colcon build 
source install/setup.bash
```

**Make sourcing permanent (optional):**
```bash
echo "source ~/lab4_ws/install/setup.bash" >> ~/.bashrc
```

## Quick Start
```bash
cd ~/lab4_ws
source install/setup.bash
ros2 launch lab4_controller_rviz lab4_rviz_launch.py
```

## Workspace Analysis
Analyze and visualize the robot's reachable workspace:
```bash
ros2 run lab4_controller_rviz workspace_analyzer_node.py
```
Publishes point cloud to `/workspace_points` topic (visualize in RViz). Computes workspace bounds with ground collision avoidance.

### Standalone Workspace Finder (Recommended)
For clear numerical boundaries without point clouds, use the standalone script:

**Option 1 - From ROS workspace:**
```bash
cd ~/lab4_ws/src
python3 workspace_finder.py
```

**Option 2 - From anywhere (if in PATH):**
```bash
python3 workspace_finder.py
```

**Option 3 - Download and run:**
```bash
# Download the script to your workspace
cd ~/lab4_ws/src
# Run it
python3 workspace_finder.py
```

**Confirmed Workspace Boundaries (from analysis):**
```
X-axis (Left/Right):  [-0.5293,  0.5266] m  (range: 1.0558 m)
Y-axis (Front/Back):  [-0.5277,  0.5288] m  (range: 1.0565 m)
Z-axis (Up/Down):     [ 0.0206,  0.7297] m  (range: 0.7092 m)
Workspace Volume:     0.791121 m³
```

**Recommended Safe Zones (85% margin for target generation):**
```
X: [-0.4501,  0.4474] m
Y: [-0.4485,  0.4496] m
Z: [ 0.0737,  0.6765] m
```

✅ **Ground Safety:** All points verified ≥2cm above ground (Z_min = 0.0206m)

The script generates `workspace_analysis.png` with 3D, top, and side views for visual confirmation.

## Singularity Detection
During teleoperation, singularity warnings are **automatically displayed** in the controller terminal. When a near-singularity configuration is detected, the controller immediately stops motion and displays a warning message.

**Example output:**
```
[WARN] [controller]: ⚠️ TO: Near singularity! σ_min=0.008234
[WARN] [controller]: TO: Motion stopped due to singularity
```

**Optional:** Monitor warnings via topic for logging or debugging:
```bash
ros2 topic echo /singularity_warning
```

The controller uses Singular Value Decomposition (SVD) to compute the minimum singular value and stops motion when it falls below the threshold (default: 0.01).

## Control Modes

### Mode 0: IPK (Inverse Position Kinematics)
```bash
ros2 service call /set_control_mode lab4_controller_rviz/srv/SetControlMode "{mode: 0}"
ros2 service call /compute_ik lab4_controller_rviz/srv/ComputeIK "{target_pose: {position: {x: 0.2, y: 0.1, z: 0.3}, orientation: {w: 1.0}}}"
```

**Response Messages:**
- **Success:** `success: true`, `message: "Position reached (error: 0.0012m)"`
- **Failure:** `success: false`, `message: "No solution found"`

**Terminal Output:**
```
[INFO] [controller]: IPK: Success - error=0.0012m, q=[0.123, 0.456, 0.789]
```
or
```
[WARN] [controller]: IPK: Failed to find solution
```

### Mode 1: Teleoperation

**Terminal 1:**
```bash
ros2 service call /set_control_mode lab4_controller_rviz/srv/SetControlMode "{mode: 1}"
```

**Terminal 2:**
```bash
ros2 run lab4_controller_rviz teleop_jog_keyboard.py
```

**Keyboard Controls:**
```
Linear Motion:          Angular Motion:
   W (X+)                 U   I   O
 A   D                    J   K   L
   S (X-)              

W/S: Forward/Back       U/O: Yaw
A/D: Left/Right         I/K: Roll  
Q/E: Up/Down            J/L: Pitch

F: Toggle Frame (World ↔ End-Effector)
SPACE: Emergency Stop
ESC: Quit
```

### Mode 2: Autonomous
```bash
ros2 service call /set_control_mode lab4_controller_rviz/srv/SetControlMode "{mode: 2}"
```
Robot automatically moves to random safe targets.
