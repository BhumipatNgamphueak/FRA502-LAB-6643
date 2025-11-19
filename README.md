# LAB4: 3R Manipulator Control System

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10+-green.svg)](https://www.python.org/)

ROS2 implementation for controlling a 3-DOF robotic manipulator with three control modes.

## Installation
```bash
mkdir -p ~/lab4_ws/src
cd ~/lab4_ws/src
# Place lab4_controller_rviz and lab4_description packages here
cd ~/lab4_ws
colcon build --symlink-install
source install/setup.bash
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
ros2 run lab4_controller_rviz workspace_analyzer_node
```
Publishes point cloud to `/workspace_points` topic (visualize in RViz). Computes workspace bounds with ground collision avoidance.

## Control Modes

### Mode 0: IPK (Inverse Position Kinematics)
```bash
ros2 service call /set_control_mode lab4_controller_rviz/srv/SetControlMode "{mode: 0}"
ros2 service call /compute_ik lab4_controller_rviz/srv/ComputeIK "{target_pose: {position: {x: 0.2, y: 0.1, z: 0.3}, orientation: {w: 1.0}}}"
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
### Mode 2: Autonomous
```bash
ros2 service call /set_control_mode lab4_controller_rviz/srv/SetControlMode "{mode: 2}"
```
Robot automatically moves to random safe targets.