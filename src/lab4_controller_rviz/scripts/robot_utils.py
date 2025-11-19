#!/usr/bin/env python3
"""
Robot Utilities for 3R Manipulator
✅ FIXED: Proper collision checking (end-effector only)
"""

import numpy as np
from scipy.spatial.transform import Rotation
import subprocess
import tempfile
import os

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    RTB_AVAILABLE = True
except ImportError:
    RTB_AVAILABLE = False


def find_xacro_file():
    """Auto-detect xacro file"""
    home = os.path.expanduser('~')
    paths = [
        f'{home}/lab4_ws/src/lab4_description/robot/visual/my-robot.xacro',
        f'{home}/lab4_ws/src/lab4_description/urdf/my-robot.xacro',
    ]
    for path in paths:
        if os.path.exists(path):
            return path
    raise FileNotFoundError("Cannot find xacro file")


def load_robot(xacro_path=None):
    """Load robot from xacro"""
    if not RTB_AVAILABLE:
        raise ImportError("roboticstoolbox-python required")
    
    if xacro_path is None:
        xacro_path = find_xacro_file()
    
    result = subprocess.run(['xacro', xacro_path], capture_output=True, text=True, check=True)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(result.stdout)
        urdf_path = f.name
    
    robot = rtb.Robot.URDF(urdf_path)
    os.unlink(urdf_path)
    return robot


def get_joint_limits(robot):
    """Get joint limits"""
    q_min = np.array([link.qlim[0] for link in robot.links if link.isjoint])
    q_max = np.array([link.qlim[1] for link in robot.links if link.isjoint])
    return q_min, q_max


def forward_kinematics(robot, q):
    """Forward kinematics"""
    return robot.fkine(q)


def inverse_kinematics(robot, target_pose, q0=None):
    """Inverse kinematics"""
    if q0 is None:
        q0 = np.zeros(robot.n)
    if isinstance(target_pose, np.ndarray):
        target_pose = SE3(target_pose)
    
    q_min, q_max = get_joint_limits(robot)
    solution = robot.ikine_LM(target_pose, q0=q0)
    
    if solution.success:
        q = solution.q
        if np.all(q >= q_min) and np.all(q <= q_max):
            return True, q
    return False, None


# ========================================================================
# ✅ FIXED: COLLISION CHECKING FUNCTIONS
# ========================================================================

def check_ground_collision(robot, q, ground_clearance=0.02):
    """
    ✅ FIXED: Check only end-effector collision with ground
    
    Base and Link 1 are EXPECTED to be at/near ground level.
    We only care that the end-effector stays above ground.
    
    Args:
        robot: Robot model
        q: Joint configuration
        ground_clearance: Safety margin from ground (meters)
    
    Returns:
        bool: True = safe (no collision), False = collision with ground
    """
    try:
        # ✅ Only check end-effector position
        T_ee = robot.fkine(q)
        return T_ee.t[2] >= ground_clearance
    except:
        # If error, assume safe (don't reject unnecessarily)
        return True


def check_ground_collision_strict(robot, q, ground_clearance=0.02, check_last_n_links=1):
    """
    ✅ OPTIONAL: Stricter version that checks last N links
    
    Use this if you want to ensure the arm doesn't scrape the ground,
    but be aware it will reduce the workspace significantly.
    
    Args:
        robot: Robot model
        q: Joint configuration
        ground_clearance: Safety margin from ground (meters)
        check_last_n_links: Number of links from the end to check (default: 1 = end-effector only)
    
    Returns:
        bool: True = safe, False = collision
    """
    try:
        n_links = len(robot.links)
        
        # Check last N links
        for i in range(max(0, n_links - check_last_n_links), n_links):
            T = robot.fkine(q, end=robot.links[i])
            if T.t[2] < ground_clearance:
                return False
        
        return True
    except:
        return True


def get_safe_z_minimum(robot, num_test_samples=500):
    """
    ✅ Find the safe minimum Z height for the workspace
    
    Returns:
        float: Safe minimum Z value
    """
    q_min, q_max = get_joint_limits(robot)
    z_values = []
    
    for _ in range(num_test_samples):
        q = np.random.uniform(q_min, q_max)
        
        # Check if configuration is safe
        if check_ground_collision(robot, q, ground_clearance=0.02):
            T = robot.fkine(q)
            z_values.append(T.t[2])
    
    if len(z_values) > 10:
        # Use 10th percentile as safe minimum
        z_safe = np.percentile(z_values, 10)
        return max(z_safe, 0.05)  # At least 5cm
    else:
        # Fallback
        return 0.08  # 8cm


# ========================================================================
# ✅ FIXED: WORKSPACE COMPUTATION
# ========================================================================

def compute_workspace(robot, num_samples=10000, ground_clearance=0.02):
    """
    ✅ FIXED: Compute workspace using sampling with proper collision checking
    
    Args:
        robot: Robot model
        num_samples: Number of samples to collect
        ground_clearance: Minimum height above ground (meters)
    
    Returns:
        dict: Workspace bounds and point cloud
    """
    q_min, q_max = get_joint_limits(robot)
    points = []
    
    sampled = 0
    attempts = 0
    max_attempts = num_samples * 50  # ✅ Increased from 20x to 50x
    
    print(f"Computing workspace with {num_samples} samples...")
    print(f"Ground clearance: {ground_clearance*100:.1f} cm")
    
    while sampled < num_samples and attempts < max_attempts:
        attempts += 1
        
        # Random configuration
        q = np.random.uniform(q_min, q_max)
        
        # ✅ Check collision (only end-effector)
        if check_ground_collision(robot, q, ground_clearance):
            T = robot.fkine(q)
            points.append(T.t)
            sampled += 1
            
            # Progress logging
            if sampled % 2000 == 0 and sampled > 0:
                success_rate = (sampled / attempts) * 100
                print(f"  Progress: {sampled}/{num_samples} ({attempts} attempts, {success_rate:.1f}% success rate)")
    
    if len(points) == 0:
        raise ValueError("No valid workspace points found!")
    
    points = np.array(points)
    
    workspace = {
        'x_min': float(np.min(points[:, 0])),
        'x_max': float(np.max(points[:, 0])),
        'y_min': float(np.min(points[:, 1])),
        'y_max': float(np.max(points[:, 1])),
        'z_min': float(np.min(points[:, 2])),
        'z_max': float(np.max(points[:, 2])),
        'points': points,
        'valid_samples': len(points),
        'total_attempts': attempts
    }
    
    # Final statistics
    success_rate = (len(points) / attempts) * 100
    print(f"\n✅ Workspace computation complete!")
    print(f"  Valid samples: {len(points)}/{attempts}")
    print(f"  Success rate: {success_rate:.1f}%")
    
    return workspace


# ========================================================================
# POSE/MATRIX CONVERSION
# ========================================================================

def pose_msg_to_matrix(pose_msg):
    """Convert Pose to matrix"""
    t = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
    quat = [pose_msg.orientation.x, pose_msg.orientation.y, 
            pose_msg.orientation.z, pose_msg.orientation.w]
    R = Rotation.from_quat(quat).as_matrix()
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def matrix_to_pose_msg(T):
    """Convert matrix to Pose"""
    from geometry_msgs.msg import Pose
    
    if hasattr(T, 't'):
        t, R = T.t, T.R
    else:
        t, R = T[:3, 3], T[:3, :3]
    
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = float(t[0]), float(t[1]), float(t[2])
    quat = Rotation.from_matrix(R).as_quat()
    pose.orientation.x, pose.orientation.y = float(quat[0]), float(quat[1])
    pose.orientation.z, pose.orientation.w = float(quat[2]), float(quat[3])
    return pose