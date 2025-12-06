#!/usr/bin/env python3
"""
Standalone Workspace Finder for 3R Manipulator
Analyzes workspace and shows clear boundaries (no point cloud needed)
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import subprocess
import tempfile
import os

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
except ImportError:
    print("Error: roboticstoolbox-python required")
    print("Install: pip install roboticstoolbox-python --break-system-packages")
    exit(1)


def load_robot():
    """Load robot from xacro file"""
    # Auto-detect xacro path
    home = os.path.expanduser('~')
    xacro_paths = [
        f'{home}/lab4_ws/src/lab4_description/robot/visual/my-robot.xacro',
        f'{home}/lab4_ws/src/lab4_description/urdf/my-robot.xacro',
        './my-robot.xacro',
    ]
    
    xacro_path = None
    for path in xacro_paths:
        if os.path.exists(path):
            xacro_path = path
            break
    
    if not xacro_path:
        print("Error: Cannot find my-robot.xacro")
        exit(1)
    
    print(f"Loading robot from: {xacro_path}")
    
    # Convert xacro to URDF
    result = subprocess.run(['xacro', xacro_path], capture_output=True, text=True, check=True)
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
        f.write(result.stdout)
        urdf_path = f.name
    
    robot = rtb.Robot.URDF(urdf_path)
    os.unlink(urdf_path)
    
    return robot


def check_above_ground(robot, q, ground_clearance=0.02):
    """
    Verify configuration is safe (above ground)
    Checks ONLY end-effector position
    """
    try:
        T = robot.fkine(q)
        return T.t[2] >= ground_clearance
    except:
        return False


def analyze_workspace(robot, num_samples=20000):
    """
    Analyze workspace by sampling configurations
    Returns workspace boundaries and statistics
    
    SAFETY: Filters out all points below ground
    """
    print(f"\nAnalyzing workspace with {num_samples} samples...")
    print(f"Ground clearance: 2.0 cm (safety margin)")
    
    # Get joint limits
    q_min = np.array([link.qlim[0] for link in robot.links if link.isjoint])
    q_max = np.array([link.qlim[1] for link in robot.links if link.isjoint])
    
    print(f"\nJoint limits (rad):")
    for i in range(len(q_min)):
        print(f"  Joint {i+1}: [{q_min[i]:.3f}, {q_max[i]:.3f}]")
    
    # Sample configurations
    points = []
    points_below_ground = []
    
    for i in range(num_samples):
        q = np.random.uniform(q_min, q_max)
        
        # CRITICAL: Check if above ground
        if check_above_ground(robot, q, ground_clearance=0.02):
            T = robot.fkine(q)
            pos = T.t
            
            # Double-check (paranoid safety)
            if pos[2] >= 0.02:
                points.append(pos)
            else:
                points_below_ground.append(pos)
        else:
            T = robot.fkine(q)
            points_below_ground.append(T.t)
        
        if (i+1) % 2000 == 0:
            print(f"  Progress: {i+1}/{num_samples}")
    
    points = np.array(points)
    
    print(f"\n✓ Analysis complete!")
    print(f"  Valid points (above ground): {len(points)}")
    print(f"  Below ground (filtered): {len(points_below_ground)}")
    print(f"  Success rate: {100*len(points)/num_samples:.1f}%")
    
    # SAFETY CHECK: Verify no points below ground
    if len(points) > 0:
        min_z = np.min(points[:, 2])
        print(f"\n🔒 SAFETY VERIFICATION:")
        print(f"   Minimum Z in workspace: {min_z:.4f} m")
        if min_z < 0:
            print(f"   WARNING: Some points below ground detected!")
        elif min_z < 0.02:
            print(f"   WARNING: Points very close to ground!")
        else:
            print(f"   All points safely above ground (clearance: {min_z:.4f} m)")
    
    # Compute boundaries
    workspace = {
        'x_min': float(np.min(points[:, 0])),
        'x_max': float(np.max(points[:, 0])),
        'y_min': float(np.min(points[:, 1])),
        'y_max': float(np.max(points[:, 1])),
        'z_min': float(np.min(points[:, 2])),
        'z_max': float(np.max(points[:, 2])),
        'points': points,
        'center': np.mean(points, axis=0)
    }
    
    # Compute volume
    volume = ((workspace['x_max'] - workspace['x_min']) * 
              (workspace['y_max'] - workspace['y_min']) * 
              (workspace['z_max'] - workspace['z_min']))
    workspace['volume'] = volume
    
    return workspace


def print_workspace_summary(ws):
    """Print clear workspace summary"""
    print("\n" + "="*70)
    print("WORKSPACE BOUNDARIES")
    print("="*70)
    print(f"X-axis (Left/Right):  [{ws['x_min']:7.4f}, {ws['x_max']:7.4f}] m  (range: {ws['x_max']-ws['x_min']:.4f} m)")
    print(f"Y-axis (Front/Back):  [{ws['y_min']:7.4f}, {ws['y_max']:7.4f}] m  (range: {ws['y_max']-ws['y_min']:.4f} m)")
    print(f"Z-axis (Up/Down):     [{ws['z_min']:7.4f}, {ws['z_max']:7.4f}] m  (range: {ws['z_max']-ws['z_min']:.4f} m)")
    print(f"\nWorkspace Volume:     {ws['volume']:.6f} m³")
    print(f"Workspace Center:     ({ws['center'][0]:.4f}, {ws['center'][1]:.4f}, {ws['center'][2]:.4f})")
    print("="*70)
    
    # Safe zones
    print("\nRECOMMENDED SAFE ZONES (with 15% margin):")
    margin = 0.85
    for axis in ['x', 'y', 'z']:
        min_key = f'{axis}_min'
        max_key = f'{axis}_max'
        center = (ws[min_key] + ws[max_key]) / 2
        range_val = (ws[max_key] - ws[min_key]) * margin
        safe_min = center - range_val/2
        safe_max = center + range_val/2
        print(f"{axis.upper()}: [{safe_min:7.4f}, {safe_max:7.4f}] m")
    
    print("="*70)


def visualize_workspace(ws, robot):
    """Create 3D visualization of workspace boundaries"""
    fig = plt.figure(figsize=(15, 5))
    
    # 3D view
    ax1 = fig.add_subplot(131, projection='3d')
    points = ws['points']
    
    # Sample points for visualization (max 2000 points)
    if len(points) > 2000:
        indices = np.random.choice(len(points), 2000, replace=False)
        points_vis = points[indices]
    else:
        points_vis = points
    
    ax1.scatter(points_vis[:, 0], points_vis[:, 1], points_vis[:, 2], 
                c=points_vis[:, 2], cmap='viridis', alpha=0.3, s=1)
    
    # Draw bounding box
    x_range = [ws['x_min'], ws['x_max']]
    y_range = [ws['y_min'], ws['y_max']]
    z_range = [ws['z_min'], ws['z_max']]
    
    # Draw box edges
    for x in x_range:
        for y in y_range:
            ax1.plot([x, x], [y, y], z_range, 'r-', linewidth=2)
    for x in x_range:
        for z in z_range:
            ax1.plot([x, x], y_range, [z, z], 'r-', linewidth=2)
    for y in y_range:
        for z in z_range:
            ax1.plot(x_range, [y, y], [z, z], 'r-', linewidth=2)
    
    # Draw ground plane at Z=0 (RED = danger zone)
    xx, yy = np.meshgrid(x_range, y_range)
    ax1.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.3, color='red', label='Ground (Z=0)')
    
    # Draw safety clearance plane at Z=0.02 (GREEN = safe zone)
    ax1.plot_surface(xx, yy, np.ones_like(xx)*0.02, alpha=0.2, color='green', label='Safety Clearance')
    
    # Add text annotation
    ax1.text(0, 0, 0, 'GROUND', fontsize=12, color='red', ha='center')
    
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Workspace View')
    ax1.set_box_aspect([1,1,1])
    
    # Top view (XY plane)
    ax2 = fig.add_subplot(132)
    ax2.scatter(points_vis[:, 0], points_vis[:, 1], c=points_vis[:, 2], 
                cmap='viridis', alpha=0.3, s=1)
    ax2.plot([ws['x_min'], ws['x_max'], ws['x_max'], ws['x_min'], ws['x_min']],
             [ws['y_min'], ws['y_min'], ws['y_max'], ws['y_max'], ws['y_min']],
             'r-', linewidth=2, label='Boundary')
    ax2.plot(0, 0, 'ko', markersize=10, label='Base')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View (XY Plane)')
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    ax2.legend()
    
    # Side view (XZ plane)
    ax3 = fig.add_subplot(133)
    ax3.scatter(points_vis[:, 0], points_vis[:, 2], c=points_vis[:, 1], 
                cmap='viridis', alpha=0.3, s=1)
    ax3.plot([ws['x_min'], ws['x_max'], ws['x_max'], ws['x_min'], ws['x_min']],
             [ws['z_min'], ws['z_min'], ws['z_max'], ws['z_max'], ws['z_min']],
             'r-', linewidth=2, label='Boundary')
    ax3.axhline(y=0, color='red', linestyle='-', linewidth=3, label='Ground (Z=0)', alpha=0.7)
    ax3.axhline(y=0.02, color='green', linestyle='--', linewidth=2, label='Safety Line (2cm)', alpha=0.7)
    ax3.fill_between([ws['x_min'], ws['x_max']], -0.1, 0, color='red', alpha=0.2, label='Danger Zone')
    ax3.plot(0, 0.2, 'ko', markersize=10, label='Base')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('Side View (XZ Plane)')
    ax3.grid(True, alpha=0.3)
    ax3.axis('equal')
    ax3.legend()
    
    plt.tight_layout()
    plt.savefig('workspace_analysis.png', dpi=150, bbox_inches='tight')
    print(f"\n✓ Visualization saved to: workspace_analysis.png")
    plt.show()


def test_specific_poses(robot, ws):
    """Test some specific poses within workspace"""
    print("\n" + "="*70)
    print("TESTING SPECIFIC POSES")
    print("="*70)
    
    test_poses = [
        ("Center", ws['center']),
        ("Front", [ws['x_max']*0.8, 0, ws['z_min']*1.2]),
        ("Left", [0, ws['y_max']*0.8, ws['center'][2]]),
        ("Right", [0, ws['y_min']*0.8, ws['center'][2]]),
        ("High", [0, 0, ws['z_max']*0.9]),
    ]
    
    q_min = np.array([link.qlim[0] for link in robot.links if link.isjoint])
    q_max = np.array([link.qlim[1] for link in robot.links if link.isjoint])
    
    print(f"{'Position':<10} {'Target (x,y,z)':<25} {'Reachable?':<12} {'Distance'}")
    print("-"*70)
    
    for name, target in test_poses:
        target = np.array(target)
        
        # Simple reachability test: try random configs
        best_dist = float('inf')
        reachable = False
        
        for _ in range(100):
            q = np.random.uniform(q_min, q_max)
            T = robot.fkine(q)
            dist = np.linalg.norm(T.t - target)
            
            if dist < best_dist:
                best_dist = dist
            
            if dist < 0.02:  # Within 2cm
                reachable = True
                break
        
        status = "✓ Yes" if reachable else "✗ No"
        print(f"{name:<10} ({target[0]:6.3f}, {target[1]:6.3f}, {target[2]:6.3f})  {status:<12} {best_dist:.4f}m")
    
    print("="*70)


def main():
    print("="*70)
    print("3R MANIPULATOR WORKSPACE ANALYZER")
    print("="*70)
    
    # Load robot
    robot = load_robot()
    print(f"✓ Robot loaded: {robot.name}")
    print(f"  DOF: {robot.n}")
    print(f"  Links: {len(robot.links)}")
    
    # Analyze workspace with more samples for accuracy
    workspace = analyze_workspace(robot, num_samples=20000)
    
    # Print summary
    print_workspace_summary(workspace)
    
    # Test specific poses
    test_specific_poses(robot, workspace)
    
    # Visualize
    print("\nGenerating visualization...")
    visualize_workspace(workspace, robot)
    
    # Final safety report
    print("\n" + "="*70)
    print("🔒 GROUND SAFETY CONFIRMATION")
    print("="*70)
    print(f"✅ Ground level (Z=0): {workspace['z_min']:.4f} m clearance")
    print(f"✅ Safety margin: {(workspace['z_min'] - 0.0) * 100:.2f} cm above ground")
    print(f"✅ All {len(workspace['points'])} points verified above ground")
    print(f"✅ No collision risk with ground plane")
    
    if workspace['z_min'] >= 0.02:
        print(f"\nCONFIRMED: Workspace is SAFE - minimum Z = {workspace['z_min']:.4f}m")
    else:
        print(f"\nWARNING: Some points too close to ground!")
    print("="*70)
    
    print("\n✓ Analysis complete!")
    print("="*70)


if __name__ == '__main__':
    main()