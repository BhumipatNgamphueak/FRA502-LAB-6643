#!/usr/bin/env python3
"""
Random Pose Node - GROUND SAFE VERSION
Only generates targets above ground (Z > 0.1m)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from lab4_controller_rviz.srv import GetTargetPose
import numpy as np
from robot_utils import load_robot, get_joint_limits, matrix_to_pose_msg

class RandomPoseNode(Node):
    def __init__(self):
        super().__init__('random_pose')
        
        self.robot = load_robot()
        self.q_min, self.q_max = get_joint_limits(self.robot)
        
        # Generate safe workspace
        self.get_logger().info('Generating safe workspace...')
        self.workspace = self._compute_safe_workspace()
        self.pose_cache = self._generate_safe_poses(100)
        
        self.current_target = None
        
        # Publishers & Service
        self.target_pub = self.create_publisher(PoseStamped, '/target', 10)
        self.marker_pub = self.create_publisher(Marker, '/target_marker', 10)
        self.create_service(GetTargetPose, 'get_target_pose', self.get_target_cb)
        
        # Timer
        self.create_timer(0.5, self.publish_target)
        
        self.get_logger().info('='*70)
        self.get_logger().info('RANDOM POSE NODE (GROUND SAFE)')
        self.get_logger().info(f'X: [{self.workspace["x"][0]:.2f}, {self.workspace["x"][1]:.2f}]')
        self.get_logger().info(f'Y: [{self.workspace["y"][0]:.2f}, {self.workspace["y"][1]:.2f}]')
        self.get_logger().info(f'Z: [{self.workspace["z"][0]:.2f}, {self.workspace["z"][1]:.2f}] (>0.1m)')
        self.get_logger().info('='*70)
    
    def _compute_safe_workspace(self):
        """Compute workspace with ground safety"""
        points = []
        
        for _ in range(500):
            q = np.random.uniform(self.q_min, self.q_max)
            T = self.robot.fkine(q)
            
            # Only include points above ground
            if T.t[2] > 0.1:  # 10cm above ground
                points.append(T.t)
        
        if len(points) == 0:
            # Fallback workspace
            return {'x': [-0.3, 0.3], 'y': [-0.3, 0.3], 'z': [0.1, 0.5]}
        
        points = np.array(points)
        workspace = {}
        
        for i, axis in enumerate(['x', 'y', 'z']):
            workspace[axis] = [
                float(np.percentile(points[:, i], 10)),  # 10th percentile
                float(np.percentile(points[:, i], 90))   # 90th percentile
            ]
        
        # Ensure Z is always above ground
        workspace['z'][0] = max(workspace['z'][0], 0.1)
        
        return workspace
    
    def _generate_safe_poses(self, n):
        """Generate reachable poses above ground"""
        cache = []
        
        for _ in range(n * 10):
            q = np.random.uniform(self.q_min, self.q_max)
            T = self.robot.fkine(q)
            
            # Check if safe
            if (T.t[2] > 0.1 and  # Above ground
                self.workspace['x'][0] <= T.t[0] <= self.workspace['x'][1] and
                self.workspace['y'][0] <= T.t[1] <= self.workspace['y'][1]):
                
                msg = PoseStamped()
                msg.header.frame_id = 'world'
                msg.pose = matrix_to_pose_msg(T)
                cache.append(msg)
                
                if len(cache) >= n:
                    break
        
        # If not enough, add safe defaults
        if len(cache) < 10:
            for i in range(10):
                msg = PoseStamped()
                msg.header.frame_id = 'world'
                msg.pose.position.x = 0.2 * np.cos(i * np.pi/5)
                msg.pose.position.y = 0.2 * np.sin(i * np.pi/5)
                msg.pose.position.z = 0.2 + 0.1 * (i/10)  # 20-30cm high
                msg.pose.orientation.w = 1.0
                cache.append(msg)
        
        return cache
    
    def get_target_cb(self, request, response):
        """Generate new SAFE target"""
        if self.pose_cache:
            idx = np.random.randint(len(self.pose_cache))
            self.current_target = self.pose_cache[idx]
            self.current_target.header.stamp = self.get_clock().now().to_msg()
        else:
            # Fallback safe target
            self.current_target = PoseStamped()
            self.current_target.header.frame_id = 'world'
            self.current_target.pose.position.x = 0.15
            self.current_target.pose.position.y = 0.0
            self.current_target.pose.position.z = 0.25  # Safe height
            self.current_target.pose.orientation.w = 1.0
        
        response.target_pose = self.current_target
        
        p = self.current_target.pose.position
        self.get_logger().info(f'🎯 Safe target: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        
        return response
    
    def publish_target(self):
        """Publish current target"""
        if not self.current_target:
            return
        
        self.current_target.header.stamp = self.get_clock().now().to_msg()
        self.target_pub.publish(self.current_target)
        
        # Marker
        m = Marker()
        m.header = self.current_target.header
        m.type = Marker.SPHERE
        m.pose = self.current_target.pose
        m.scale.x = m.scale.y = m.scale.z = 0.05
        m.color.r = 1.0
        m.color.a = 0.8
        self.marker_pub.publish(m)

def main():
    rclpy.init()
    rclpy.spin(RandomPoseNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()