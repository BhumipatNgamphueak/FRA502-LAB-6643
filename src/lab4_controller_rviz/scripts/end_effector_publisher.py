#!/usr/bin/env python3
"""
End-Effector Publisher Node for LAB4
Publishes the current end-effector pose based on joint states
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
from robot_utils import load_robot, forward_kinematics, matrix_to_pose_msg

class EndEffectorPublisher(Node):
    def __init__(self):
        super().__init__('end_effector_publisher')
        
        # Load robot model
        self.robot = load_robot()
        self.q_current = np.zeros(3)
        
        # Subscriber for joint states
        self.joint_sub = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.joint_state_callback, 
            10
        )
        
        # Publisher for end-effector pose
        self.ee_pub = self.create_publisher(
            PoseStamped, 
            '/end_effector', 
            10
        )
        
        # Timer to publish at 50 Hz
        self.create_timer(0.02, self.publish_end_effector)
        
        self.get_logger().info('End-effector publisher started')
    
    def joint_state_callback(self, msg):
        """Update current joint configuration"""
        if len(msg.position) >= 3:
            self.q_current = np.array(msg.position[:3])
    
    def publish_end_effector(self):
        """Compute and publish end-effector pose"""
        # Forward kinematics
        T = forward_kinematics(self.robot, self.q_current)
        
        # Create PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose = matrix_to_pose_msg(T)
        
        # Publish
        self.ee_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()