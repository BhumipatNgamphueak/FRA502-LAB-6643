#!/usr/bin/env python3
"""
Mock Robot - HOME CONFIGURATION VERSION
Starts at URDF home position [0, 0, 0]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class MockRobotNode(Node):
    def __init__(self):
        super().__init__('mock_robot')
        
        # HOME CONFIGURATION (URDF default)
        self.HOME = np.array([0.0, 0.0, 0.0])
        
        # Start at home
        self.q = self.HOME.copy()
        self.q_dot = np.zeros(3)
        self.q_cmd = self.HOME.copy()
        
        # Limits
        self.q_min = np.array([-np.pi, -np.pi, -np.pi])
        self.q_max = np.array([np.pi, np.pi, np.pi])
        
        # Control gains
        self.kp = 15.0
        self.kd = 3.0
        self.max_vel = 3.0
        self.max_acc = 10.0
        
        # Subscribers/Publishers
        self.create_subscription(Float64MultiArray, '/joint_commands', self.cmd_cb, 10)
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Control timer
        self.dt = 0.02
        self.create_timer(self.dt, self.update)
        
        self.get_logger().info('='*70)
        self.get_logger().info('MOCK ROBOT READY')
        self.get_logger().info(f'HOME: [{self.HOME[0]:.1f}, {self.HOME[1]:.1f}, {self.HOME[2]:.1f}] rad')
        self.get_logger().info('='*70)
    
    def cmd_cb(self, msg):
        if len(msg.data) >= 3:
            self.q_cmd = np.clip(msg.data[:3], self.q_min, self.q_max)
    
    def update(self):
        # PD control
        error = self.q_cmd - self.q
        
        if np.linalg.norm(error) > 0.001:
            q_dot_desired = self.kp * error - self.kd * self.q_dot
            q_acc = np.clip((q_dot_desired - self.q_dot)/self.dt, -self.max_acc, self.max_acc)
            self.q_dot = np.clip(self.q_dot + q_acc*self.dt, -self.max_vel, self.max_vel)
            self.q += self.q_dot * self.dt
            self.q = np.clip(self.q, self.q_min, self.q_max)
        else:
            self.q_dot *= 0.9
        
        # Publish
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3']
        msg.position = self.q.tolist()
        msg.velocity = self.q_dot.tolist()
        self.state_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(MockRobotNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()