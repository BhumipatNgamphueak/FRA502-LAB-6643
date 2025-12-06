#!/usr/bin/env python3
"""
LAB4 Controller - PROPER TO MODE IMPLEMENTATION
Following exact RVIZ workflow as specified
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Float64MultiArray
from lab4_controller_rviz.srv import ComputeIK, SetControlMode, GetTargetPose
import numpy as np
import time
from robot_utils import load_robot, get_joint_limits, pose_msg_to_matrix

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')
        
        self.robot = load_robot()
        self.q_min, self.q_max = get_joint_limits(self.robot)
        
        # State
        self.q_current = np.array([0.0, 0.785, 0.785])
        self.q_dot_current = np.zeros(3)
        
        # Control
        self.control_mode = 0
        self.cmd_vel = Twist()  # Store latest velocity command
        self.velocity_frame = 'world'
        
        # AM mode (keep working version)
        self.am_state = 'idle'
        self.am_target = None
        self.am_target_q = None
        self.am_start_time = None
        self.am_reached_time = None
        
        # Parameters
        self.control_rate = 50.0  # 50 Hz control loop
        self.dt = 1.0 / self.control_rate  # Time step
        self.singularity_threshold = 0.01  # Singularity detection threshold
        
        # Subscribers
        self.create_subscription(JointState, '/joint_states', self.joint_state_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(PoseStamped, '/target', self.target_cb, 10)
        self.create_subscription(String, '/velocity_frame', self.frame_cb, 10)
        
        # Publishers
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/joint_commands', 10)
        self.warning_pub = self.create_publisher(String, '/singularity_warning', 10)
        
        # Services
        self.create_service(ComputeIK, 'compute_ik', self.ik_service)
        self.create_service(SetControlMode, 'set_control_mode', self.mode_service)
        self.target_client = self.create_client(GetTargetPose, 'get_target_pose')
        
        # Control timer - runs at control_rate Hz
        self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('='*70)
        self.get_logger().info('CONTROLLER - PROPER TO MODE IMPLEMENTATION')
        self.get_logger().info(f'Control Rate: {self.control_rate} Hz, dt: {self.dt:.3f}s')
        self.get_logger().info('='*70)
    
    def joint_state_cb(self, msg):
        """Update current joint state from robot"""
        if len(msg.position) >= 3:
            self.q_current = np.array(msg.position[:3])
            if len(msg.velocity) >= 3:
                self.q_dot_current = np.array(msg.velocity[:3])
    
    def cmd_vel_cb(self, msg):
        """Step 1: Receive Velocity (áº‹) from keyboard"""
        self.cmd_vel = msg
        
        # Debug: Log when we receive velocity
        v_linear = np.linalg.norm([msg.linear.x, msg.linear.y, msg.linear.z])
        v_angular = np.linalg.norm([msg.angular.x, msg.angular.y, msg.angular.z])
        if v_linear > 0 or v_angular > 0:
            self.get_logger().debug(f'TO: Received velocity - linear={v_linear:.3f}, angular={v_angular:.3f}')
    
    def target_cb(self, msg):
        """AM mode target reception"""
        if self.control_mode == 2 and self.am_state == 'idle':
            self.am_target = msg
            p = msg.pose.position
            self.get_logger().info(f'AM: Target received ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})')
    
    def frame_cb(self, msg):
        """Update velocity frame selection"""
        if msg.data in ['world', 'end_effector']:
            old_frame = self.velocity_frame
            self.velocity_frame = msg.data
            if old_frame != msg.data:
                self.get_logger().info(f'TO: Frame changed to {self.velocity_frame.upper()}')
    
    def check_singularity(self, q):
        """
        Step 2: Check for Singularity
        Returns True if near singularity
        """
        try:
            # Get Jacobian at current configuration
            J = self.robot.jacob0(q)
            
            # Method 1: Singular Value Decomposition
            _, S, _ = np.linalg.svd(J)
            min_singular_value = np.min(S)
            
            # Method 2: Manipulability (optional, using roboticstoolbox)
            manipulability = np.sqrt(np.linalg.det(J @ J.T))
            
            # Check if near singularity
            is_singular = min_singular_value < self.singularity_threshold
            
            if is_singular:
                # Publish warning
                warning_msg = String()
                warning_msg.data = f"SINGULARITY WARNING: Ïƒ_min={min_singular_value:.6f}, manipulability={manipulability:.6f}"
                self.warning_pub.publish(warning_msg)
                
                # Print to terminal
                self.get_logger().warn(f'âš ï¸ TO: Near singularity! Ïƒ_min={min_singular_value:.6f}')
                
            return is_singular
            
        except Exception as e:
            self.get_logger().error(f'Singularity check error: {e}')
            return True  # Assume singular if check fails
    
    def teleoperation_mode(self):
        """
        Complete TO Mode Implementation - FIXED VERSION
        """
        # Extract velocity command
        v_linear = np.array([
            self.cmd_vel.linear.x,
            self.cmd_vel.linear.y,
            self.cmd_vel.linear.z
        ])
        v_angular = np.array([
            self.cmd_vel.angular.x,
            self.cmd_vel.angular.y,
            self.cmd_vel.angular.z
        ])
        
        # Check if there's any velocity command
        if np.linalg.norm(v_linear) < 1e-6 and np.linalg.norm(v_angular) < 1e-6:
            return  # No motion commanded
        
        # Check for Singularity
        if self.check_singularity(self.q_current):
            self.get_logger().warn('TO: Motion stopped due to singularity')
            return
        
        # Calculate Jacobian
        try:
            if self.velocity_frame == 'world':
                J = self.robot.jacob0(self.q_current)
            else:  # end_effector
                J = self.robot.jacobe(self.q_current)
            
            # Combine velocities
            x_dot = np.concatenate([v_linear, v_angular])
            
            # FIX 1: Reduced damping (0.001 instead of 0.01)
            damping = 0.001
            J_damped = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(6))
            
            # Calculate joint velocities
            q_dot = J_damped @ x_dot
            
            # FIX 2: Apply velocity scaling for better responsiveness
            velocity_scale = 3.0
            q_dot *= velocity_scale
            
            # FIX 3: Increased velocity limit
            max_joint_velocity = 4.0  # rad/s (was 2.0)
            q_dot = np.clip(q_dot, -max_joint_velocity, max_joint_velocity)
            
            # Calculate new position
            q_next = self.q_current + (q_dot * self.dt)
            
            # Apply joint limits
            q_next = np.clip(q_next, self.q_min, self.q_max)
            
            # Send command (removed unnecessary movement check)
            self._send_joint_command(q_next)
            
        except Exception as e:
            self.get_logger().error(f'TO mode error: {e}')


    def position_only_ik(self, target_pos, q0=None):
        """IK for position only (ignores orientation) - FIXED VERSION"""
        if q0 is None:
            q0 = self.q_current
        
        best_q = None
        best_error = float('inf')
        
        # Try multiple random starts
        for _ in range(50):
            # Random initial guess
            q_init = np.random.uniform(self.q_min, self.q_max)
            
            # Try from random start
            for _ in range(100):
                T = self.robot.fkine(q_init)
                error = np.linalg.norm(T.t - target_pos)
                
                if error < best_error:
                    best_error = error
                    best_q = q_init.copy()
                
                if error < 0.001:  # Good enough
                    return True, best_q
                
                # Numerical gradient descent
                eps = 0.001
                J = np.zeros((3, 3))
                for i in range(3):
                    q_plus = q_init.copy()
                    q_plus[i] += eps
                    T_plus = self.robot.fkine(q_plus)
                    J[:, i] = (T_plus.t - T.t) / eps
                
                # Update
                try:
                    dq = np.linalg.pinv(J) @ (target_pos - T.t)
                    q_init = q_init + 0.1 * dq
                    q_init = np.clip(q_init, self.q_min, self.q_max)
                except:
                    break
        
        # Return best found
        if best_error < 0.05:  # Within 5cm
            return True, best_q
        
        return False, None
    
    def ik_service(self, req, res):
        """IPK service - Position-only IK (ignores orientation)"""
        # Extract target position only
        target_pos = np.array([
            req.target_pose.position.x,
            req.target_pose.position.y,
            req.target_pose.position.z
        ])
        
        # Find IK solution for position only
        success, q = self.position_only_ik(target_pos, self.q_current)
        
        if success:
            self._send_joint_command(q)
            res.success = True
            res.joint_positions = q.tolist()
            T = self.robot.fkine(q)
            error = np.linalg.norm(T.t - target_pos)
            res.message = f"Position reached (error: {error:.4f}m)"
            self.get_logger().info(f'IPK: Success - error={error:.4f}m, q={q}')
        else:
            res.success = False
            res.message = "No solution found"
            self.get_logger().warn('IPK: Failed to find solution')
        
        return res
    
    def mode_service(self, req, res):
        """Mode switching"""
        self.control_mode = req.mode
        names = {0: 'IPK', 1: 'TO', 2: 'AM'}
        
        self.get_logger().info(f'='*70)
        self.get_logger().info(f'Mode changed to: {names[req.mode]}')
        
        if req.mode == 1:  # TO
            self.get_logger().info('Teleoperation Mode Active')
            self.get_logger().info(f'Current frame: {self.velocity_frame.upper()}')
            self.get_logger().info('Use keyboard to control, press F to toggle frame')
            
        elif req.mode == 2:  # AM
            self.am_state = 'idle'
            self.am_target = None
            self._request_target()
        
        self.get_logger().info(f'='*70)
        
        res.success = True
        res.message = names[req.mode]
        return res
    
    def control_loop(self):
        """Main control loop running at control_rate Hz"""
        if self.control_mode == 1:
            # TO mode - process velocity commands
            self.teleoperation_mode()
            
        elif self.control_mode == 2:
            # AM mode - autonomous movement
            self._am_mode()
    
    def _am_mode(self):
        """AM mode state machine (keep working version)"""
        if self.am_state == 'idle':
            if self.am_target is not None:
                target_pos = np.array([
                    self.am_target.pose.position.x,
                    self.am_target.pose.position.y,
                    self.am_target.pose.position.z
                ])
                
                success, q = self.position_only_ik(target_pos)
                
                if success:
                    self.am_target_q = q
                    self._send_joint_command(q)
                    self.am_state = 'moving'
                    self.am_start_time = time.time()
                    self.get_logger().info('AM: Moving to target')
                else:
                    self.get_logger().warn('AM: IK failed')
                    self.am_target = None
                    self._request_target()
        
        elif self.am_state == 'moving':
            if self.am_target is None:
                self.am_state = 'idle'
                return
            
            T = self.robot.fkine(self.q_current)
            target_pos = np.array([
                self.am_target.pose.position.x,
                self.am_target.pose.position.y,
                self.am_target.pose.position.z
            ])
            
            pos_error = np.linalg.norm(T.t - target_pos)
            elapsed = time.time() - self.am_start_time
            
            if pos_error < 0.02:
                self.get_logger().info(f'AM: Reached! error={pos_error:.3f}m')
                self.am_state = 'reached'
                self.am_reached_time = time.time()
            elif elapsed > 5.0:
                self.get_logger().warn('AM: Timeout')
                self.am_state = 'reached'
                self.am_reached_time = time.time()
            elif int(elapsed) != int(elapsed - self.dt):
                self.get_logger().info(f'AM: Moving... error={pos_error:.3f}m, t={elapsed:.1f}s')
        
        elif self.am_state == 'reached':
            if time.time() - self.am_reached_time > 1.0:
                self.am_state = 'idle'
                self.am_target = None
                self._request_target()
    
    def _request_target(self):
        """Request target for AM mode"""
        if self.target_client.wait_for_service(timeout_sec=0.5):
            future = self.target_client.call_async(GetTargetPose.Request())
            self.get_logger().info('AM: Requesting new target...')
    
    def _send_joint_command(self, q):
        """Send joint command to robot"""
        msg = Float64MultiArray()
        msg.data = q.tolist()
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(ControllerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()