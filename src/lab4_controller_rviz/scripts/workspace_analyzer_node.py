#!/usr/bin/env python3
"""
Part 1.1: Workspace Analysis Node
✅ WITH FALLBACK MECHANISM
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from robot_utils import load_robot, compute_workspace


class WorkspaceAnalyzerNode(Node):
    def __init__(self):
        super().__init__('workspace_analyzer')
        
        # Parameters
        self.declare_parameter('num_samples', 10000)
        self.declare_parameter('ground_clearance', 0.02)  # ✅ ลดเหลือ 2 cm
        
        num_samples = self.get_parameter('num_samples').value
        ground_clearance = self.get_parameter('ground_clearance').value
        
        self.get_logger().info('Loading robot...')
        self.robot = load_robot()
        
        self.get_logger().info(
            f'Computing workspace ({num_samples} samples) '
            f'with {ground_clearance*100:.0f} cm ground clearance...'
        )
        
        # ✅ คำนวณ workspace โดยกรอง collision ออก
        try:
            self.workspace_data = compute_workspace(
                self.robot, 
                num_samples,
                ground_clearance
            )
            
            # ✅ เช็คว่าได้ samples เพียงพอหรือไม่
            if self.workspace_data['valid_samples'] < 100:
                self.get_logger().warn(
                    f"⚠️  Only {self.workspace_data['valid_samples']} valid samples found. "
                    f"Using fallback workspace."
                )
                self.workspace_data = self._generate_fallback_workspace()
            
        except ValueError as e:
            # ✅ ถ้าหาไม่เจอเลย ใช้ fallback
            self.get_logger().warn(f'⚠️  {str(e)} - Using fallback workspace')
            self.workspace_data = self._generate_fallback_workspace()
        
        self._log_workspace_info()
        
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/workspace_points', 10)
        self.timer = self.create_timer(1.0, self.publish_pointcloud)
        
        self.get_logger().info('✅ Workspace analyzer ready')
    
    def _generate_fallback_workspace(self):
        """
        ✅ สร้าง fallback workspace โดยใช้ FK กับ random configurations
        (ไม่เช็ค collision เข้มงวด)
        """
        from robot_utils import get_joint_limits, forward_kinematics
        
        self.get_logger().info('Generating fallback workspace (relaxed constraints)...')
        
        q_min, q_max = get_joint_limits(self.robot)
        points = []
        
        # สุ่ม 1000 configurations โดยไม่เช็ค collision
        for _ in range(1000):
            q = np.random.uniform(q_min, q_max)
            T = forward_kinematics(self.robot, q)
            
            # เช็คเฉพาะว่า Z > 0.03 (เช็คหลวมมาก)
            if T.t[2] > 0.03:
                points.append(T.t)
        
        if len(points) == 0:
            # ถ้ายังหาไม่ได้ ใช้ค่า default
            self.get_logger().warn('⚠️  Cannot generate any points, using defaults')
            points = np.array([
                [0.0, 0.0, 0.3],
                [0.3, 0.0, 0.4],
                [-0.3, 0.0, 0.4],
                [0.0, 0.3, 0.4],
                [0.0, -0.3, 0.4],
            ])
        else:
            points = np.array(points)
        
        self.get_logger().info(f'✅ Generated {len(points)} fallback points')
        
        return {
            'x_min': float(np.min(points[:, 0])),
            'x_max': float(np.max(points[:, 0])),
            'y_min': float(np.min(points[:, 1])),
            'y_max': float(np.max(points[:, 1])),
            'z_min': float(np.min(points[:, 2])),
            'z_max': float(np.max(points[:, 2])),
            'points': points,
            'valid_samples': len(points),
            'total_attempts': len(points)
        }
    
    def _log_workspace_info(self):
        data = self.workspace_data
        self.get_logger().info('='*70)
        self.get_logger().info(f"X: [{data['x_min']:.4f}, {data['x_max']:.4f}] m")
        self.get_logger().info(f"Y: [{data['y_min']:.4f}, {data['y_max']:.4f}] m")
        self.get_logger().info(f"Z: [{data['z_min']:.4f}, {data['z_max']:.4f}] m")
        
        volume = ((data['x_max']-data['x_min']) * 
                  (data['y_max']-data['y_min']) * 
                  (data['z_max']-data['z_min']))
        self.get_logger().info(f"Volume: {volume:.6f} m³")
        
        # แสดงข้อมูลการกรอง collision
        if data['total_attempts'] > 0:
            self.get_logger().info(f"Valid samples: {data['valid_samples']}/{data['total_attempts']}")
            success_rate = (data['valid_samples'] / data['total_attempts']) * 100
            self.get_logger().info(f"Success rate: {success_rate:.1f}%")
        
        self.get_logger().info('='*70)
    
    def publish_pointcloud(self):
        points = self.workspace_data['points']
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        self.pointcloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WorkspaceAnalyzerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()