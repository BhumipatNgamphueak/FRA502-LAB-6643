#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped
import numpy as np 

class DummyNode(Node):
    def __init__(self):
        super().__init__('turtlesim_pose_node')
        self.create_subscription(Pose, "/turtle1/pose",self.turtle1_pose_callback,10)
        self.create_subscription(Pose, "/turtle2/pose",self.turtle2_pose_callback,10)
        self.Odom_publisher1 = self.create_publisher(Odometry, '/odom1',10)
        self.Odom_publisher2 = self.create_publisher(Odometry, '/odom2',10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.robot_pose = [0.0,0.0,0.0]
        
    def Odo_pub(self, msg, turtle_name,child_frame_id):
        self.robot_pose[0] = msg.x
        self.robot_pose[1] = msg.y
        self.robot_pose[2] = msg.theta
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = child_frame_id

        odom_msg.pose.pose.position.x = self.robot_pose[0] 
        odom_msg.pose.pose.position.y = self.robot_pose[1]

        q = quaternion_from_euler(0,0,self.robot_pose[2])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.x = q[2]
        odom_msg.pose.pose.orientation.x = q[3]

        if turtle_name == "turtle1" :
            self.Odom_publisher1.publish(odom_msg)
        elif turtle_name == "turtle2" :
            self.Odom_publisher2.publish(odom_msg)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "Odom"
        t.child_frame_id = child_frame_id

        t.transform.translation.x = self.robot_pose[0]
        t.transform.translation.y = self.robot_pose[1]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def turtle1_pose_callback (self,msg):
        self.Odo_pub(msg,"turtle1","turtle1")

    def turtle2_pose_callback (self,msg):
        self.Odo_pub(msg,"turtle2","turtle2")

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
