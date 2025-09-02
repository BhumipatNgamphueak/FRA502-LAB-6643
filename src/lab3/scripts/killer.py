#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill
import numpy as np 
import math
from std_msgs.msg import  Bool
from controller_interfaces.srv import SetParam

class DummyNode(Node):
    def __init__(self):
        super().__init__('killer_node')
        #self.name_space = self.get_namespace()
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel',10)
        #self.cmd_vel_pub = self.create_publisher(Twist, f'{self.name_space}/cmd_vel',10)
        self.create_subscription(Pose, "/eater/pose",self.turtle1_pose_callback,10)
        self.create_subscription(Bool,'/eater/eat_status',self.AlreadyEat_callback,10)
        self.Kill_client = self.create_client(Kill, "/remove_turtle")
        #self.create_subscription(Pose, "/pose",self.turtle2_pose_callback,10)
        self.create_subscription(Pose, "pose",self.turtle2_pose_callback,10)
        self.declare_parameter('sampling_frequency', 100.0)
        self.sampling_frequency = float(self.get_parameter('sampling_frequency').get_parameter_value().double_value)
        self.create_timer(1/self.sampling_frequency, self.timer_callback)

        request = Spawn.Request()
        request.name = 'turtle2'
        request.x = 2.0
        request.y = 2.0
        request.theta = 0.2
        self.spawn_turtle_client = self.create_client(Spawn, "spawn_turtle")
        self.spawn_turtle_client.call_async(request)
        self.turtle1_pose = np.array([0.0,0.0,0.0])# x, y, theta
        self.turtle2_pose = np.array([0.0,0.0,0.0]) # x, y, theta
        self.Can_Eat = False
        self.setParam_server = self.create_service(SetParam, 'set_Param',self.set_Param_callback)

        self.Kp_Angular = 20.0
        self.Kp_linear = 2.0


    def turtle1_pose_callback (self,msg):
        self.turtle1_pose[0] = msg.x
        self.turtle1_pose[1] = msg.y
        self.turtle1_pose[2] = msg.theta
        #return self.turtle1_pose

    def turtle2_pose_callback (self,msg):
        self.turtle2_pose[0] = msg.x
        self.turtle2_pose[1] = msg.y
        self.turtle2_pose[2] = msg.theta
        #self.get_logger().info(f'{msg}')
        self.Controller(self.turtle1_pose, self.turtle2_pose)
        #return self.turtle2_pose

    def Controller (self,turtle1_pose, turtle2_pose):
        self.delta_x = turtle1_pose[0]-turtle2_pose[0]
        self.delta_y = turtle1_pose[1]-turtle2_pose[1]
        #self.get_logger().info(f'{turtle2_pose[0]}')
        d = math.sqrt(pow(self.delta_x,2)+pow(self.delta_y,2)) 
        self.alpha = math.atan2(self.delta_y,self.delta_x )
        e_1 = self.alpha-self.turtle2_pose[2]
        e_2 = math.atan2(math.sin(e_1),math.cos(e_1))
        if (self.Can_Eat == True):
            self.cmdvel(d*self.Kp_linear , self.Kp_Angular*e_2)
        else:
            self.cmdvel(0.0,0.0)
        if ( d < 1):
            if self.Can_Eat == True:
                self.Kill_Turtle()
        
        #e_1 = turtle1_pose[2]-turtle2_pose[2]
        #e_2 = math.atan2(self.delta_y,self.delta_x)
        #e_2 = math.atan2(math.sin(e_1),math.cos(e_1))

    def timer_callback(self):
        pass
        #self.Controller(self.turtle1_pose, self.turtle2_pose)
        #self.cmdvel(0.1,0.5)

    def cmdvel(self,v,w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)
    
    def Kill_Turtle(self):
        Kill_request = Kill.Request()
        Kill_request.name = "eater"
        self.Kill_client.call_async(Kill_request)

    def AlreadyEat_callback(self,msg):
        self.Can_Eat = msg.data 

    def set_Param_callback(self, request:SetParam.Request, response:SetParam.Response):

        self.Kp_Angular = request.kp_angular.data
        self.Kp_linear = request._kp_linear.data
        
        return response
def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
