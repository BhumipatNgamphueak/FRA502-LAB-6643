#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
import numpy as np 
import math
from std_msgs.msg import Int16, Bool

class DummyNode(Node):
    def __init__(self):
        super().__init__('eater_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        self.AlreadyEat_pub = self.create_publisher(Bool, '/AlreadyEat',10)
        self.spawn_pizza_client = self.create_client(GivePosition, "spawn_pizza")
        self.eat_pizza_client = self.create_client(Empty, "turtle1/eat")
        self.create_subscription(Pose, "/turtle1/pose",self.turtle1_pose_callback,10)
        self.create_subscription(Point, "/mouse_position",self.mouse_position_callback,10)
        self.create_subscription(Int16, "/set_max_pizza",self.max_pizza_callback,10)
        self.create_subscription(PoseStamped, "/goal_pose",self.goal_pose_callback,10)
        self.create_timer(0.01, self.timer_callback)
        self.mouse_pose = np.array([0.0,0.0])
        self.num_pizza = 0
        self.turtle1_pose = np.array([0.0,0.0,0.0])# x, y, theta
        self.Pizza_targat_X = []
        self.Pizza_targat_Y = []
        self.index = 0
        self.All = 0
        self.Max_pizza = 5

    def mouse_position_callback(self,msg):
        self.mouse_pose[0] = msg.x
        self.mouse_pose[1] = msg.y
        if self.num_pizza <= self.Max_pizza - 1  :
            self.spawn_pizza(self.mouse_pose[0],self.mouse_pose[1])
        self.Pizza_targat_X.append (self.mouse_pose[0])
        self.Pizza_targat_Y.append (self.mouse_pose[1])
        self.index += 1


        #self.cmdvel(0,0)

    def spawn_pizza(self, x, y):
        position_request = GivePosition.Request()
        position_request.x = x
        position_request.y = y
        self.spawn_pizza_client.call_async(position_request)
        self.num_pizza += 1
        
    def Controller (self):
        if self.num_pizza != 0:
            #self.delta_x = self.mouse_pose[0]-self.turtle1_pose[0]
            #self.delta_y = self.mouse_pose[1]-self.turtle1_pose[1]
            self.delta_x = self.Pizza_targat_X[self.All]-self.turtle1_pose[0]
            self.delta_y = self.Pizza_targat_Y[self.All]-self.turtle1_pose[1]
            #self.get_logger().info(f'{self.All}')

            d = math.sqrt(self.delta_x**2 +self.delta_y**2)
            #self.get_logger().info(f'{d}')

            self.alpha = math.atan2(self.delta_y,self.delta_x )

            e_1 = self.alpha-self.turtle1_pose[2]

            e_2 = math.atan2(math.sin(e_1),math.cos(e_1))
            if len(self.Pizza_targat_X) != 0: 
                self.cmdvel(d*2,e_2*20)
            if (d <= 0.01):
                self.eat_pizza()
                if(self.All >= self.Max_pizza):
                    self.All = self.num_pizza - 1
                else:
                    self.All += 1

    def max_pizza_callback(self, msg):
        self.Max_pizza = msg.data

    def turtle1_pose_callback (self,msg):
        self.turtle1_pose[0] = msg.x
        self.turtle1_pose[1] = msg.y
        self.turtle1_pose[2] = msg.theta
        if (self.num_pizza == 0):
            self.cmdvel(0.0,0.0)

    def AlreadyEat(self, a):
        msg = Bool()
        msg.data = a
        self.AlreadyEat_pub.publish(msg)
        
    def cmdvel(self,v,w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)
    
    def timer_callback(self):
        self.Controller()
        if (self.Max_pizza <= self.All):
            self.AlreadyEat(True)
        else:
            self.AlreadyEat(False)
        #self.eat_pizza()

    def eat_pizza(self):
        eat_pizza_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_pizza_request)

    def goal_pose_callback(self,msg):
        #A = PoseStamped()
        #msg._pose.position.x
        #msg._pose.position.y
        if self.num_pizza <= self.Max_pizza - 1  :
           self.spawn_pizza(msg._pose.position.x+5.44,msg._pose.position.y+5.44)
        self.Pizza_targat_X.append (msg._pose.position.x +5.44)
        self.Pizza_targat_Y.append (msg._pose.position.y+5.44)
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
