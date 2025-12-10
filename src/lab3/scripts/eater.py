#!/usr/bin/python3

from lab2.dummy_module import dummy_function, dummy_var
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from controller_interfaces.srv import SetMaxpizza
from controller_interfaces.srv import SetParam
import numpy as np 
import math
from std_msgs.msg import Int64, Bool

class DummyNode(Node):

    def __init__(self):
        
        super().__init__('eater_node')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel',10)
        self.AlreadyEat_pub = self.create_publisher(Bool, 'eat_status',10)
               
        self.create_subscription(Pose, "pose",self.turtle1_pose_callback,10)
        self.create_subscription(Point, "/mouse_position",self.mouse_position_callback,10)
        self.create_subscription(Int64,"pizza_count",self.pizza_count_callback,10)

        self.spawn_pizza_client = self.create_client(GivePosition, "/spawn_pizza")
        self.eat_pizza_client = self.create_client(Empty, "eat")

        self.setMaxpizza_server = self.create_service(SetMaxpizza, 'set_max_pizza',self.set_max_pizza_callback)
        self.setParam_server = self.create_service(SetParam, 'set_Param',self.set_Param_callback)

        self.declare_parameter('sampling_frequency', 100.0)
        self.sampling_frequency = float(self.get_parameter('sampling_frequency').get_parameter_value().double_value)

        self.timer = self.create_timer(1/self.sampling_frequency, self.timer_callback)

        self.mouse_pose = np.array([0.0,0.0])
        self.num_pizza = 0
        self.eaten_pizza = 0
        self.turtle1_pose = np.array([0.0,0.0,0.0])# x, y, theta
        self.Pizza_targat_X = []
        self.Pizza_targat_Y = []
        self.index = 0
        self.All = 0
        self.Max_pizza = 5
        self.Kp_Angular = 10.0
        self.Kp_linear = 2.0
        self.count = 0
        self.log = [self.All,self.Max_pizza]
        self.temp_target_len_before = 0
    
    def Controller (self):
        if self.num_pizza != 0:
            try:
                self.delta_x = self.Pizza_targat_X[self.All]-self.turtle1_pose[0]
                self.delta_y = self.Pizza_targat_Y[self.All]-self.turtle1_pose[1]
            except:
                pass

            d = math.sqrt(self.delta_x**2 +self.delta_y**2)

            self.alpha = math.atan2(self.delta_y,self.delta_x )

            e_1 = self.alpha-self.turtle1_pose[2]
            e_2 = math.atan2(math.sin(e_1),math.cos(e_1))

            if len(self.Pizza_targat_X) != 0: 
                self.cmdvel(d*self.Kp_linear,e_2*self.Kp_Angular)
                
            if (d <= 0.005):
                self.eat_pizza()
                #self.All += 1
                if (self.All == len(self.Pizza_targat_X) ):
                    pass
                else :
                    self.All += 1

    def spawn_pizza(self, x, y):
        position_request = GivePosition.Request()
        position_request.x = x
        position_request.y = y
        self.spawn_pizza_client.call_async(position_request)
        self.num_pizza += 1

    def AlreadyEat(self, a):
        msg = Bool()
        msg.data = a
        self.AlreadyEat_pub.publish(msg)
        
    def cmdvel(self,v,w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_vel_pub.publish(msg)

    def eat_pizza(self):
        eat_pizza_request = Empty.Request()
        self.eat_pizza_client.call_async(eat_pizza_request)

    def mouse_position_callback(self,msg):
        self.mouse_pose[0] = msg.x
        self.mouse_pose[1] = msg.y
        if self.num_pizza <= self.Max_pizza - 1  :
            self.spawn_pizza(self.mouse_pose[0],self.mouse_pose[1])
        self.Pizza_targat_X.append (self.mouse_pose[0])
        self.Pizza_targat_Y.append (self.mouse_pose[1])
        self.index += 1

    def max_pizza_callback(self, msg):
        self.Max_pizza = msg.data

    def turtle1_pose_callback (self,msg):
        self.turtle1_pose[0] = msg.x
        self.turtle1_pose[1] = msg.y
        self.turtle1_pose[2] = msg.theta
        if (self.num_pizza == 0):
            self.cmdvel(0.0,0.0)

    def timer_callback(self):
        self.Controller()
        #self.get_logger().info(f'{self.num_pizza}')
        if (self.Max_pizza <= self.All):
            self.AlreadyEat(True)
        else:
            self.AlreadyEat(False)

    def goal_pose_callback(self,msg):
        if self.num_pizza <= self.Max_pizza - 1  :
           self.spawn_pizza(msg._pose.position.x+5.44,msg._pose.position.y+5.44)
        self.Pizza_targat_X.append (msg._pose.position.x +5.44)
        self.Pizza_targat_Y.append (msg._pose.position.y + 5.44)
        self.index += 1

    def set_max_pizza_callback(self, request:SetMaxpizza.Request, response:SetMaxpizza.Response):
        
        self.dummy_pizza = request.max_pizza.data
        if(self.Max_pizza < self.dummy_pizza):
            response.log.data = "success"
            self.Max_pizza = request.max_pizza.data
        else:
            response.log.data = "failed"
        return response
    
    def set_Param_callback(self, request:SetParam.Request, response:SetParam.Response):

        self.Kp_Angular = request.kp_angular.data
        self.Kp_linear = request.kp_linear.data

        return response
    
    def pizza_count_callback(self, msg):
        #pass
        self.eaten_pizza = msg.data
        #self.get_logger().info(f'{self.num_pizza}')

def main(args=None):
    rclpy.init(args=args)
    node = DummyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
