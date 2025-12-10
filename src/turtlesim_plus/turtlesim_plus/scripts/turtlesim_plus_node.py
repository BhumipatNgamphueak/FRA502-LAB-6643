#!/usr/bin/env python3
# quick skeleton; TODO fill logic

import rclpy
from rclpy.node import Node
from turtlesim_plus_interfaces.srv import GivePosition

class TurtlesimPlusNode(Node):
    def __init__(self):
        super().__init__('turtlesim_plus_node')
        
        # Service server
        self.spawn_pizza_srv = self.create_service(
            GivePosition, '/spawn_pizza', self.spawn_pizza_callback)
        
        # Parameter
        self.declare_parameter('yaml_path', 'pizza_registry.yaml')
        self.yaml_path = self.get_parameter('yaml_path').value
        
        # State variables
        self.pizza_names = []
        
        self.get_logger().info('Turtlesim plus node started')
    
    def spawn_pizza_callback(self, request, response):
        # TODO: call /spawn_turtle with current pose
        # TODO: track pizza names in a list
        self.get_logger().info(f'Would spawn pizza at x={request.x:.2f}, y={request.y:.2f}')
        self.pizza_names.append(f'pizza_{len(self.pizza_names)}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimPlusNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()