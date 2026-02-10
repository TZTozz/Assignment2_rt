#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim_custom_msgs.srv import VelocityComponents

class AverageVelocity(Node):

    def __init__(self):
        super().__init__('average_velocity')
        self.subscription = self.create_subscription(Twist, '/user_input', self.listener_callback, 10)
        self.srv = self.create_service(VelocityComponents, 'average_velocity', self.handle_service)
        self.velocities = []
    
    def listener_callback(self, msg):
        self.velocities.append(msg)
        if len(self.velocities) > 5:
            self.velocities.pop(0)


    def handle_service(self, request, response):
        if self.velocities:
            sum_linear = sum(v.linear.x for v in self.velocities)
            sum_angular = sum(v.angular.z for v in self.velocities)
            average_linear = sum_linear / len(self.velocities)
            average_angular = sum_angular / len(self.velocities)
            response.linear = average_linear
            response.angular = average_angular
        else:
            response.linear = 0.0
            response.angular = 0.0
        return response
    
def main(args=None):
    rclpy.init(args=args)
    
    node = AverageVelocity()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()