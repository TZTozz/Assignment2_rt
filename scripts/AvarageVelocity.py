#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim_custom_msgs.srv import AverageVelocity_srv

class AverageVelocity(Node):

    def __init__(self):
        super().__init__('average_velocity')
        self.subscription = self.create_subscription(Twist, '/user_input', self.listener_callback, 10)
        self.srv = self.create_service(AverageVelocity_srv, 'average_velocity', self.handle_service)
        self.velocities = []
    
    def listener_callback(self, msg):
        self.velocities.append(msg)
        if len(self.velocities) > 5:
            self.velocities.pop(0)


    def handle_service(self, request, response):
        if self.velocities:
            average_velocity = sum(self.velocities) / len(self.velocities)
            response.linear = average_velocity.linear.x
            response.angular = average_velocity.angular.z
        else:
            response.linear = 0.0
            response.angular = 0.0
        return response