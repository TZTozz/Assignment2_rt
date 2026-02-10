#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from turtlesim_custom_msgs.srv import VelocityComponents

class AskAverageVelocity(Node):

    def __init__(self):
        super().__init__('average_velocity_client')

        self.cli = self.create_client(VelocityComponents, 'average_velocity')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the service')
        
        self.req = VelocityComponents.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    client = AskAverageVelocity()

    try:
        while rclpy.ok():
            user_input = input("Press ENTER to request average velocity (or 'q' to quit): ")
            
            if user_input.lower() == 'q':
                break
            
            response = client.send_request()
            
            print(f"Avarage velocity:\nLinear: {response.linear:.2f}, Angular: {response.angular:.2f}")

    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()