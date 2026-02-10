#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from turtlesim_custom_msgs.srv import AvarageVelocity_srv 

class AskAverageVelocity(Node):

    def __init__(self):
        super().__init__('average_velocity_client')

        self.cli = self.create_client(AvarageVelocity_srv, 'average_velocity')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for the service')
        
        self.req = AvarageVelocity_srv.Request()

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
            
            print("Sended request...")
            response = client.send_request()
            
            # Qui assumiamo che la risposta abbia i campi linear e angular
            # Adatta questi campi in base alla definizione del tuo .srv
            print(f"Avarage velocity:\nLinear: {response.linear:.2f}, Angular: {response.angular:.2f}")
            print("-" * 30)

    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()