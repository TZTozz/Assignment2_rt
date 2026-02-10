#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.duration import Duration

class MobileController(Node):

    def __init__(self):
        super().__init__('UI')
        self.publisherVelocity_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisherInput_ = self.create_publisher(Twist, '/user_input', 10)

        self.subscription = self.create_subscription(Bool, 'stop', self.listener_callback, 10)
        
        timer_period = 0.01 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        #Time
        self.start_time = self.get_clock().now()
        self.lastTime = self.start_time - Duration(seconds = 6.0) 
        self.duration = 3.0  #(s)
        self.durationStop = 1.0 #(s)

        self.message = Twist()
        self.needStop = False
        self.firstStop = True
    
    def listener_callback(self, msg):
        self.needStop = msg.data
        if self.firstStop: self.timeStop = self.get_clock().now()
        

    def timer_callback(self):
        #Computed the elapsed time
        current_time = self.get_clock().now()
        elapsed_time_command = (current_time - self.lastTime).nanoseconds / 1e9
        
        if not self.needStop:

            if elapsed_time_command > self.duration:        #More than a second
                
                #Stops the robot
                self.message.linear.x = 0.0
                self.message.angular.z = 0.0
                self.publisherVelocity_.publish(self.message)

                #Ask the velocity
                while True:
                    try:
                        self.vel = float(input("Linear velocity? "))
                        break
                    except ValueError:
                        print("Error: not a valid input")
                while True:
                    try:
                        self.angular = float(input("Angular velocity? "))
                        break
                    except ValueError:
                        print("Error: not a valid input")

                self.message.linear.x = self.vel
                self.message.angular.z = self.angular
                self.get_logger().info(f'The velocity of robot is {self.message.linear}')
                
                #Send it to the robot
                self.publisherVelocity_.publish(self.message)
                self.publisherInput_.publish(self.message)
                self.lastTime = self.get_clock().now()

            else:       #Less than a second
                self.get_logger().info(f'Still sending to the robot the linear velocity of {self.vel} and angular velocity of {self.angular}')
                self.publisherVelocity_.publish(self.message)


            self.firstStop = True
        else:                           #Stops the turtles
            elapsed_time_stop = (current_time - self.timeStop).nanoseconds / 1e9     #Time passed from the stop
            
            if self.firstStop:                                  #If it's the first time to stop
                self.tempLinear = self.message.linear.x         #Save the turtle input
                self.tempAngular = self.message.angular.z
                self.firstStop = False

            if elapsed_time_stop < self.durationStop:        #Wait for a second
                self.message.linear.x = 0.0
                self.message.angular.z = 0.0
                self.get_logger().info('Stopped Robot')
            else:                                   #Moves the robot in the opposit direction
                self.message.linear.x = -self.tempLinear
                self.message.angular.z = -self.tempAngular
                self.get_logger().info('Moving robot in the opposite direction')


            self.publisherVelocity_.publish(self.message)
            

        

            

def main(args=None):
    rclpy.init(args=args)
    node = MobileController()

    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Main").info("Nodo terminato.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()