# Assignment2_rt package
## Overview
`assignment2_rt` is a ROS2 package designed to control a mobile robot and prevent collisions with obstacle in a simulation environment.

### Features
- Set a linear and angular velocity for the robot
- Avoid the collision with obstacles
- If the robot is too close to an obstacle, steer it away in a secure position
- Set a new threshold
- Get the average velocity of the last five inputs

## Installation
### Prerequisites
- ROS2 Jazzy or newer
- Ubuntu 24.04

### Create the workspace
```
mkdir -p ~/ros_ws/src
```
### Clone the package
```
cd ~/ros_ws/src
git clone https://github.com/TZTozz/Assignment1_rt.git
cd ..
colcon build --packages-select assignment1_rt
```

## Usage
Setup the terminal:
```
cd ~/ros_ws
source install/local_setup.sh
```
Launcher:
```
ros2 launch assignment2_rt launch.py
```

## Nodes
<img width="70%" height="70%" alt="turtlesim(1)" src="https://github.com/user-attachments/assets/803d54f7-38da-49ab-ac27-695ae22a4e02" />

### Mobile controller
- Ask the user the desired linear and angular velocity.
- Moves the robot for three second.
- If the robot enters a prohibited zone, stop it for one second and then move it backward until it exits the restricted area.

### Distance 
- Read the output of the `LaserScan`.
- Find the minimum distance from the obstancle and computes the angle.
- If it is too close publish on the topic `stop`.

### Average velocity
It is a service that computes the average velocity of the last five inputs.

### Ask velocity
Enables the user to sent request at the service `averageVelocity`.

### Set threshold
Enables the user to set a new threshold for the prohibited zone. Send the request to the server in the node `distance`.

## Topic
| Topic | Type | Description |
| ---- | --- | --- |
| /cmd_vel | geometry_msgs/msg/Twist | Control the velocity of the robot |
| /scan | sensor_msgs/msg/LaserScan | Measurement of the laser |
| /stop | std_msgs/msg/Bool | Is true if the robot is in a prohibited zone |
| /distance | custom_msgs/msg/DistanceAngle | Distance and angle of the obstacle and the threshold |
| /user_input | geometry_msgs/msg/Twist | Input from the user |
