import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_bme_gazebo_sensors = get_package_share_directory('bme_gazebo_sensors')

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_gazebo_sensors, 'launch', 'spawn_robot.launch.py'),
        ),
        launch_arguments={
            'x': '0.0',
            'y': '0.0',
            'rviz': 'true',
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([

        spawn_robot_launch,

        Node(
            package='assignment2_rt',
            executable='mobileController',
            name='controller',
            prefix='gnome-terminal --', 
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),

        Node(
            package='assignment2_rt',
            executable='distance',
            name='distance',
            prefix='gnome-terminal --', 
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),

        Node(
            package='assignment2_rt',
            executable='change_threshold',
            name='change_threshold',
            prefix='gnome-terminal --', 
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),

        Node(
            package='assignment2_rt',
            executable='averageVelocity',
            name='average_velocity',
            arguments=['--ros-args', '--log-level', 'info']
        ),

        Node(
            package='assignment2_rt',
            executable='askAverage',
            name='ask_average',
            prefix='gnome-terminal --', 
            output='screen',
            arguments=['--ros-args', '--log-level', 'info']
        ),

    ])