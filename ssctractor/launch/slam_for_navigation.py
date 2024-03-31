#!/usr/bin/env python3


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription(
        [
            # ExecuteProcess(
            #     cmd=["ros2", "launch", "ssctractor", "robot_state_publisher.py"],
            # ),
            ExecuteProcess(
                cmd=["ros2", "launch", "cartographer_2dslam", "cartographer.launch.py", "use_sim_time:=false"],
                output="screen"
            ),
            # ExecuteProcess(
            #     cmd=["ros2", "launch", "ssctractor", "robot_state_publisher.launch.py", "use_sim_time:=false"],
            #     output="screen"
            # ),
            ExecuteProcess(
            	cmd=["ros2", "run", "ssctractor", "odom_publisher"]
            	),  
#            ExecuteProcess(
#            	cmd=["ros2", "run", "move", "move"]
#            	),  
            ExecuteProcess(
            	cmd=["ros2", "launch", "rplidar_ros", "rplidar.launch.py"]
            	),  
        ]
    )
