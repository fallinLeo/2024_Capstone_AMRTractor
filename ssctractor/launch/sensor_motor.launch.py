import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    imu_publisher_path = '/home/leo/ros2_ws/src/2024_capstone/ZLAC8015D_python/examples/imu_publisher.py'
    joint_pub_controller_path = '/home/leo/ros2_ws/src/2024_capstone/ZLAC8015D_python/examples/joint_pub_controller.py'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', imu_publisher_path],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['python3', joint_pub_controller_path],
            output='screen'
        ),
    ])

