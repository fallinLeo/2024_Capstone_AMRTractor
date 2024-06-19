#!/usr/bin/env python3


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from geometry_msgs.msg import Pose
import xacro


def generate_launch_description():

    robot_file = "tractor.urdf"
    package_name = "leo_gazebo_simulation"
    world_file_name = "Basic_world.world" #Basic_world.world

    # full  path to urdf and world file
    world = os.path.join(
        get_package_share_directory(package_name), "worlds", world_file_name
    )
    urdf = os.path.join(get_package_share_directory(package_name), "urdf", robot_file)

    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    xml = open(urdf, "r").read()
    xml = xml.replace('"', '\\"')
    spwan_args = '{name: "tractor", xml: "' + xml + '"  }'
    

    #추가한 state_publisher들
    doc = xacro.parse(open(urdf))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    start_gazebo_server_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
                ),
                launch_arguments={"world": world}.items(),
            )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'tractor', '-x', '0.0', '-y', '1.5', '-z', '3.0']
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )


    return LaunchDescription(
        [
            start_gazebo_server_cmd,
            start_gazebo_client_cmd, # gazebo gui
            ExecuteProcess(
                cmd=["ros2", "param", "set", "/gazebo", "use_sim_time", use_sim_time],
                output="screen",
            ),
            robot_state_publisher_node,
            joint_state_publisher,
            spawn_entity
        ]
    )
