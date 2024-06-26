#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

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

    robot_file = "skidbot.urdf"
    #robot_file = "tractor.urdf"
    package_name = "leo_gazebo_simulation"
    world_file_name = "bocbot_office.world"

    # full  path to urdf and world file
    world = os.path.join(
        get_package_share_directory(package_name), "worlds", world_file_name
    )
    urdf = os.path.join(get_package_share_directory(package_name), "urdf", robot_file)

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    xml = open(urdf, "r").read()
    xml = xml.replace('"', '\\"')
    spwan_args = '{name: "skidbot", xml: "' + xml + '"  }'
    

    #추가한 state_publisher들
    doc = xacro.parse(open(urdf))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
                ),
                launch_arguments={"world": world}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
                ),
            ),
            ExecuteProcess(
                cmd=["ros2", "param", "set", "/gazebo", "use_sim_time", use_sim_time],
                output="screen",
            ),
            Node(
                package='py_service_pkg',
                executable='spawn_skidbot',
                name='spawn_skidbot',
                output='screen'
            ),
            robot_state_publisher_node,
            joint_state_publisher
            # ExecuteProcess(
            #     cmd=[
            #         "ros2",
            #         "service",
            #         "call",
            #         "/spawn_entity",
            #         "gazebo_msgs/SpawnEntity",
            #         spwan_args,
            #     ],
            #     output="screen",
            # ),

        ]
    )
