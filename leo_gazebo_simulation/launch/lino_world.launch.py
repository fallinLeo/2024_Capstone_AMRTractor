import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = "leo_gazebo_simulation"
    world_file_name = "Basic_world.world" #Basic_world.world
    urdf = PathJoinSubstitution(
        [FindPackageShare("linorobot2_description"), "urdf/robots", "2wd.urdf.xacro"]
    )
    world = os.path.join(
        get_package_share_directory(package_name), "worlds", world_file_name
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'linorobot2_2wd', '-x', '0.0', '-y', '1.5', '-z', '3.0']
    )

    start_gazebo_server_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
            ),
            launch_arguments={"world": world}.items(),
        )
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    return LaunchDescription([
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity_node
    ])
