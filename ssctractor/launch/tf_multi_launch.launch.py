from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # rplidar_ros 패키지의 런치 파일 경로를 구합니다.
    rplidar_launch_file = os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar.launch.py')

    # ssctractor 패키지의 런치 파일 경로를 구합니다.
    robot_state_publisher_launch_file = os.path.join(get_package_share_directory('ssctractor'), 'launch', 'robot_state_publisher.launch.py')

    return LaunchDescription([
        # rplidar 런치 파일 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch_file),
            launch_arguments={'parameter_name': 'parameter_value'}.items(), # 필요한 파라미터가 있다면 여기에 추가
        ),

        # robot_state_publisher 런치 파일 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_publisher_launch_file),
            launch_arguments={'parameter_name': 'parameter_value'}.items(), # 필요한 파라미터가 있다면 여기에 추가
        ),

        # odom_publisher 노드 실행
        Node(
            package='ssctractor',
            executable='odom_publisher',
            name='odom_publisher'
        )
    ])
