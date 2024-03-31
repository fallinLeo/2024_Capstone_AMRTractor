from setuptools import setup
import os

package_name = 'ssctractor'

launch_directory = os.path.join('share', package_name, 'launch')

launch_files = [
    'launch/slam.launch.py',
    'launch/robot_state_publisher.launch.py'
]

urdf_directory = os.path.join('share', package_name, 'urdf')
urdf_files = [
	'urdf/tractor_notgazebo.urdf',
	'urdf/tractor_only_odom.urdf',	
	
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (launch_directory, launch_files),
        (urdf_directory, urdf_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='122957896+fallinLeo@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'odom_publisher = ssctractor.odom_publisher:main',
        	'path_subscriber = ssctractor.topic_subscriber_path:main',
        ],
    },
)

