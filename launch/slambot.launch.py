import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo

ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='turtlebot4',
                          description='Robot name')
]

def generate_launch_description():
    rplidar_launchDescription = Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'rplidar_link',
                'inverted': False,
                'angle_compensate': True,
            }],
        )

    rplidar_stf = Node(
            name='rplidar_stf',
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '0', '0', '0', '0', '0.0', '0.0',
                'rplidar_link', [LaunchConfiguration('robot_name'), '/rplidar_link/rplidar']]
        )


    slambotaux_launchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slambot_core'),
                'launch',
                'slambotaux.launch.py')),
        launch_arguments={}.items()
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rplidar_launchDescription)
    ld.add_action(rplidar_stf)
    ld.add_action(slambotaux_launchDescription)

    return ld
