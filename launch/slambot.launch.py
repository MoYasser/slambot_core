import os

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') #for A1/A2 is 115200
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    # # Include the default rplidar_ros launch file with default parameters.
    # # If you need to customize any rplidar parameters uncomment and use the
    # # the launch config below.
    # rplidar_launchDescription = IncludeLaunchDescription(
        # PythonLaunchDescriptionSource(
            # os.path.join(
                # get_package_share_directory('rplidar_ros'),
                # 'launch',
                # 'rplidar.launch.py')),
        # launch_arguments={}.items()
    # )

    # This launch action (node) was copied from rplidar.launch.py
    rplidar_launchDescription =Node(
                package='rplidar_ros2',
                executable='rplidar_scan_publisher',
                name='rplidar_scan_publisher',
                parameters=[{'serial_port': '/dev/ttyUSB0',
                             'serial_baudrate': 115200,
                             'frame_id': 'laser',
                             'inverted': False,
                             'angle_compensate': True}],
                output='screen',
        )

    slambotaux_launchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slambot_core'),
                'launch',
                'slambotaux.launch.py')),
        launch_arguments={}.items()
        )

        ld = LaunchDescription()
        ld.add_action(rplidar_launchDescription)
        ld.add_action(slambotaux_launchDescription)

        return ld
