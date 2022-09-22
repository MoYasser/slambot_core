import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

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
    rplidar_launchDescription = ([

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),


        Node(
            package='rplidar_ros2',
            executable='rplidar_scan_publisher',
            name='rplidar_scan_publisher',
            parameters=[{'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate}],
            output='screen'),
    ])

    gbot_ros2_no_lidar_launchDescription = IncludeLaunchDescription(
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

