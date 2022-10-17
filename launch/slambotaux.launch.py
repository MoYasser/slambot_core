import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          choices=['true', 'false'],
                          description='Use sim time'),
]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    xacro_file = os.path.join(
        get_package_share_directory('slambot_core'),
        'urdf',
        'head_2d.urdf.xacro')

    start_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro', ' ', xacro_file, ' ', 'gazebo:=ignition'])
        }]
    )

    slam_config = PathJoinSubstitution(
        [slambot_core, 'config', 'slam_async.yaml'])

    start_async_slam_toolbox_node = Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
              slam_config,
              {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        )

    ld = LaunchDescription(ARGUMENTS)

    ld.add_action(start_async_slam_toolbox_node)

    return ld