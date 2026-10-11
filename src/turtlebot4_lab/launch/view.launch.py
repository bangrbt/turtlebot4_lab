"""RViz only, using the description and TF already published by the simulator."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def view(context):
    if LaunchConfiguration('enabled').perform(context) != 'true':
        return []
    frame = LaunchConfiguration('frame').perform(context)
    config = Path(get_package_share_directory('turtlebot4_lab')) / 'config' / (frame + '.rviz')
    return [Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                 arguments=['-d', str(config)], parameters=[{'use_sim_time': True}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('frame', default_value='map', choices=['odom', 'map']),
        OpaqueFunction(function=view),
    ])
