"""One official Humble simulator; no SLAM, AMCL or Nav2."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description():
    official = Path(get_package_share_directory('turtlebot4_ignition_bringup'))
    lab = Path(get_package_share_directory('turtlebot4_lab'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='maze', choices=['maze']),
        DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        SetLaunchConfiguration('lab_rviz_enabled', LaunchConfiguration('rviz')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(official / 'launch/turtlebot4_ignition.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'),
                'model': LaunchConfiguration('model'), 'use_sim_time': 'true',
                'rviz': 'false', 'slam': 'false', 'localization': 'false', 'nav2': 'false',
                'x': '0.0', 'y': '0.0', 'z': '0.0', 'yaw': '0.0'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lab / 'launch/view.launch.py')),
            launch_arguments={'enabled': LaunchConfiguration('lab_rviz_enabled'), 'frame': 'odom'}.items()),
    ])
