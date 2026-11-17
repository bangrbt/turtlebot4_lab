"""Launch Map Server, AMCL Localization, Nav2 Stack, and RViz2 for Smart Campus."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(share('turtlebot4_voice_navigation'))
    nav_share = Path(share('turtlebot4_navigation'))

    default_map = str(pkg_share / 'maps/map.yaml')
    default_rviz = str(pkg_share / 'config/voice_nav.rviz')
    default_params = str(pkg_share / 'config/localization.yaml')
    default_nav2_params = str(pkg_share / 'config/nav2.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map, description='Path to map.yaml'),
        DeclareLaunchArgument('params', default_value=default_params, description='Path to localization.yaml'),
        DeclareLaunchArgument('nav2_params', default_value=default_nav2_params, description='Path to nav2.yaml'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'], description='Start RViz2'),
        DeclareLaunchArgument('nav2', default_value='true', choices=['true', 'false'], description='Start Nav2'),

        # AMCL & Map Server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_share / 'launch/localization.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'map': LaunchConfiguration('map'),
                'params': LaunchConfiguration('params'),
                'namespace': ''
            }.items()
        ),

        # Auto Initial Pose: Publishes /initialpose (0, 0) and waits for map -> base_link
        Node(
            package='turtlebot4_voice_navigation',
            executable='auto_initial_pose',
            name='auto_initial_pose',
            output='screen',
            parameters=[{'use_sim_time': True, 'initial_x': 0.0, 'initial_y': 0.0, 'initial_yaw': 0.0}]
        ),

        # Nav2 Navigation Stack with robust transform tolerance
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(nav_share / 'launch/nav2.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': LaunchConfiguration('nav2_params'),
                'namespace': ''
            }.items(),
            condition=IfCondition(LaunchConfiguration('nav2'))
        ),

        # RViz2 with pre-configured displays and semantic markers
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
            arguments=['-d', default_rviz],
            parameters=[{'use_sim_time': True}]
        ),
    ])
