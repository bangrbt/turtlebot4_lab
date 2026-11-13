#!/usr/bin/env python3
"""Main launch file for TurtleBot 4 Facial Emotion Control system."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot4_facial_emotion_control')
    default_config = os.path.join(pkg_dir, 'config', 'emotion_params.yaml')

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to launch the PyQt5 Emotion Studio GUI'
    )
    declare_config = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to YAML configuration file'
    )
    declare_camera_idx = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera device index (/dev/videoX)'
    )
    declare_show_window = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='Whether to display OpenCV detection window'
    )

    recognition_node = Node(
        package='turtlebot4_facial_emotion_control',
        executable='emotion_recognition_node',
        name='emotion_recognition_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'camera_index': LaunchConfiguration('camera_index'),
                'show_window': LaunchConfiguration('show_window'),
            }
        ],
    )

    behavior_node = Node(
        package='turtlebot4_facial_emotion_control',
        executable='robot_behavior_node',
        name='robot_behavior_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
    )

    gui_node = Node(
        package='turtlebot4_facial_emotion_control',
        executable='emotion_gui',
        name='emotion_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    return LaunchDescription([
        declare_gui,
        declare_config,
        declare_camera_idx,
        declare_show_window,
        recognition_node,
        behavior_node,
        gui_node,
    ])
