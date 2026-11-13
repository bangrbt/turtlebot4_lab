#!/usr/bin/env python3
"""Launch Ignition Gazebo with an open arena and TurtleBot 4 for emotion driving."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('turtlebot4_facial_emotion_control')
    world_path = os.path.join(pkg_dir, 'worlds', 'empty_arena.sdf')

    tb4_bringup = get_package_share_directory('turtlebot4_ignition_bringup')
    resources = ':'.join(
        str(Path(get_package_share_directory(n)).parent)
        for n in ['turtlebot4_description', 'irobot_create_description']
    )

    declare_gui = DeclareLaunchArgument('gazebo_gui', default_value='true', choices=['true', 'false'])

    ign_gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-v', '2', world_path],
        output='screen',
        on_exit=Shutdown(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    tb4_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb4_bringup, 'launch', 'turtlebot4_spawn.launch.py')
        ),
        launch_arguments={
            'world': 'emotion_arena',
            'model': 'standard',
            'use_sim_time': 'true',
            'rviz': 'false',
            'nav2': 'false',
            'slam': 'false',
            'localization': 'false',
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'yaw': '0.0',
        }.items(),
    )

    return LaunchDescription([
        declare_gui,
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resources),
        ign_gazebo,
        clock_bridge,
        tb4_spawn,
    ])
