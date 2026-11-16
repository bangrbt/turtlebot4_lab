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

    sys_plugins = os.environ.get('LD_LIBRARY_PATH', '') + ':' + str(Path(get_package_share_directory('ign_ros2_control')).parents[1] / 'lib')
    gui_plugins = ':'.join(str(Path(get_package_share_directory(n)) / 'lib') for n in ['turtlebot4_ignition_gui_plugins', 'irobot_create_ignition_plugins'])

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

    prepare_sim_node = Node(
        package='turtlebot4_facial_emotion_control',
        executable='prepare_simulation',
        name='emotion_prepare_simulation',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        declare_gui,
        SetEnvironmentVariable('ROS_DOMAIN_ID', '88'),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('IGN_IP', '127.0.0.1'),
        SetEnvironmentVariable('IGN_PARTITION', 'turtlebot4_emotion_88'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resources),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', sys_plugins),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', gui_plugins),
        ign_gazebo,
        clock_bridge,
        tb4_spawn,
        prepare_sim_node,
    ])
