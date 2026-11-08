"""Launch Ignition Gazebo simulation with Smart Campus world, 3D models,
clock bridge, TurtleBot 4 standard robot, and auto dock clearance.
"""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction, ExecuteProcess, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def simulator(context):
    pkg_share = Path(share('turtlebot4_voice_navigation'))
    world_sdf = pkg_share / 'worlds/smart_campus.sdf'
    gui_cfg = pkg_share / 'config/campus_gui.config'

    command = ['ign', 'gazebo', '-r', '-v', '2', str(world_sdf)]
    if LaunchConfiguration('gui').perform(context) == 'true':
        if gui_cfg.is_file():
            command += ['--gui-config', str(gui_cfg)]
    else:
        command += ['-s', '--headless-rendering']

    return [ExecuteProcess(cmd=command, output='screen', on_exit=Shutdown())]


def generate_launch_description():
    pkg_share = Path(share('turtlebot4_voice_navigation'))
    official = Path(share('turtlebot4_ignition_bringup'))

    # Base description paths for robot meshes, plus package 3D models
    desc_paths = [str(Path(share(n)).parent) for n in ['turtlebot4_description', 'irobot_create_description']]
    model_paths = [
        str(pkg_share / 'models'),
        str(pkg_share.parent.parent / 'src/turtlebot4_voice_navigation/models'),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ]
    all_resources = ':'.join(p for p in desc_paths + model_paths if p)

    system_plugins = os.environ.get('LD_LIBRARY_PATH', '') + ':' + str(Path(share('ign_ros2_control')).parents[1] / 'lib')
    gui_plugins = ':'.join(str(Path(share(n)) / 'lib') for n in ['turtlebot4_ignition_gui_plugins', 'irobot_create_ignition_plugins'])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'], description='Enable Gazebo GUI'),
        DeclareLaunchArgument('world', default_value='smart_campus', choices=['smart_campus'], description='World name'),
        DeclareLaunchArgument('model', default_value='standard', choices=['standard'], description='TurtleBot 4 model'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Initial spawn X coordinate'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Initial spawn Y coordinate'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Initial spawn Yaw orientation'),

        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', all_resources),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', system_plugins),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', gui_plugins),

        OpaqueFunction(function=simulator),

        # Bridge clock from Ignition Gazebo to ROS 2 (lidar/odom/tf are bridged by turtlebot4_spawn)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='voice_clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
        ),

        # Spawn TurtleBot 4 in Smart Campus world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(official / 'launch/turtlebot4_spawn.launch.py')),
            launch_arguments={
                'world': 'smart_campus',
                'model': 'standard',
                'use_sim_time': 'true',
                'rviz': 'false',
                'nav2': 'false',
                'slam': 'false',
                'localization': 'false',
                'x': LaunchConfiguration('spawn_x'),
                'y': LaunchConfiguration('spawn_y'),
                'z': '0.0',
                'yaw': LaunchConfiguration('spawn_yaw')
            }.items()
        ),

        # Auto-prepare simulation: un-dock robot so Nav2 /cmd_vel operates freely without motor lock
        Node(
            package='turtlebot4_voice_navigation',
            executable='prepare_voice_simulation',
            name='prepare_voice_simulation',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
