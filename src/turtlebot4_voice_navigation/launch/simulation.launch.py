"""Launch Ignition Gazebo simulation with Smart Campus world and TurtleBot 4."""
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
        command += ['--gui-config', str(gui_cfg)]
    else:
        command += ['-s', '--headless-rendering']

    return [ExecuteProcess(cmd=command, output='screen', on_exit=Shutdown())]


def generate_launch_description():
    official = Path(share('turtlebot4_ignition_bringup'))
    resources = ':'.join(str(Path(share(n)).parent) for n in ['turtlebot4_description', 'irobot_create_description'])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'], description='Enable Gazebo GUI'),
        DeclareLaunchArgument('model', default_value='standard', choices=['standard', 'lite'], description='TurtleBot 4 model'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Initial spawn X coordinate'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Initial spawn Y coordinate'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Initial spawn Yaw orientation'),

        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resources),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', os.environ.get('LD_LIBRARY_PATH', '') + ':' + str(Path(share('ign_ros2_control')).parents[1] / 'lib')),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', ':'.join(str(Path(share(n)) / 'lib') for n in ['turtlebot4_ignition_gui_plugins', 'irobot_create_ignition_plugins'])),

        OpaqueFunction(function=simulator),

        # Bridge clock from Ignition Gazebo to ROS 2
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
                'model': LaunchConfiguration('model'),
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
    ])
