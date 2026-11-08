"""Launch Ignition Gazebo simulation with Smart Campus world, 3D models,
full sensor/control bridges, and TurtleBot 4 standard robot.
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

    # Resource search paths for Gazebo to load both robot URDFs and realistic 3D mesh assets
    resource_paths = [
        str(Path(share('turtlebot4_description')).parent),
        str(Path(share('irobot_create_description')).parent),
        str(pkg_share / 'models'),
        str(pkg_share.parent.parent / 'src/turtlebot4_voice_navigation/models'),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ]
    ign_resources = ':'.join(p for p in resource_paths if p)

    system_plugins = os.environ.get('LD_LIBRARY_PATH', '') + ':' + str(Path(share('ign_ros2_control')).parents[1] / 'lib')
    gui_plugins = ':'.join(str(Path(share(n)) / 'lib') for n in ['turtlebot4_ignition_gui_plugins', 'irobot_create_ignition_plugins'])

    # Bridge arguments for ROS-Ignition communication
    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'], description='Enable Gazebo GUI'),
        DeclareLaunchArgument('model', default_value='standard', choices=['standard'], description='TurtleBot 4 model'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Initial spawn X coordinate'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Initial spawn Y coordinate'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='Initial spawn Yaw orientation'),

        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_resources),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', system_plugins),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', gui_plugins),

        OpaqueFunction(function=simulator),

        # Bridge LiDAR /scan and /clock from Ignition Gazebo to ROS 2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='turtlebot4_voice_bridge',
            output='screen',
            arguments=bridge_args
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

        # Auto-prepare simulation: un-dock robot so Nav2 /cmd_vel operates freely without motor lock
        Node(
            package='turtlebot4_voice_navigation',
            executable='prepare_voice_simulation',
            name='prepare_voice_simulation',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
