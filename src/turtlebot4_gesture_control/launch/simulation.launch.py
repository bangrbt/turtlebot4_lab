"""Roomy local world, official TB4 standard robot; no Nav2/SLAM/teleop GUI."""
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,ExecuteProcess,Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def simulator(context):
    package=Path(share('turtlebot4_gesture_control'))
    command=['ign','gazebo','-r','-v','2',str(package/'worlds/gesture_arena.sdf')]
    if LaunchConfiguration('gui').perform(context)=='true':command+=['--gui-config',str(package/'config/arena_gui.config')]
    else:command+=['-s','--headless-rendering']
    return [ExecuteProcess(cmd=command,output='screen',on_exit=Shutdown())]


def generate_launch_description():
    official=Path(share('turtlebot4_ignition_bringup'))
    resources=':'.join(str(Path(share(n)).parent) for n in ['turtlebot4_description','irobot_create_description'])
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('world',default_value='gesture_arena',choices=['gesture_arena']),
        DeclareLaunchArgument('model',default_value='standard',choices=['standard']),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',resources),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',os.environ.get('LD_LIBRARY_PATH','')+':'+str(Path(share('ign_ros2_control')).parents[1]/'lib')),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH',':'.join(str(Path(share(n))/'lib') for n in ['turtlebot4_ignition_gui_plugins','irobot_create_ignition_plugins'])),
        OpaqueFunction(function=simulator),
        Node(package='ros_gz_bridge',executable='parameter_bridge',name='gesture_clock_bridge',output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(official/'launch/turtlebot4_spawn.launch.py')),
            launch_arguments={'world':'gesture_arena','model':'standard','use_sim_time':'true','rviz':'false',
                              'nav2':'false','slam':'false','localization':'false','x':'0.0','y':'0.0','z':'0.0','yaw':'0.0'}.items()),
        Node(package='turtlebot4_gesture_control',executable='prepare_simulation',parameters=[{'use_sim_time':True}],output='screen')])
