from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    p=Path(get_package_share_directory('turtlebot4_gesture_control'))/'launch'
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('camera_index',default_value='0'),
        GroupAction([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(p/'simulation.launch.py')),
                     launch_arguments={'gui':LaunchConfiguration('gui')}.items())]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(p/'control.launch.py')),
                     launch_arguments={'camera_index':LaunchConfiguration('camera_index')}.items())])
