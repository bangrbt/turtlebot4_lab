"""SLAM only; simulation must be running in a separate terminal."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav = Path(get_package_share_directory('turtlebot4_navigation'))
    lab = Path(get_package_share_directory('turtlebot4_lab'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav / 'launch/slam.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'sync': 'true', 'namespace': ''}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(lab / 'launch/view.launch.py')),
            launch_arguments={'enabled': LaunchConfiguration('rviz'), 'frame': 'map'}.items()),
    ])
