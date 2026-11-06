"""Master launch file: Simulation + Nav2 + Voice Navigation + RViz2."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = Path(share('turtlebot4_voice_navigation'))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'], description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'], description='RViz2 GUI'),
        DeclareLaunchArgument('backend', default_value='vosk', choices=['vosk', 'whisper', 'text_only'], description='Speech recognition engine'),
        DeclareLaunchArgument('language', default_value='vi', choices=['vi', 'en'], description='Speech language'),
        DeclareLaunchArgument('enable_mic', default_value='true', choices=['true', 'false'], description='Microphone capture'),

        # 1. Simulation (Gazebo)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg_share / 'launch/simulation.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items()
        ),

        # 2. Navigation (AMCL + Nav2 + RViz2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg_share / 'launch/navigation.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()
        ),

        # 3. Voice Navigation Nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(pkg_share / 'launch/voice_nav.launch.py')),
            launch_arguments={
                'backend': LaunchConfiguration('backend'),
                'language': LaunchConfiguration('language'),
                'enable_mic': LaunchConfiguration('enable_mic')
            }.items()
        ),
    ])
