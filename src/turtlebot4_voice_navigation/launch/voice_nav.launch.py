"""Launch Voice Recognition, Navigation Manager, and Zone Visualizer nodes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = Path(share('turtlebot4_voice_navigation'))
    default_zones = str(pkg_share / 'config/zones.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('zones_file', default_value=default_zones, description='Path to zones.yaml'),
        DeclareLaunchArgument('backend', default_value='vosk', choices=['vosk', 'whisper', 'text_only'], description='Speech recognition backend'),
        DeclareLaunchArgument('language', default_value='vi', choices=['vi', 'en'], description='Speech language'),
        DeclareLaunchArgument('enable_mic', default_value='true', choices=['true', 'false'], description='Enable microphone stream'),
        DeclareLaunchArgument('waypoint_dwell_seconds', default_value='2.0', description='Pause time between sequential waypoints'),

        # 1. 3D Semantic Zone Visualizer for RViz
        Node(
            package='turtlebot4_voice_navigation',
            executable='zone_visualizer_node',
            name='zone_visualizer_node',
            output='screen',
            parameters=[{'zones_file': LaunchConfiguration('zones_file')}]
        ),

        # 2. Voice Recognizer (Vosk / Whisper / Mic / Text fallback)
        Node(
            package='turtlebot4_voice_navigation',
            executable='voice_recognizer_node',
            name='voice_recognizer_node',
            output='screen',
            parameters=[{
                'backend': LaunchConfiguration('backend'),
                'language': LaunchConfiguration('language'),
                'enable_mic': ParameterValue(LaunchConfiguration('enable_mic'), value_type=bool)
            }]
        ),

        # 3. Voice Navigation Manager (NLU + Nav2 Action Client)
        Node(
            package='turtlebot4_voice_navigation',
            executable='voice_navigation_manager',
            name='voice_navigation_manager',
            output='screen',
            parameters=[{
                'zones_file': LaunchConfiguration('zones_file'),
                'waypoint_dwell_seconds': ParameterValue(LaunchConfiguration('waypoint_dwell_seconds'), value_type=float)
            }]
        ),
    ])
