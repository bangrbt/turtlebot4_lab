from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    p=Path(get_package_share_directory('turtlebot4_gesture_control'))
    return LaunchDescription([
        DeclareLaunchArgument('camera_index',default_value='0'),
        DeclareLaunchArgument('show_window',default_value='true',choices=['true','false']),
        DeclareLaunchArgument('enable_motion',default_value='true',choices=['true','false']),
        Node(package='turtlebot4_gesture_control',executable='gesture_recognition_node',output='screen',
             parameters=[str(p/'config/gesture.yaml'),{'camera_index':ParameterValue(LaunchConfiguration('camera_index'),value_type=int),
                         'show_window':ParameterValue(LaunchConfiguration('show_window'),value_type=bool)}]),
        Node(package='turtlebot4_gesture_control',executable='robot_behavior_node',output='screen',
             parameters=[str(p/'config/behavior.yaml'),{'enable_motion':ParameterValue(LaunchConfiguration('enable_motion'),value_type=bool)}])])
