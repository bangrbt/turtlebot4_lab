from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('camera_index',default_value='0'),
        Node(package='turtlebot4_gesture_control',executable='gesture_recognition_node',output='screen',
             parameters=[{'inference_enabled':False,'show_window':True,
                          'camera_index':ParameterValue(LaunchConfiguration('camera_index'),value_type=int)}])])
