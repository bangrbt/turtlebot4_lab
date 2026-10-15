from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('turtlebot4_lab05'))
    return LaunchDescription([
        DeclareLaunchArgument('device',default_value='0'),
        DeclareLaunchArgument('weights',default_value='yolo11n.pt'),
        DeclareLaunchArgument('camera_topic',default_value='/oakd/rgb/preview/image_raw'),
        DeclareLaunchArgument('show_window',default_value='true'),
        DeclareLaunchArgument('log_file',default_value=''),
        Node(package='turtlebot4_lab05',executable='yolo_node',output='screen',parameters=[str(share/'config/yolo.yaml'),
            {'device':ParameterValue(LaunchConfiguration('device'),value_type=str),'weights':LaunchConfiguration('weights'),
             'camera_topic':LaunchConfiguration('camera_topic'),
             'log_file':ParameterValue(LaunchConfiguration('log_file'),value_type=str),
             'show_window':ParameterValue(LaunchConfiguration('show_window'),value_type=bool)}])])
