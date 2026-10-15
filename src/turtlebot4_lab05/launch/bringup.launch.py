"""Localization/Nav2 plus independent YOLO; autonomous mission is opt-in."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('turtlebot4_lab05'))
    lab = Path(get_package_share_directory('turtlebot4_lab'))
    args = [DeclareLaunchArgument('map', default_value=str(share/'maps/map.yaml')),
            DeclareLaunchArgument('waypoints', default_value=str(share/'config/waypoints.yaml')),
            DeclareLaunchArgument('start_yolo', default_value='true'),
            DeclareLaunchArgument('start_navigation', default_value='false'),
            DeclareLaunchArgument('localization_confirmed', default_value='false'),
            DeclareLaunchArgument('device', default_value='0'),
            DeclareLaunchArgument('camera_topic', default_value='/oakd/rgb/preview/image_raw'),
            DeclareLaunchArgument('weights', default_value='yolo11n.pt'),
            DeclareLaunchArgument('confidence', default_value='0.25'),
            DeclareLaunchArgument('image_size', default_value='640'),
            DeclareLaunchArgument('inference_rate', default_value='5.0'),
            DeclareLaunchArgument('show_window', default_value='true'),
            DeclareLaunchArgument('goal_timeout', default_value='300.0'),
            DeclareLaunchArgument('retries', default_value='1'),
            DeclareLaunchArgument('dwell_seconds', default_value='8.0'),
            DeclareLaunchArgument('rviz', default_value='true')]
    nodes = [GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(lab/'launch/navigation.launch.py')),
                 launch_arguments={'map': LaunchConfiguration('map'), 'nav2': 'true', 'rviz': 'false'}.items())]),
             Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                  condition=IfCondition(LaunchConfiguration('rviz')),
                  arguments=['-d', str(share/'config/lab05.rviz')], parameters=[{'use_sim_time': True}],
                  additional_env={'QT_SCALE_FACTOR':'1', 'QT_AUTO_SCREEN_SCALE_FACTOR':'0', 'QT_ENABLE_HIGHDPI_SCALING':'0'}),
             Node(package='turtlebot4_lab05', executable='yolo_node', output='screen',
                  condition=IfCondition(LaunchConfiguration('start_yolo')),
                  parameters=[str(share/'config/yolo.yaml'),
                      {'device': ParameterValue(LaunchConfiguration('device'),value_type=str), 'camera_topic': LaunchConfiguration('camera_topic'),
                       'weights': LaunchConfiguration('weights'),
                       'confidence':ParameterValue(LaunchConfiguration('confidence'),value_type=float),
                       'image_size':ParameterValue(LaunchConfiguration('image_size'),value_type=int),
                       'inference_rate':ParameterValue(LaunchConfiguration('inference_rate'),value_type=float),
                       'show_window':ParameterValue(LaunchConfiguration('show_window'),value_type=bool)}]),
             Node(package='turtlebot4_lab05', executable='sequential_navigation', output='screen',
                  condition=IfCondition(LaunchConfiguration('start_navigation')),
                  parameters=[str(share/'config/navigation.yaml'),
                      {'waypoint_file':LaunchConfiguration('waypoints'), 'map_file':LaunchConfiguration('map'),
                       'localization_confirmed':ParameterValue(LaunchConfiguration('localization_confirmed'),value_type=bool),
                       'goal_timeout':ParameterValue(LaunchConfiguration('goal_timeout'),value_type=float),
                       'retries':ParameterValue(LaunchConfiguration('retries'),value_type=int),
                       'dwell_seconds':ParameterValue(LaunchConfiguration('dwell_seconds'),value_type=float)}])]
    return LaunchDescription(args+nodes)
