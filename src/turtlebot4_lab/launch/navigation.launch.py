"""AMCL + map server, optionally Nav2; never starts robot or SLAM."""
from pathlib import Path
import math
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def navigation(context):
    path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not path.is_file():
        raise RuntimeError(f'Chua co ban do thuc: {path}. Hoan tat teleop va save_map.py truoc.')
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or not isinstance(data.get('image'), str):
        raise RuntimeError('Map YAML khong co image hop le.')
    image = path.parent / data['image']
    if not image.is_file():
        raise RuntimeError(f'Khong tim thay anh ban do: {image}')
    resolution = float(data.get('resolution', 0))
    if not math.isfinite(resolution) or resolution <= 0:
        raise RuntimeError('Map resolution phai duong va huu han.')
    nav = Path(get_package_share_directory('turtlebot4_navigation'))
    lab = Path(get_package_share_directory('turtlebot4_lab'))
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav / 'launch/localization.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'map': str(path), 'namespace': ''}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav / 'launch/nav2.launch.py')),
            launch_arguments={'use_sim_time': 'true', 'namespace': ''}.items(),
            condition=IfCondition(LaunchConfiguration('nav2'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(lab / 'launch/view.launch.py')),
            launch_arguments={'enabled': LaunchConfiguration('rviz'), 'frame': 'map'}.items()),
    ]


def generate_launch_description():
    default_map = str(Path(get_package_share_directory('turtlebot4_lab')) / 'maps/map.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map, description='Absolute path to a REAL saved map YAML'),
        DeclareLaunchArgument('nav2', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=navigation),
    ])
