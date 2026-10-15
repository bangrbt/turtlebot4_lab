from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('turtlebot4_lab05'))
    strings = dict(device='0', weights='yolo11s.pt',
                   camera_topic='/oakd/rgb/preview/image_raw', log_file='')
    numeric = dict(confidence=('0.30', float), person_confidence=('0.55', float),
                   plant_confidence=('0.15', float), image_size=('640', int),
                   inference_rate=('3.0', float), nms_iou=('0.45', float),
                   confirmation_frames=('2', int), confirmation_iou=('0.25', float))
    declarations = [DeclareLaunchArgument(k, default_value=v) for k,v in strings.items()]
    declarations += [DeclareLaunchArgument(k, default_value=v[0]) for k,v in numeric.items()]
    declarations += [DeclareLaunchArgument('show_window', default_value='true')]
    parameters = {k: ParameterValue(LaunchConfiguration(k), value_type=str) for k in strings}
    parameters.update({k: ParameterValue(LaunchConfiguration(k), value_type=v[1]) for k,v in numeric.items()})
    parameters['show_window'] = ParameterValue(LaunchConfiguration('show_window'), value_type=bool)
    return LaunchDescription(declarations + [Node(package='turtlebot4_lab05', executable='yolo_node',
        output='screen', parameters=[str(share/'config/yolo.yaml'), parameters])])

