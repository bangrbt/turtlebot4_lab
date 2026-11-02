"""Explicit AMCL initialization. Never sends navigation or velocity commands."""
import json
import hashlib
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
import yaml
from .configuration import quaternion_from_yaw


def main(args=None):
    rclpy.init(args=args)
    node = Node('lab05_initialize_pose')
    share = Path(get_package_share_directory('turtlebot4_lab05'))
    node.declare_parameter('config_file', str(share/'config/initial_pose.yaml'))
    node.declare_parameter('source', 'configured')
    node.declare_parameter('map_file', str(share/'maps/map.yaml'))
    node.declare_parameter('output_file', '')
    node.declare_parameter('timeout', 30.0)
    data = {}
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(Odometry, '/sim_ground_truth_pose', lambda m: data.update(truth=m), qos_profile_sensor_data),
                     node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: data.update(amcl=m), tl)]
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    client = node.create_client(Empty, '/request_nomotion_update')
    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=0.05)
    def pose_values(p):
        q = p.orientation
        return dict(x=p.position.x, y=p.position.y, yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    try:
        deadline = time.monotonic()+node.get_parameter('timeout').value
        mode = node.get_parameter('source').value
        settings = yaml.safe_load(Path(node.get_parameter('config_file').value).expanduser().read_text())
        map_path = Path(node.get_parameter('map_file').value).expanduser().resolve()
        map_metadata = yaml.safe_load(map_path.read_text())
        digest = hashlib.sha256((map_path.parent/map_metadata['image']).read_bytes()).hexdigest()
        if settings.get('map_image_sha256') != digest:
            raise ValueError('Initial pose alignment does not belong to this map image')
        while publisher.get_subscription_count() == 0 or (mode == 'sim_ground_truth' and 'truth' not in data):
            if time.monotonic()>deadline:
                raise RuntimeError('AMCL subscriber or simulated ground truth missing')
            spin(0.2)
        if mode == 'configured':
            config = yaml.safe_load(Path(node.get_parameter('config_file').value).expanduser().read_text())
            if config.get('validated') is not True or config.get('frame_id') != 'map':
                raise ValueError('Initial pose not verified in map frame')
            target = config['pose']
        elif mode == 'sim_ground_truth':
            # Only explicitly selected in this measured maze: map/world alignment must be confirmed.
            config = yaml.safe_load(Path(node.get_parameter('config_file').value).expanduser().read_text())
            if config.get('world_map_alignment_verified') is not True:
                raise ValueError('World/map alignment not verified; use RViz instead')
            target = pose_values(data['truth'].pose.pose)
        else:
            raise ValueError('source must be configured or sim_ground_truth')
        if not all(math.isfinite(float(target[k])) for k in ['x', 'y', 'yaw']):
            raise ValueError('Invalid pose')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(target['x']), float(target['y'])
        q = quaternion_from_yaw(target['yaw'])
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.01
        publisher.publish(msg)
        spin(2)
        for _ in range(3):
            if client.wait_for_service(timeout_sec=1):
                future = client.call_async(Empty.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2)
                spin(0.5)
        if 'amcl' not in data:
            raise RuntimeError('AMCL did not publish pose')
        transform = None
        for _ in range(20):
            try:
                spin(0.2)
                transform = buffer.lookup_transform('map', 'base_link', Time())
                break
            except Exception:
                continue
        if transform is None:
            transform = buffer.lookup_transform('map', 'base_link', Time())
        result = dict(source=mode, initial_pose=target, amcl_pose=pose_values(data['amcl'].pose.pose),
                      map_base_translation=[transform.transform.translation.x, transform.transform.translation.y],
                      note='Initialization only, before mission; no velocity or action goal publisher')
        print(json.dumps(result, indent=2), flush=True)
        output = node.get_parameter('output_file').value
        if output:
            path = Path(output).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(result, indent=2)+'\n')
        return 0
    except (KeyboardInterrupt, Exception) as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
