"""Record real RViz Publish Point locations; yaw is a separate choice."""
import hashlib
import math
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from ament_index_python.packages import get_package_share_directory
from .configuration import point_is_free


class CaptureWaypoints(Node):
    def __init__(self):
        super().__init__('lab05_capture_waypoints')
        share = Path(get_package_share_directory('turtlebot4_lab05'))
        self.declare_parameter('output_file', 'waypoints_recorded.yaml')
        self.declare_parameter('map_file', str(share/'maps/map.yaml'))
        self.declare_parameter('yaw_mode', 'amcl')
        self.declare_parameter('fixed_yaw', 0.0)
        self.path = Path(self.get_parameter('output_file').value).expanduser().resolve()
        if self.path.exists():
            raise FileExistsError('Refusing overwrite: '+str(self.path))
        self.map_path = Path(self.get_parameter('map_file').value).expanduser().resolve()
        meta = yaml.safe_load(self.map_path.read_text())
        self.digest = hashlib.sha256((self.map_path.parent/meta['image']).read_bytes()).hexdigest()
        self.pose = None
        self.grid = None
        self.goals = []
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda msg: setattr(self, 'pose', msg), tl)
        self.create_subscription(OccupancyGrid, '/map', lambda msg: setattr(self, 'grid', msg), tl)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked, 10)
        self.get_logger().info('RViz Publish Point -> /clicked_point supplies x/y only, no yaw. Configure yaw_mode=amcl or fixed.')

    def clicked(self, msg):
        try:
            if msg.header.frame_id != 'map' or self.grid is None:
                raise ValueError('Need /map and clicked_point in map frame')
            if not point_is_free(self.grid, msg.point.x, msg.point.y):
                raise ValueError('Point is not free with 0.25 m margin')
            mode = self.get_parameter('yaw_mode').value
            if mode == 'amcl':
                if self.pose is None or self.pose.header.frame_id != 'map':
                    raise ValueError('AMCL pose absent')
                q = self.pose.pose.pose.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            elif mode == 'fixed':
                yaw = float(self.get_parameter('fixed_yaw').value)
            else:
                raise ValueError('yaw_mode must be amcl or fixed')
            if not math.isfinite(yaw):
                raise ValueError('Nonfinite yaw')
            self.goals.append(dict(name=f'Goal_{len(self.goals)+1}', x=float(msg.point.x), y=float(msg.point.y),
                                   yaw=yaw, source=f'RViz /clicked_point stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}; yaw_mode={mode}'))
            data = dict(validated=len(self.goals)>=3, frame_id='map', map_image_sha256=self.digest, goals=self.goals)
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix('.yaml.tmp')
            temporary.write_text(yaml.safe_dump(data, sort_keys=False))
            temporary.replace(self.path)
            self.get_logger().info(f'Saved {len(self.goals)} measured goals to {self.path}; validated={data["validated"]}')
        except (ValueError, OSError) as exc:
            self.get_logger().error(str(exc))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CaptureWaypoints()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'LOI WAYPOINT: {exc}', flush=True)
        return 1
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
