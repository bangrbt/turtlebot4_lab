"""Bounded asynchronous NavigateToPose mission. Never publishes velocity."""
import json
import math
import signal
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as RclpyClock, ClockType
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from ament_index_python.packages import get_package_share_directory
from .configuration import load_waypoints, quaternion_from_yaw, point_is_free


class SequentialNavigation(Node):
    def __init__(self):
        super().__init__('lab05_sequential_navigation')
        share = Path(get_package_share_directory('turtlebot4_lab05'))
        defaults = dict(waypoint_file=str(share/'config/waypoints.yaml'),
                        map_file=str(share/'maps/map.yaml'), localization_confirmed=False,
                        readiness_timeout=180.0, goal_timeout=300.0, retries=1,
                        retry_delay=2.0, dwell_seconds=6.0, require_yolo=True,
                        yolo_status_topic='/yolo/status', log_file='', free_margin=0.25)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.params = {key: self.get_parameter(key).value for key in defaults}
        for key in ['readiness_timeout', 'goal_timeout', 'retry_delay']:
            if self.params[key] <= 0:
                raise ValueError(f'{key} must be positive')
        if self.params['retries'] < 0 or self.params['dwell_seconds'] < 0:
            raise ValueError('retries and dwell_seconds cannot be negative')
        self.config = load_waypoints(self.params['waypoint_file'], self.params['map_file'])
        self.goals = self.config['goals']
        self.index = 0
        self.attempt = 0
        self.state = 'WAIT_READY'
        self.exit_code = None
        self.started = time.monotonic()
        self.last_graph = 0.0
        self.last_progress = 0.0
        self.last_wait_log = 0.0
        self.goal_handle = None
        self.result_future = None
        self.send_future = None
        self.cancel_reason = None
        self.cancel_started = None
        self.pending_cancel = None
        self.pose = None
        self.grid = None
        self.clock_stamp = None
        self.clock_advanced = 0.0
        self.scan_received = 0.0
        self.yolo = None
        self.yolo_received = 0.0
        self.yolo_interrupted = False
        self.lifecycle = {}
        self.lifecycle_pending = {}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.input_subscriptions = [
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_pose, tl),
            self.create_subscription(OccupancyGrid, '/map', self.on_map, tl),
            self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data),
            self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data),
            self.create_subscription(String, self.params['yolo_status_topic'], self.on_yolo, tl)]
        self.publisher = self.create_publisher(String, '/lab05/navigation_status', tl)
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        names = ['amcl', 'map_server', 'planner_server', 'controller_server', 'bt_navigator',
                 'behavior_server', 'velocity_smoother', 'global_costmap/global_costmap',
                 'local_costmap/local_costmap']
        self.lifecycle_clients = {name: self.create_client(GetState, '/'+name+'/get_state') for name in names}
        self.wall_clock = RclpyClock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.1, self.tick, clock=self.wall_clock)
        self.log('mission_loaded', waypoint_file=self.params['waypoint_file'], goals=self.goals,
                 note='Only Nav2 actions; no cmd_vel publisher; localization_confirmed required')

    def log(self, event, **values):
        record = dict(event=event, wall_time_unix=time.time(), sim_time_ns=self.get_clock().now().nanoseconds,
                      state=self.state, goal_index=self.index+1, total=len(self.goals), **values)
        line = json.dumps(record, ensure_ascii=False)
        self.get_logger().info(line)
        if self.params['log_file']:
            path = Path(self.params['log_file']).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('a') as stream:
                stream.write(line+'\n')
        self.publisher.publish(String(data=line))

    def on_pose(self, msg):
        self.pose = msg

    def on_map(self, msg):
        self.grid = msg

    def on_scan(self, msg):
        self.scan_received = time.monotonic()

    def on_clock(self, msg):
        stamp = (msg.clock.sec, msg.clock.nanosec)
        if stamp != self.clock_stamp:
            self.clock_advanced = time.monotonic()
        self.clock_stamp = stamp

    def on_yolo(self, msg):
        try:
            self.yolo = json.loads(msg.data)
            self.yolo_received = time.monotonic()
        except (ValueError, TypeError):
            pass

    def refresh_lifecycle(self):
        now = time.monotonic()
        for name, (future, sent) in list(self.lifecycle_pending.items()):
            if future.done():
                try:
                    self.lifecycle[name] = (future.result().current_state.id, now)
                except Exception:
                    self.lifecycle.pop(name, None)
                self.lifecycle_pending.pop(name)
            elif now-sent > 3:
                self.lifecycle_pending.pop(name)
                self.lifecycle.pop(name, None)
        if now-self.last_graph < 2:
            return
        self.last_graph = now
        for name, client in self.lifecycle_clients.items():
            if name not in self.lifecycle_pending and client.service_is_ready():
                self.lifecycle_pending[name] = (client.call_async(GetState.Request()), now)

    def ready_reason(self):
        now = time.monotonic()
        if not self.params['localization_confirmed']:
            return 'localization_confirmed=false: kiem tra scan/map va initial pose truoc'
        if now-self.clock_advanced > 2 or now-self.scan_received > 2:
            return 'Clock/scan missing, stale or simulation paused'
        if self.pose is None or self.grid is None:
            return 'Waiting for AMCL pose and map (Transient Local QoS)'
        covariance = self.pose.pose.covariance
        if max(covariance[0], covariance[7]) > 0.3 or covariance[35] > 0.2:
            return 'AMCL covariance too large; correct initial pose'
        try:
            transform = self.buffer.lookup_transform('map', 'base_link', Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec/1e9
            if self.get_clock().now().nanoseconds/1e9-stamp > 2:
                return 'map->base_link TF stale'
        except Exception:
            return 'Waiting for map->odom->base_link TF'
        for name in self.lifecycle_clients:
            state, observed = self.lifecycle.get(name, (None, 0))
            if state != 3 or now-observed > 8:
                return f'{name} not confirmed active'
        if not self.action.server_is_ready():
            return 'NavigateToPose server not ready'
        names = self.get_node_names()
        if 'slam_toolbox' in names or any('teleop' in name for name in names):
            return 'Stop SLAM and all teleop nodes before autonomous mission'
        if self.params['require_yolo'] and (not self.yolo or self.yolo.get('state') != 'ready'
                                          or now-self.yolo_received > 4):
            return 'Waiting for YOLO successful camera inference'
        for goal in self.goals:
            if not point_is_free(self.grid, goal['x'], goal['y'], self.params['free_margin']):
                return f'Goal {goal["name"]} is not in known free space with margin'
        return None

    def send_goal(self):
        self.attempt += 1
        target = self.goals[self.index]
        pose = PoseStamped()
        pose.header.frame_id = self.config['frame_id']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(target['x'])
        pose.pose.position.y = float(target['y'])
        q = quaternion_from_yaw(target['yaw'])
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        request = NavigateToPose.Goal()
        request.pose = pose
        self.state = 'SENDING'
        self.goal_started = time.monotonic()
        self.cancel_reason = None
        self.cancel_started = None
        self.log('goal_sent', name=target['name'], target=target, attempt=self.attempt)
        self.send_future = self.action.send_goal_async(request, feedback_callback=self.feedback)
        self.send_future.add_done_callback(self.accepted)

    def accepted(self, future):
        try:
            self.goal_handle = future.result()
            if self.goal_handle is None or not self.goal_handle.accepted:
                self.goal_handle = None
                self.failed('Goal rejected')
                return
            self.result_future = self.goal_handle.get_result_async()
            self.result_future.add_done_callback(self.result)
            self.state = 'RUNNING'
            self.log('goal_accepted', goal_uuid=bytes(self.goal_handle.goal_id.uuid).hex())
            if self.cancel_reason:
                self.request_cancel(self.cancel_reason)
        except Exception as exc:
            self.failed(f'Goal request error: {exc}')

    def feedback(self, msg):
        now = time.monotonic()
        if now-self.last_progress >= 2:
            self.last_progress = now
            feedback = msg.feedback
            self.log('progress', name=self.goals[self.index]['name'],
                     distance_remaining=float(feedback.distance_remaining),
                     recoveries=int(feedback.number_of_recoveries))

    def result(self, future):
        try:
            response = future.result()
            status = response.status
        except Exception as exc:
            self.failed(f'Result error: {exc}')
            return
        uid = bytes(self.goal_handle.goal_id.uuid).hex() if self.goal_handle else None
        self.goal_handle = None
        self.result_future = None
        self.log('goal_result', name=self.goals[self.index]['name'], status=status, goal_uuid=uid,
                 cancel_reason=self.cancel_reason)
        if self.cancel_reason == 'interrupt':
            self.finish(130, 'mission_interrupted')
        elif self.cancel_reason:
            self.failed(self.cancel_reason)
        elif status == 4:
            self.index += 1
            self.attempt = 0
            self.state = 'DWELL'
            self.next_at = time.monotonic()+self.params['dwell_seconds']
            self.log('goal_succeeded', completed=self.index, dwell_seconds=self.params['dwell_seconds'])
        else:
            self.failed(f'Action terminal status {status}; only 4 is success')

    def failed(self, reason):
        self.goal_handle = None
        if self.cancel_reason == 'interrupt':
            self.finish(130, 'mission_interrupted')
        elif self.attempt <= self.params['retries']:
            self.state = 'RETRY'
            self.next_at = time.monotonic()+self.params['retry_delay']
            self.log('goal_retry_pending', reason=reason, attempt=self.attempt)
        else:
            self.finish(1, 'mission_failed', reason=reason, completed=self.index)

    def request_cancel(self, reason):
        if self.exit_code is not None:
            return
        if self.cancel_reason == 'interrupt':
            reason = 'interrupt'
        self.cancel_reason = reason
        if self.cancel_started is None:
            self.cancel_started = time.monotonic()
        if self.goal_handle and self.state != 'CANCELING':
            self.state = 'CANCELING'
            self.pending_cancel = self.goal_handle.cancel_goal_async()
            self.log('cancel_requested', reason=reason)
        elif self.state != 'SENDING' and self.goal_handle is None:
            self.finish(130 if reason == 'interrupt' else 1, 'mission_stopped', reason=reason)

    def finish(self, code, event, **values):
        self.state = 'DONE'
        self.exit_code = code
        self.log(event, yolo_interrupted=self.yolo_interrupted, **values)

    def tick(self):
        if self.exit_code is not None:
            return
        now = time.monotonic()
        if self.state == 'SENDING' and self.cancel_reason and now-self.cancel_started > 8:
            self.finish(130 if self.cancel_reason == 'interrupt' else 1, 'cancel_timeout',
                        reason='No goal acknowledgement; cannot confirm cancellation; no retry sent')
            return
        if self.state == 'WAIT_READY':
            self.refresh_lifecycle()
            reason = self.ready_reason()
            if reason is None:
                self.log('readiness_passed')
                self.send_goal()
            elif now-self.started > self.params['readiness_timeout']:
                self.finish(1, 'readiness_timeout', reason=reason)
            elif now-self.last_wait_log > 5:
                self.last_wait_log = now
                self.log('waiting', reason=reason)
        elif self.state in ['SENDING', 'RUNNING']:
            if now-self.goal_started > self.params['goal_timeout']:
                self.request_cancel('goal_timeout')
                # A pending send response must be resolved before another action may be sent.
                if self.state == 'SENDING' and now-self.goal_started > self.params['goal_timeout']+8:
                    self.finish(1, 'goal_acceptance_timeout', reason='Cannot prove server acceptance/cancel; no retry sent')
            elif now-self.clock_advanced > 4:
                self.request_cancel('simulation_clock_stalled')
            if self.params['require_yolo'] and (not self.yolo or self.yolo.get('state') != 'ready'
                                              or now-self.yolo_received > 5):
                if not self.yolo_interrupted:
                    self.log('yolo_interrupted', note='YOLO does not steer robot; Nav2 remains independent')
                self.yolo_interrupted = True
        elif self.state == 'CANCELING' and now-self.cancel_started > 8:
            self.finish(130 if self.cancel_reason == 'interrupt' else 1, 'cancel_timeout',
                        reason='No terminal result; mission stops without sending another goal')
        elif self.state in ['DWELL', 'RETRY'] and now >= self.next_at:
            if self.index == len(self.goals):
                self.finish(0, 'mission_completed', completed=self.index)
            else:
                self.refresh_lifecycle()
                reason = self.ready_reason()
                if reason:
                    if now-self.next_at > self.params['readiness_timeout']:
                        self.finish(1, 'readiness_timeout', reason=reason)
                    elif now-self.last_wait_log > 5:
                        self.last_wait_log = now
                        self.log('waiting_between_goals', reason=reason)
                else:
                    self.send_goal()


def main(args=None):
    rclpy.init(args=args)
    node = None
    interrupted = False
    def signal_handler(signum, frame):
        nonlocal interrupted
        interrupted = True
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    try:
        node = SequentialNavigation()
        while rclpy.ok() and node.exit_code is None:
            if interrupted:
                node.request_cancel('interrupt')
                interrupted = False
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.exit_code if node.exit_code is not None else 1
    except Exception as exc:
        if node:
            node.get_logger().error(str(exc))
        else:
            print(f'LOI NAVIGATION: {exc}', flush=True)
        return 1
    finally:
        if node:
            if node.goal_handle:
                node.request_cancel('interrupt')
                end = time.monotonic()+8
                while rclpy.ok() and node.goal_handle and time.monotonic()<end:
                    rclpy.spin_once(node, timeout_sec=0.1)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
