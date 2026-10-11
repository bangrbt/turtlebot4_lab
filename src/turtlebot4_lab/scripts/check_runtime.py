#!/usr/bin/env python3
"""Read live topics/TF/lifecycle. Exit nonzero for missing evidence; no motion."""
import argparse
import json
import math
import os
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage', choices=['simulation', 'mapping', 'localization', 'navigation'], default='simulation')
    p.add_argument('--seconds', type=float, default=12)
    p.add_argument('--output', type=Path)
    args = p.parse_args()
    if args.seconds < 2:
        p.error('--seconds must be >= 2')
    rclpy.init()
    node = rclpy.create_node('turtlebot4_lab_check')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples = {}
    def receive(key):
        def callback(msg):
            stamp = msg.clock if key == 'clock' else msg.header.stamp
            now = stamp.sec + stamp.nanosec / 1e9
            record = samples.setdefault(key, {'count': 0, 'first_stamp': now})
            record.update(count=record['count'] + 1, last_stamp=now)
            if key == 'scan':
                record.update(frame=msg.header.frame_id, rays=len(msg.ranges),
                    finite_rays=sum(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges))
            if key == 'odom':
                record.update(frame=msg.header.frame_id, child_frame=msg.child_frame_id,
                              position=[msg.pose.pose.position.x, msg.pose.pose.position.y])
            if key == 'map':
                record.update(frame=msg.header.frame_id, width=msg.info.width, height=msg.info.height,
                              known_cells=sum(v >= 0 for v in msg.data), resolution=msg.info.resolution)
            if key == 'amcl_pose':
                record.update(frame=msg.header.frame_id,
                              position=[msg.pose.pose.position.x, msg.pose.pose.position.y])
        return callback
    subs = [node.create_subscription(Clock, '/clock', receive('clock'), qos_profile_sensor_data),
            node.create_subscription(LaserScan, '/scan', receive('scan'), qos_profile_sensor_data),
            node.create_subscription(Odometry, '/odom', receive('odom'), qos_profile_sensor_data)]
    if args.stage != 'simulation':
        subs.append(node.create_subscription(OccupancyGrid, '/map', receive('map'),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    if args.stage in ['localization', 'navigation']:
        subs.append(node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                    receive('amcl_pose'), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    errors = []
    for key in ['clock', 'scan', 'odom']:
        sample = samples.get(key, {})
        if sample.get('count', 0) < 2 or sample.get('last_stamp', 0) <= sample.get('first_stamp', 0):
            errors.append(f'{key}: missing data or timestamps do not advance; check Play/bridge/QoS')
    if samples.get('scan', {}).get('finite_rays', 0) == 0:
        errors.append('scan: no finite returns within sensor range')
    if args.stage != 'simulation' and samples.get('map', {}).get('known_cells', 0) == 0:
        errors.append('map: no observed known cells')
    names = node.get_node_names()
    if args.stage == 'mapping':
        if 'slam_toolbox' not in names:
            errors.append('SLAM node missing')
        if 'amcl' in names or 'map_server' in names:
            errors.append('AMCL/map server running concurrently with mapping')
    if args.stage in ['localization', 'navigation']:
        if 'slam_toolbox' in names:
            errors.append('SLAM is still running: stop it before AMCL')
        if not samples.get('amcl_pose'):
            errors.append('AMCL pose missing: set 2D Pose Estimate and wait for laser update')
    transforms = {}
    base = samples.get('odom', {}).get('child_frame', 'base_link')
    pairs = [('odom', base), (base, samples.get('scan', {}).get('frame', 'rplidar_link'))]
    if args.stage != 'simulation':
        pairs.append(('map', 'odom'))
    for parent, child in pairs:
        try:
            tf = buffer.lookup_transform(parent, child, Time())
            transforms[f'{parent} <- {child}'] = {'translation': [tf.transform.translation.x,
                tf.transform.translation.y, tf.transform.translation.z],
                'stamp': tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9}
        except TransformException as exc:
            errors.append(f'TF {parent} <- {child}: {exc}')
    lifecycle = {}
    required = ['amcl', 'map_server'] if args.stage in ['localization','navigation'] else []
    if args.stage == 'navigation':
        required += ['planner_server','controller_server','bt_navigator','behavior_server',
                     'smoother_server','velocity_smoother','global_costmap/global_costmap',
                     'local_costmap/local_costmap']
    for name in required:
        client = node.create_client(GetState, '/' + name + '/get_state')
        if not client.wait_for_service(timeout_sec=1):
            errors.append(f'{name}: lifecycle service missing')
            continue
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2)
        if future.done() and future.result() is not None:
            state = future.result().current_state
            lifecycle[name] = {'id': state.id, 'label': state.label}
            if state.id != 3:
                errors.append(f'{name}: not active ({state.label})')
        else:
            errors.append(f'{name}: lifecycle query timed out')
    action_ready = None
    if args.stage == 'navigation':
        action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        action_ready = action.wait_for_server(timeout_sec=2)
        if not action_ready:
            errors.append('navigate_to_pose action server not ready')
        for topic in ['/global_costmap/costmap','/local_costmap/costmap']:
            if node.count_publishers(topic) == 0:
                errors.append(topic + ': no publisher')
    endpoints = {}
    for topic in ['/cmd_vel','/diffdrive_controller/cmd_vel_unstamped','/odom','/scan']:
        endpoints[topic] = {kind: [{'node':i.node_namespace.rstrip('/')+'/'+i.node_name,
                                  'type':i.topic_type} for i in method(topic)]
            for kind, method in [('publishers',node.get_publishers_info_by_topic),
                                 ('subscribers',node.get_subscriptions_info_by_topic)]}
    if not endpoints['/cmd_vel']['subscribers']:
        errors.append('/cmd_vel has no subscribers; do not send motion')
    result = {'stage':args.stage, 'ros_domain_id':os.environ.get('ROS_DOMAIN_ID','0'),
              'samples':samples, 'tf':transforms, 'nodes':sorted(names),
              'endpoints':endpoints, 'lifecycle':lifecycle, 'navigate_action_ready':action_ready,
              'errors':errors, 'passed':not errors,
              'scope':'Topic/TF readiness only; GUI visibility, map quality and navigation outcomes require separate observation.'}
    data = json.dumps(result,ensure_ascii=False,indent=2)+'\n'
    print(data)
    if args.output:
        args.output.expanduser().parent.mkdir(parents=True,exist_ok=True)
        args.output.expanduser().write_text(data)
    node.destroy_node()
    rclpy.shutdown()
    return 1 if errors else 0


if __name__ == '__main__':
    raise SystemExit(main())
