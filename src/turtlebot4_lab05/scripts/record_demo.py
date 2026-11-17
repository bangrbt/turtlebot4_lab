#!/usr/bin/env python3
"""Read-only runtime evidence: actual RGB/YOLO, navigation, ground truth and hazards."""
import argparse
import json
import math
import signal
import time
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry, Path as NavPath
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from irobot_create_msgs.msg import HazardDetectionVector
from cv_bridge import CvBridge


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=1200)
    args = parser.parse_args()
    output = args.output_directory.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node('lab05_demo_observer')
    state = dict(odom=None, truth=None, amcl=None, status=None, yolo=None, hazards=[])
    raw_headers = {}
    annotated = {}
    pending = {}
    detections = []
    navigation = []
    samples = []
    saved = set()
    counts = dict(raw=0, annotated=0, header_matches=0, inference_while_moving=0)
    bridge = CvBridge()
    stopped = False
    finished_at = None
    last_sample = 0.0
    trace = output/'demo_trace.jsonl'
    def log(event, **values):
        record = {'event':event, 'wall_time_unix':time.time(), **values}
        with trace.open('a') as stream:
            stream.write(json.dumps(record)+'\n')
    def key(msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id)
    def pose(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return dict(x=p.x, y=p.y, yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def raw(msg):
        raw_headers[key(msg)] = (msg.width, msg.height)
        counts['raw'] += 1
        if len(raw_headers)>2000:
            raw_headers.pop(next(iter(raw_headers)))
    def image(msg):
        counts['annotated'] += 1
        if key(msg) in raw_headers:
            counts['header_matches'] += 1
        annotated[key(msg)] = msg
        if len(annotated)>20:
            annotated.pop(next(iter(annotated)))
        save_pending()
    def save_pending():
        for stamp, record in list(pending.items()):
            if stamp not in annotated:
                continue
            labels = {d['label'] for d in record['detections']}
            for label in labels-saved:
                frame = bridge.imgmsg_to_cv2(annotated[stamp], 'bgr8')
                path = output/f'detection_{label}_annotated.png'
                cv2.imwrite(str(path), frame)
                saved.add(label)
                log('detection_image', label=label, file=path.name, header=stamp)
            pending.pop(stamp)
    def detection(msg):
        record = json.loads(msg.data)
        moving = bool(state['odom'] and (abs(state['odom']['linear_x'])>.02 or abs(state['odom']['angular_z'])>.05))
        record.update(wall_time_unix=time.time(), robot_moving=moving)
        detections.append(record)
        counts['inference_while_moving'] += int(moving)
        if record['detections']:
            stamp=(record['stamp']['sec'],record['stamp']['nanosec'],record['frame_id'])
            pending[stamp] = record
            if len(pending)>30:
                pending.pop(next(iter(pending)))
        save_pending()
        log('inference', **record)
    def nav(msg):
        nonlocal finished_at
        record = json.loads(msg.data)
        navigation.append(record)
        state['status'] = record
        log('navigation', record=record)
        if record['event'] in ['mission_completed', 'mission_failed', 'mission_interrupted']:
            finished_at = time.monotonic()
    def odom(msg):
        state['odom'] = dict(pose=pose(msg),linear_x=msg.twist.twist.linear.x,angular_z=msg.twist.twist.angular.z)
    def truth(msg):
        state['truth'] = pose(msg)
    def amcl(msg):
        state['amcl'] = pose(msg)
    observed_hazard_types = set()
    def hazards(msg):
        items=[dict(type=m.type,frame=m.header.frame_id) for m in msg.detections]
        observed_hazard_types.update(h['type'] for h in items)
        if state['hazards'] != items:
            state['hazards'] = items
            log('hazard_change', detections=items)
    def plan(msg):
        if msg.poses:
            log('plan', points=[[p.pose.position.x,p.pose.position.y] for p in msg.poses])
    tl=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions=[node.create_subscription(Image,'/oakd/rgb/preview/image_raw',raw,qos_profile_sensor_data),
                   node.create_subscription(Image,'/yolo/annotated_image',image,qos_profile_sensor_data),
                   node.create_subscription(String,'/yolo/detections',detection,10),
                   node.create_subscription(String,'/yolo/status',lambda m:state.update(yolo=json.loads(m.data)),tl),
                   node.create_subscription(String,'/lab05/navigation_status',nav,tl),
                   node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data),
                   node.create_subscription(Odometry,'/sim_ground_truth_pose',truth,qos_profile_sensor_data),
                   node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',amcl,tl),
                   node.create_subscription(HazardDetectionVector,'/hazard_detection',hazards,qos_profile_sensor_data),
                   node.create_subscription(NavPath,'/plan',plan,10)]
    def stop(signum, frame):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop)
    signal.signal(signal.SIGTERM,stop)
    deadline=time.monotonic()+args.seconds
    try:
        while rclpy.ok() and not stopped and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if time.monotonic()-last_sample>=.5:
                last_sample=time.monotonic()
                item=dict(wall_time_unix=time.time(),**state)
                samples.append(item)
                log('sample', **state)
            if finished_at and time.monotonic()-finished_at>5:
                break
    finally:
        results=[r for r in navigation if r['event']=='goal_result']
        confidence={}
        for r in detections:
            for d in r['detections']:
                confidence[d['label']]=max(confidence.get(d['label'],0),d['confidence'])
        hazard_types=sorted(observed_hazard_types)
        summary=dict(counts=counts,goal_results=results,mission_completed=any(r['event']=='mission_completed' for r in navigation),
                     max_confidence=confidence,actual_model_devices=sorted({r.get('actual_model_device','unknown') for r in detections}),
                     hazard_types_observed=hazard_types,bump_cliff_stall_wheel_drop_observed=bool(set(hazard_types)&{1,2,3,4}),
                     final_state=state,scope='Read-only live simulation observations, no velocity or action publisher; no general collision-avoidance guarantee.')
        (output/'demo_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(summary,indent=2),flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if summary['mission_completed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
