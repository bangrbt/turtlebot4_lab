#!/usr/bin/env python3
"""Short guarded simulation movement on the verified /cmd_vel path; always publish stop."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path)
    args = p.parse_args()
    rclpy.init()
    node = rclpy.create_node('turtlebot4_lab_motion_probe')
    latest = {}
    def remember(key):
        return lambda msg: latest.update({key:(msg,time.monotonic())})
    subscriptions = [node.create_subscription(LaserScan,'/scan',remember('scan'),qos_profile_sensor_data),
        node.create_subscription(Odometry,'/odom',remember('odom'),qos_profile_sensor_data),
        node.create_subscription(Clock,'/clock',remember('clock'),qos_profile_sensor_data)]
    pub = node.create_publisher(Twist,'/cmd_vel',10)
    report = {'passed':False, 'topic':'/cmd_vel','speed_m_s':0.08,'duration_wall_s':2.0}
    def spin_for(duration):
        end = time.monotonic()+duration
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=0.05)
    def clearance():
        if 'scan' not in latest or time.monotonic()-latest['scan'][1]>1:
            raise RuntimeError('Scan missing/stale')
        scan = latest['scan'][0]
        front = [v for i,v in enumerate(scan.ranges) if abs(math.atan2(
            math.sin(scan.angle_min+i*scan.angle_increment),
            math.cos(scan.angle_min+i*scan.angle_increment)))<math.radians(25)]
        if not front or any(math.isnan(v) or v<scan.range_min for v in front):
            raise RuntimeError('Invalid front scan')
        return min(front)
    try:
        spin_for(3)
        names = node.get_node_names()
        if 'clock_bridge' not in names or 'pose_bridge' not in names:
            raise RuntimeError('Not a verified TurtleBot simulator; movement refused')
        if 'controller_server' in names or 'bt_navigator' in names:
            raise RuntimeError('Nav2 is running; stop it before this manual movement test')
        subscribers = node.get_subscriptions_info_by_topic('/cmd_vel')
        report['subscribers'] = [i.node_name for i in subscribers]
        if 'motion_control' not in report['subscribers']:
            raise RuntimeError('Expected Create 3 motion_control subscriber missing from /cmd_vel')
        if 'odom' not in latest or 'clock' not in latest:
            raise RuntimeError('Odom/clock missing')
        first_clock = latest['clock'][0].clock
        spin_for(0.3)
        current_clock = latest['clock'][0].clock
        if (current_clock.sec,current_clock.nanosec)<=(first_clock.sec,first_clock.nanosec):
            raise RuntimeError('Simulation paused; press Play')
        if clearance()<0.8:
            raise RuntimeError('Obstacle within 0.8 m in front; motion refused')
        start = latest['odom'][0].pose.pose.position
        report['start_xy'] = [start.x,start.y]
        command = Twist()
        command.linear.x = 0.08
        end = time.monotonic()+2
        while time.monotonic()<end:
            if clearance()<0.6 or time.monotonic()-latest['odom'][1]>1:
                raise RuntimeError('Obstacle or stale odom during movement; stopping')
            pub.publish(command)
            spin_for(0.05)
        report['command_completed'] = True
    except (RuntimeError,KeyboardInterrupt) as exc:
        report['error'] = str(exc) or 'Interrupted'
    finally:
        for _ in range(10):
            pub.publish(Twist())
            spin_for(0.05)
        spin_for(0.5)
        if 'odom' in latest and 'start_xy' in report:
            end_pose = latest['odom'][0].pose.pose.position
            velocity = latest['odom'][0].twist.twist
            distance = math.hypot(end_pose.x-report['start_xy'][0],end_pose.y-report['start_xy'][1])
            report.update(end_xy=[end_pose.x,end_pose.y],distance_m=distance,
                          final_linear_x=velocity.linear.x,final_angular_z=velocity.angular.z)
            report['passed'] = bool(report.get('command_completed') and distance>0.03 and
                                    abs(velocity.linear.x)<0.01 and abs(velocity.angular.z)<0.02)
            if not report['passed'] and 'error' not in report:
                report['error'] = 'Motion/stop not proven; inspect dock status, safety hazards and controller'
        text = json.dumps(report,indent=2)+'\n'
        print(text)
        if args.output:
            args.output.expanduser().parent.mkdir(parents=True,exist_ok=True)
            args.output.expanduser().write_text(text)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
