#!/usr/bin/env python3
"""Bounded simulator-only /cmd_vel probe; never run alongside gesture behavior."""
import argparse,json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from turtlebot4_gesture_control.safety import scan_clearance


def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path);a=p.parse_args()
 rclpy.init();n=Node('gesture_velocity_probe');d={};pub=n.create_publisher(Twist,'/cmd_vel',10)
 subs=[n.create_subscription(Odometry,'/odom',lambda m:d.update(odom=m,odom_at=time.monotonic()),qos_profile_sensor_data),
       n.create_subscription(LaserScan,'/scan',lambda m:d.update(scan=m,scan_at=time.monotonic()),qos_profile_sensor_data),
       n.create_subscription(Bool,'/gesture/arena_ready',lambda m:d.update(ready=m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
 def send(v):
  msg=Twist();msg.linear.x=v;pub.publish(msg)
 try:
  deadline=time.monotonic()+12
  while not all(k in d for k in ['odom','scan','ready']) and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
  if 'robot_behavior_node' in n.get_node_names():raise RuntimeError('Stop robot_behavior_node before probing /cmd_vel')
  if not d.get('ready') or 'odom' not in d or 'scan' not in d:raise RuntimeError('Prepared gesture_arena not ready')
  start=d['odom'].pose.pose.position;xy=(start.x,start.y);end=time.monotonic()+.8
  while time.monotonic()<end:
   scan=d['scan'];clear=scan_clearance(scan.ranges,scan.range_min,scan.range_max,scan.angle_increment)
   if clear is None or clear<1.4 or time.monotonic()-d['scan_at']>1 or time.monotonic()-d['odom_at']>.75:raise RuntimeError('Sensor/clearance guard failed')
   send(.10);rclpy.spin_once(n,timeout_sec=.05)
  stop_until=time.monotonic()+1.
  while time.monotonic()<stop_until:send(0.);rclpy.spin_once(n,timeout_sec=.05)
  finish=d['odom'].pose.pose.position;v=d['odom'].twist.twist
  distance=math.hypot(finish.x-xy[0],finish.y-xy[1])
  result=dict(displacement_m=distance,final_linear=v.linear.x,final_angular=v.angular.z,passed=distance>.005 and abs(v.linear.x)<.02 and abs(v.angular.z)<.03)
  print(json.dumps(result,indent=2))
  if a.output:a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
  return 0 if result['passed'] else 1
 finally:
  for _ in range(5):send(0.);rclpy.spin_once(n,timeout_sec=.02)
  n.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
