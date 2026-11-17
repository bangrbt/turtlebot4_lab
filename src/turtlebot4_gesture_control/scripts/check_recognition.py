#!/usr/bin/env python3
"""Passive per-gesture webcam check; never publishes commands or gesture messages."""
import argparse,collections,json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
 p=argparse.ArgumentParser();p.add_argument('--expected',required=True,choices=['None','Pointing_Up','Victory','Thumb_Up','Open_Palm','Closed_Fist','Wave']);p.add_argument('--seconds',type=float,default=8.);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if not 1<=a.seconds<=60:raise ValueError('Capture must last 1..60 seconds')
 rclpy.init();n=Node('gesture_passive_recognition_check');raw=collections.Counter();accepted=collections.Counter();total=visible=stable=0
 def cb(msg):
  nonlocal total,visible,stable
  d=json.loads(msg.data);total+=1
  if d['hand_visible']:visible+=1;raw[d['raw_label']]+=1
  if d['stable'] and d['valid_control']:accepted[d['label']]+=1;stable+=1
 sub=n.create_subscription(String,'/detected_gesture',cb,10)
 end=time.monotonic()+a.seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
 result=dict(expected=a.expected,observations=total,hand_visible=visible,raw_counts=dict(raw),accepted_counts=dict(accepted),stable_control_observations=stable,
             scope='User must actually hold the expected gesture. Raw counts and accepted controls are separate; no inferred ground truth and no motion commands.')
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
 return 0 if total else 1
if __name__=='__main__':raise SystemExit(main())
