"""Real ROS adapter checks on an isolated domain; synthetic sensors, no simulator."""
import json
import time
from unittest.mock import Mock
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from turtlebot4_gesture_control.robot_behavior_node import RobotBehaviorNode
from turtlebot4_gesture_control.gestures import Observation
from turtlebot4_gesture_control.motion import Pose
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock


@pytest.fixture
def robot():
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=RobotBehaviorNode();node.publish_velocity=Mock()
    yield node
    node.destroy_node();rclpy.shutdown()


def arm(node):
    node.arena_ready=True;node.arena_seen=time.monotonic()
    node.clearance=4.;node.hazards=set();node.pose=Pose(0,0,0,1.)
    node.odom_seen=node.scan_seen=node.clock_seen=node.hazard_seen=time.monotonic()
    seq=0
    for label in ('Pointing_Up','Thumb_Up'):
        start=time.monotonic()
        while time.monotonic()-start<.7:
            now=time.monotonic();node.arena_seen=node.odom_seen=node.scan_seen=node.clock_seen=node.hazard_seen=now
            seq+=1
            o=Observation(label,.99,label,.99,True,True,True,'isolated-test',seq,time.time(),'adapter_test')
            node.on_gesture(String(data=o.dumps()));time.sleep(.05)
    assert node.machine.state=='RUNNING'
    return seq


def test_steady_timer_stops_when_ros_time_never_advances(robot):
    arm(robot)
    assert robot.get_clock().now().nanoseconds==0
    end=time.monotonic()+1.2
    while time.monotonic()<end and robot.machine.state=='RUNNING':rclpy.spin_once(robot,timeout_sec=.05)
    assert robot.machine.state=='IDLE'
    assert robot.publish_velocity.call_args.args==(0.,0.)
    assert robot.get_clock().now().nanoseconds==0


def test_malformed_and_stale_wire_messages_disarm_immediately(robot):
    arm(robot);robot.on_gesture(String(data='{"label": "Thumb_Up"}'))
    assert robot.machine.state=='IDLE' and robot.machine.reason=='invalid_or_stale_gesture'
    assert robot.publish_velocity.call_args.args==(0.,0.)


def test_frozen_clock_packets_do_not_refresh_clock_watchdog(robot):
    message=Clock();message.clock.sec=1
    robot.on_clock(message);first=robot.clock_seen
    robot.on_clock(message)
    assert robot.clock_seen==first


@pytest.mark.parametrize('confidence',[.50,.55,.64])
def test_fist_callback_disarms_and_publishes_zero_before_any_timer(robot,confidence):
    seq=arm(robot)
    robot.publish_velocity.reset_mock()
    o=Observation('Closed_Fist',confidence,'Closed_Fist',confidence,True,False,True,
                  'isolated-test',seq+1,time.time(),'cancel_priority')
    robot.on_gesture(String(data=o.dumps()))
    assert robot.machine.state=='IDLE' and robot.machine.selected is None and robot.machine.executor is None
    assert robot.machine.reason=='closed_fist_cancel'
    robot.publish_velocity.assert_called_once_with(0.,0.)



def test_landmark_fist_callback_with_no_class_probability_cancels_immediately(robot):
    seq=arm(robot)
    robot.publish_velocity.reset_mock()
    o=Observation('Closed_Fist',0.,'None',.91,True,True,True,
                  'isolated-test',seq+1,time.time(),'fist_landmarks_cancel')
    robot.on_gesture(String(data=o.dumps()))
    assert robot.machine.state=='IDLE' and robot.machine.executor is None
    assert robot.machine.reason=='closed_fist_cancel'
    robot.publish_velocity.assert_called_once_with(0.,0.)
