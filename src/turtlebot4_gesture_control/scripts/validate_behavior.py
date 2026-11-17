#!/usr/bin/env python3
"""SIMULATION ONLY: synthetic gestures -> real behavior node -> Gazebo odometry."""
import argparse
import json
import math
import time
import uuid
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlebot4_gesture_control.gestures import Observation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('gesture_behavior_validation')
    data, path, commands, events = {}, [], [], []
    sequence, session, last_publish = 0, uuid.uuid4().hex, 0.
    publisher = node.create_publisher(String, '/detected_gesture', 10)

    def status(msg):
        value = json.loads(msg.data)
        data['status'] = value
        key = [value['state'], value['reason'], value['completions']]
        if not events or events[-1]['key'] != key:
            events.append(dict(key=key, status=value))

    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = (p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        data.update(pose=pose, speed=(msg.twist.twist.linear.x, msg.twist.twist.angular.z))
        path.append(pose)

    inputs = [
        node.create_subscription(String, '/gesture/behavior_status', status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(Odometry, '/odom', odom, qos_profile_sensor_data),
        node.create_subscription(Twist, '/cmd_vel', lambda m: commands.append((m.linear.x, m.angular.z)), 10)]

    def pump(label='None', duration=.1, until=None, reason='synthetic_simulation_test', confidence=.99):
        nonlocal sequence, last_publish
        end = time.monotonic()+duration
        while time.monotonic() < end:
            now = time.monotonic()
            if label is not None and now-last_publish >= .05:
                sequence += 1
                obs = Observation(label=label, confidence=confidence,
                    raw_label='Open_Palm' if label == 'Wave' else label, raw_confidence=confidence,
                    hand_visible=label != 'None', stable=label != 'None', valid_control=label != 'None',
                    session_id=session, sequence=sequence, captured_unix=time.time(), reason=reason)
                publisher.publish(String(data=obs.dumps()))
                last_publish = now
            rclpy.spin_once(node, timeout_sec=.01)
            if until and until():
                return True
        return False

    def expect(state, reason=None):
        value = data.get('status', {})
        assert value.get('state') == state, value
        if reason:
            assert value.get('reason') == reason, value

    def wait_stopped():
        # Gazebo may run below real time while webcam inference is active.
        # Check measured braking completion, not a fixed wall-time sleep.
        stopped = pump('None', 5., lambda: abs(data.get('speed',(1.,1.))[0]) < .02 and
                       abs(data.get('speed',(1.,1.))[1]) < .03)
        assert stopped, ('Robot did not finish braking', data.get('speed'))

    def start_circle():
        pump('Victory', .7)
        expect('SELECTED')
        pump('Thumb_Up', .65)
        expect('RUNNING')

    result = dict(scope='Synthetic gestures, real ROS behavior node and Gazebo. No webcam accuracy claim.', checks={}, behaviors=[])
    try:
        if 'gesture_recognition_node' in node.get_node_names():
            raise RuntimeError('Stop camera in this domain before synthetic validation')
        if not pump(None, 60, lambda: data.get('status', {}).get('ready') and 'pose' in data):
            raise RuntimeError('Behavior not ready: '+str(data.get('status')))
        pump('Thumb_Up', .8)
        expect('IDLE')
        result['checks']['confirmation_alone_no_motion'] = True
        pump('Pointing_Up', .7)
        expect('SELECTED')
        assert abs(data['speed'][0]) < .01
        result['checks']['selection_alone_no_motion'] = True
        pump('Open_Palm', .2)
        expect('IDLE', 'open_palm_stop')
        result['checks']['open_palm_clears_selection'] = True

        start_circle()
        pump('None', 1.3)
        expect('RUNNING')
        assert abs(data['speed'][0]) > .03
        result['checks']['lowering_hand_keeps_motion'] = True
        pump('Open_Palm', .7)
        expect('IDLE', 'open_palm_stop')
        wait_stopped()
        result['checks']['open_palm_stops_running'] = True

        start_circle()
        pump(None, 1.5)
        expect('IDLE', 'camera_signal_lost')
        wait_stopped()
        result['checks']['missing_camera_messages_stops'] = True
        pump('Thumb_Up', .7)
        expect('IDLE')
        result['checks']['no_automatic_resume_after_camera_loss'] = True

        start_circle()
        pump('None', .2, reason='camera_error')
        expect('IDLE', 'camera_source_failed:camera_error')
        result['checks']['camera_error_stops'] = True

        start_circle()
        pump('Pointing_Up', 1.1)
        expect('SELECTED')
        assert data['status']['selected'] == 'back_and_forth'
        wait_stopped()
        result['checks']['new_selector_interrupts_and_waits_for_confirmation'] = True
        pump('Closed_Fist', .2)
        expect('IDLE', 'closed_fist_cancel')

        start_circle()
        pump('Wave', .15, reason='wave_landmarks', confidence=0.)
        expect('RUNNING')
        assert data['status']['selected'] == 'greeting'
        result['checks']['wave_preempts_without_thumb_up'] = True
        pump('Open_Palm', .15, reason='palm_motion')
        expect('RUNNING')
        result['checks']['moving_palm_does_not_cancel_greeting'] = True
        pump('Closed_Fist', .3, confidence=.54)
        expect('IDLE', 'closed_fist_cancel')
        result['checks']['closed_fist_cancels_running'] = True

        for label, kind in [('Wave', 'greeting'), ('Pointing_Up', 'back_and_forth'), ('Victory', 'circle')]:
            pump('None', .5)
            baseline = data['status']['completions']
            if kind == 'greeting':
                pump(label, .15, reason='wave_landmarks', confidence=0.)
            else:
                pump(label, .7)
                pump('Thumb_Up', .65)
            expect('RUNNING')
            commands.clear()
            pump('Closed_Fist', .5, confidence=.54)
            expect('IDLE', 'closed_fist_cancel')
            assert data['status']['selected'] is None
            assert any(v == 0. and w == 0. for v,w in commands)
            wait_stopped()
            pump('None', .4)
            expect('IDLE')
            result['checks']['low_confidence_fist_cancels_' + kind] = True
            start = data['pose']
            path.clear()
            commands.clear()
            began = time.monotonic()
            if kind == 'greeting':
                pump(label, .15, reason='wave_landmarks', confidence=0.)
                expect('RUNNING')
                assert data['status']['selected'] == 'greeting'
            else:
                pump(label, .7)
                expect('SELECTED')
                pump('Thumb_Up', .65)
                expect('RUNNING')
            while data['status']['completions'] == baseline:
                pump('None', .15)
                if data['status']['state'] == 'IDLE' and data['status']['completions'] == baseline:
                    raise RuntimeError(kind+' stopped: '+str(data['status']))
                if time.monotonic()-began > 110:
                    raise RuntimeError(kind+' timed out')
            pump('None', .3)
            finish = data['pose']
            record = dict(behavior=kind, completed=True, wall_seconds=time.monotonic()-began,
                start=list(start), finish=list(finish),
                return_position_error_m=math.hypot(finish[0]-start[0], finish[1]-start[1]),
                return_heading_error_rad=math.atan2(math.sin(finish[2]-start[2]),math.cos(finish[2]-start[2])),
                cycles=data['status']['completed_cycles'], samples=len(path),
                max_distance_m=max((math.hypot(p[0]-start[0],p[1]-start[1]) for p in path), default=0.),
                peak_linear=max(abs(v) for v,w in commands), peak_angular=max(abs(w) for v,w in commands))
            if kind == 'back_and_forth':
                assert record['cycles'] == 4
            assert record['return_position_error_m'] < .15
            expect('IDLE', 'behavior_completed')
            # Continuing the previous command must not repeat the finished action.
            pump('Thumb_Up', .65)
            expect('IDLE')
            assert data['status']['completions'] == baseline+1
            result['behaviors'].append(record)
            print(json.dumps(record), flush=True)
        result['checks']['each_behavior_completes_once_without_holding_thumb'] = True
        result['passed'] = True
    except Exception as exc:
        import traceback
        traceback.print_exc()
        result.update(passed=False, error=repr(exc))
        print('VALIDATION FAILURE:', exc, flush=True)
    finally:
        pump('Open_Palm', .4)
        result['final_speed'] = list(data.get('speed', []))
        result['events'] = events
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
