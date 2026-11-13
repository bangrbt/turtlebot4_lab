#!/usr/bin/env python3
"""ROS 2 Node: Translates detected facial emotions to TurtleBot 4 /cmd_vel trajectories.
Includes smooth acceleration ramping, safety watchdog timeout, and manual overrides.
"""
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Dict, Optional, Tuple

# Inject workspace virtualenv site-packages
for _cand in [
    Path('/home/anhbang/turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages'),
    Path.home() / 'turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages',
    Path(__file__).resolve().parents[4] / '.venv_gesture/lib/python3.10/site-packages',
    Path(__file__).resolve().parents[3] / '.venv_gesture/lib/python3.10/site-packages',
]:
    if _cand.is_dir() and str(_cand) not in sys.path:
        sys.path.insert(0, str(_cand))
        break

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .emotion_detector import TARGET_EMOTIONS, ACTION_DESCRIPTIONS, EMOTION_VIETNAMESE


class RobotBehaviorNode(Node):
    """Translates facial emotion states into smooth TurtleBot 4 differential-drive trajectories."""

    def __init__(self):
        super().__init__('robot_behavior_node')

        # Parameter Declarations
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('emotion_topic', '/detected_emotion')
        self.declare_parameter('override_topic', '/emotion/manual_override')
        self.declare_parameter('status_topic', '/emotion/behavior_status')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('signal_timeout', 1.0)
        self.declare_parameter('linear_accel_limit', 0.60)
        self.declare_parameter('angular_accel_limit', 2.00)

        # 6 Target Emotion Speeds
        self.declare_parameter('happy_linear_x', 0.20)
        self.declare_parameter('happy_angular_z', 0.60)

        self.declare_parameter('sad_linear_x', 0.20)
        self.declare_parameter('sad_angular_z', -0.60)

        self.declare_parameter('neutral_linear_x', 0.00)
        self.declare_parameter('neutral_angular_z', 0.00)

        self.declare_parameter('angry_linear_x', -0.15)
        self.declare_parameter('angry_angular_z', 0.00)

        self.declare_parameter('surprise_linear_x', 0.00)
        self.declare_parameter('surprise_angular_z', 1.20)

        self.declare_parameter('fear_linear_x', 0.18)
        self.declare_parameter('fear_angular_z', 0.00)

        # Load parameters
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.emotion_topic = self.get_parameter('emotion_topic').value
        self.override_topic = self.get_parameter('override_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.signal_timeout = float(self.get_parameter('signal_timeout').value)
        self.linear_accel_limit = float(self.get_parameter('linear_accel_limit').value)
        self.angular_accel_limit = float(self.get_parameter('angular_accel_limit').value)

        # Velocity lookups
        self.speed_map = {
            'happy': (float(self.get_parameter('happy_linear_x').value), float(self.get_parameter('happy_angular_z').value)),
            'sad': (float(self.get_parameter('sad_linear_x').value), float(self.get_parameter('sad_angular_z').value)),
            'neutral': (float(self.get_parameter('neutral_linear_x').value), float(self.get_parameter('neutral_angular_z').value)),
            'angry': (float(self.get_parameter('angry_linear_x').value), float(self.get_parameter('angry_angular_z').value)),
            'surprise': (float(self.get_parameter('surprise_linear_x').value), float(self.get_parameter('surprise_angular_z').value)),
            'fear': (float(self.get_parameter('fear_linear_x').value), float(self.get_parameter('fear_angular_z').value)),
        }

        # Runtime State
        self.current_emotion = 'neutral'
        self.manual_override: Optional[str] = None
        self.last_msg_time = time.monotonic()
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.is_safety_stopped = False
        self.state_start_time = time.monotonic()

        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.emotion_sub = self.create_subscription(String, self.emotion_topic, self._on_emotion_msg, 10)
        self.override_sub = self.create_subscription(String, self.override_topic, self._on_override_msg, 10)

        # Control Loop Timer (20 Hz)
        timer_period = 1.0 / self.control_rate_hz
        self.timer = self.create_timer(timer_period, self._control_loop)
        self.last_tick_time = time.monotonic()

        self.get_logger().info(
            f"RobotBehaviorNode initialized on {self.cmd_vel_topic} at {self.control_rate_hz}Hz. Ready to execute 6 emotion trajectories!"
        )

    def _on_emotion_msg(self, msg: String) -> None:
        """Handle incoming emotion from vision node."""
        try:
            data = json.loads(msg.data)
            face_detected = data.get('face_detected', False)

            if not face_detected:
                # No face detected
                self.target_linear_x, self.target_angular_z = 0.0, 0.0
                if self.current_emotion != 'neutral':
                    self.current_emotion = 'neutral'
                    self.state_start_time = time.monotonic()
                return

            detected_emo = data.get('emotion', 'neutral').lower()
            if detected_emo in self.speed_map and self.manual_override is None:
                if detected_emo != self.current_emotion:
                    self.current_emotion = detected_emo
                    self.state_start_time = time.monotonic()
                self.target_linear_x, self.target_angular_z = self.speed_map[detected_emo]
                self.last_msg_time = time.monotonic()
                self.is_safety_stopped = False
        except Exception as e:
            self.get_logger().warn(f"Failed to parse emotion JSON: {e}")

    def _on_override_msg(self, msg: String) -> None:
        """Handle manual override commands from GUI or tests."""
        cmd = msg.data.strip().lower()
        if cmd in ('auto', 'clear', 'none'):
            self.manual_override = None
            self.get_logger().info("Manual override cleared. Resumed auto emotion detection.")
        elif cmd in self.speed_map:
            self.manual_override = cmd
            self.current_emotion = cmd
            self.state_start_time = time.monotonic()
            self.target_linear_x, self.target_angular_z = self.speed_map[cmd]
            self.last_msg_time = time.monotonic()
            self.is_safety_stopped = False
            self.get_logger().info(f"Manual override forced to emotion: {cmd.upper()}")
        elif cmd in ('stop', 'estop'):
            self.manual_override = 'neutral'
            self.current_emotion = 'neutral'
            self.target_linear_x, self.target_angular_z = 0.0, 0.0
            self.current_linear_x, self.current_angular_z = 0.0, 0.0
            self.is_safety_stopped = True
            self.get_logger().warn("EMERGENCY STOP (E-STOP) triggered via override!")

    def _control_loop(self) -> None:
        """Periodic control loop: apply safety watchdog, acceleration ramp, and publish Twist."""
        now = time.monotonic()
        dt = max(1e-3, now - self.last_tick_time)
        self.last_tick_time = now

        # 1. Watchdog: Check signal timeout (unless manual override is active)
        if self.manual_override is None and (now - self.last_msg_time > self.signal_timeout):
            self.target_linear_x = 0.0
            self.target_angular_z = 0.0
            if not self.is_safety_stopped:
                self.is_safety_stopped = True
                self.current_emotion = 'neutral'
                self.get_logger().warn(f"Watchdog Timeout (> {self.signal_timeout}s without face signal). Robot stopped.")

        # 2. Acceleration Ramping Filter (S-curve smooth acceleration)
        max_lin_delta = self.linear_accel_limit * dt
        lin_diff = self.target_linear_x - self.current_linear_x
        if abs(lin_diff) <= max_lin_delta:
            self.current_linear_x = self.target_linear_x
        else:
            self.current_linear_x += math.copysign(max_lin_delta, lin_diff)

        max_ang_delta = self.angular_accel_limit * dt
        ang_diff = self.target_angular_z - self.current_angular_z
        if abs(ang_diff) <= max_ang_delta:
            self.current_angular_z = self.target_angular_z
        else:
            self.current_angular_z += math.copysign(max_ang_delta, ang_diff)

        # 3. Publish /cmd_vel
        twist = Twist()
        twist.linear.x = float(self.current_linear_x)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(self.current_angular_z)
        self.cmd_vel_pub.publish(twist)

        # 4. Publish Telemetry Status
        action_name, action_desc = ACTION_DESCRIPTIONS.get(self.current_emotion, ('STOP', 'Dừng'))
        status_payload = {
            'emotion': self.current_emotion,
            'vietnamese': EMOTION_VIETNAMESE.get(self.current_emotion, 'Bình thường'),
            'action': action_name,
            'action_desc': action_desc,
            'manual_override': self.manual_override is not None,
            'target_linear_x': round(self.target_linear_x, 3),
            'target_angular_z': round(self.target_angular_z, 3),
            'current_linear_x': round(self.current_linear_x, 3),
            'current_angular_z': round(self.current_angular_z, 3),
            'safety_stopped': self.is_safety_stopped,
            'time_in_state': round(now - self.state_start_time, 2),
            'timestamp': now,
        }
        msg = String()
        msg.data = json.dumps(status_payload)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotBehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Emergency stop on exit
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
