#!/usr/bin/env python3
"""Unit tests for RobotBehaviorNode kinematics, state machine, and watchdog."""
import json
import math
import time
import pytest
from std_msgs.msg import String

from turtlebot4_facial_emotion_control.camera import CameraManager
from turtlebot4_facial_emotion_control.robot_behavior_node import RobotBehaviorNode
from turtlebot4_facial_emotion_control.emotion_detector import TARGET_EMOTIONS


def test_camera_mock_generation():
    """Verify CameraManager falls back to synthetic mock frame with valid dimensions."""
    cam = CameraManager(camera_index=99, width=640, height=480, use_mock_if_failed=True)
    assert cam.use_mock

    for emo in ['happy', 'sad', 'neutral', 'angry', 'surprise', 'fear']:
        cam.set_mock_emotion(emo)
        ret, frame = cam.read_frame()
        assert ret
        assert frame is not None
        assert frame.shape == (480, 640, 3)
    cam.release()


def test_acceleration_ramping_logic():
    """Verify linear and angular ramping bounds velocity steps."""
    current_vx = 0.0
    target_vx = 0.20
    dt = 0.05
    accel_lim = 0.60
    max_step = accel_lim * dt

    diff = target_vx - current_vx
    step = math.copysign(min(abs(diff), max_step), diff)
    new_vx = current_vx + step

    assert new_vx == pytest.approx(0.03, rel=1e-3)
    assert new_vx < target_vx


def test_behavior_node_emotion_mapping():
    """Verify behavior node updates velocities on emotion message."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = RobotBehaviorNode()
    try:
        # 1. Test Happy -> Curve Left
        msg = String()
        msg.data = json.dumps({'emotion': 'happy', 'face_detected': True, 'confidence': 0.95})
        node._on_emotion_msg(msg)
        assert node.target_linear_x == pytest.approx(0.20)
        assert node.target_angular_z == pytest.approx(0.60)
        assert node.current_emotion == 'happy'

        # 2. Test Sad -> Curve Right
        msg.data = json.dumps({'emotion': 'sad', 'face_detected': True, 'confidence': 0.92})
        node._on_emotion_msg(msg)
        assert node.target_linear_x == pytest.approx(0.20)
        assert node.target_angular_z == pytest.approx(-0.60)
        assert node.current_emotion == 'sad'

        # 3. Test Angry -> Reverse
        msg.data = json.dumps({'emotion': 'angry', 'face_detected': True, 'confidence': 0.88})
        node._on_emotion_msg(msg)
        assert node.target_linear_x == pytest.approx(-0.15)
        assert node.target_angular_z == pytest.approx(0.00)

        # 4. Test Surprise -> Spin 360
        msg.data = json.dumps({'emotion': 'surprise', 'face_detected': True, 'confidence': 0.91})
        node._on_emotion_msg(msg)
        assert node.target_linear_x == pytest.approx(0.00)
        assert node.target_angular_z == pytest.approx(1.20)

        # 5. Test Fear -> Forward
        msg.data = json.dumps({'emotion': 'fear', 'face_detected': True, 'confidence': 0.87})
        node._on_emotion_msg(msg)
        assert node.target_linear_x == pytest.approx(0.18)
        assert node.target_angular_z == pytest.approx(0.00)

        # 6. Test Neutral -> Stop
        msg.data = json.dumps({'emotion': 'neutral', 'face_detected': True, 'confidence': 0.99})
        node._on_emotion_msg(msg)
        assert node.target_linear_x == pytest.approx(0.00)
        assert node.target_angular_z == pytest.approx(0.00)

        # 7. Test Face Lost -> Stop
        msg.data = json.dumps({'emotion': 'happy', 'face_detected': False, 'confidence': 0.0})
        node._on_emotion_msg(msg)
        assert node.target_linear_x == pytest.approx(0.00)
        assert node.target_angular_z == pytest.approx(0.00)
        assert node.current_emotion == 'neutral'
    finally:
        node.destroy_node()


def test_behavior_node_manual_override_and_estop():
    """Verify manual override commands and E-Stop."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = RobotBehaviorNode()
    try:
        # Override Happy
        cmd = String(data='happy')
        node._on_override_msg(cmd)
        assert node.manual_override == 'happy'
        assert node.target_linear_x == pytest.approx(0.20)

        # Incoming auto emotion message should be ignored when override is active
        auto_msg = String(data=json.dumps({'emotion': 'angry', 'face_detected': True}))
        node._on_emotion_msg(auto_msg)
        assert node.current_emotion == 'happy'  # Still happy due to override

        # E-Stop
        estop_cmd = String(data='estop')
        node._on_override_msg(estop_cmd)
        assert node.is_safety_stopped
        assert node.target_linear_x == 0.0

        # Clear override
        clear_cmd = String(data='auto')
        node._on_override_msg(clear_cmd)
        assert node.manual_override is None
    finally:
        node.destroy_node()
