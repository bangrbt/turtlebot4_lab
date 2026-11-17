"""Regression checks for bounded cancellation and a stopped ROS clock.

No goal is sent; run on an isolated ROS_DOMAIN_ID.
"""
import time
from unittest.mock import Mock
import pytest
import rclpy
from turtlebot4_lab05.sequential_navigation import SequentialNavigation


@pytest.fixture
def mission():
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true',
                    '-p', 'readiness_timeout:=0.3'])
    node = SequentialNavigation()
    node.action.send_goal_async = Mock()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_ctrl_c_without_goal_ack_does_not_wait_for_goal_timeout(mission, monkeypatch):
    from turtlebot4_lab05 import sequential_navigation as module
    mission.state = 'SENDING'
    mission.goal_started = 100.0
    mission.log = Mock()
    monkeypatch.setattr(module.time, 'monotonic', lambda: 101.0)
    mission.request_cancel('interrupt')
    monkeypatch.setattr(module.time, 'monotonic', lambda: 110.0)
    mission.tick()
    assert mission.exit_code == 130
    assert mission.state == 'DONE'
    assert mission.log.call_args.args[0] == 'cancel_timeout'
    mission.action.send_goal_async.assert_not_called()


def test_goal_timeout_cannot_overwrite_user_interrupt(mission):
    mission.state = 'SENDING'
    mission.request_cancel('interrupt')
    mission.request_cancel('goal_timeout')
    assert mission.cancel_reason == 'interrupt'
    mission.action.send_goal_async.assert_not_called()


def test_readiness_timeout_runs_without_simulation_clock(mission):
    assert mission.get_clock().now().nanoseconds == 0
    deadline = time.monotonic()+2.0
    while mission.exit_code is None and time.monotonic()<deadline:
        rclpy.spin_once(mission, timeout_sec=0.05)
    assert mission.exit_code == 1
    assert mission.get_clock().now().nanoseconds == 0
    mission.action.send_goal_async.assert_not_called()
