#!/usr/bin/env python3
"""Unified Emotion Pipeline: runs Recognition & Behavior in a single process."""
import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor

from .emotion_recognition_node import EmotionRecognitionNode
from .robot_behavior_node import RobotBehaviorNode


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    rec_node = EmotionRecognitionNode()
    beh_node = RobotBehaviorNode()

    executor.add_node(rec_node)
    executor.add_node(beh_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rec_node.destroy_node()
        beh_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
