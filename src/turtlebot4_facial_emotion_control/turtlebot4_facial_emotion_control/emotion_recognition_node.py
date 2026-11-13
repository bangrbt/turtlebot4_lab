#!/usr/bin/env python3
"""ROS 2 Node: Real-time Camera Facial Emotion Recognition via DeepFace / OpenCV / MediaPipe.
Publishes /detected_emotion (JSON) and renders live OpenCV feedback window.
"""
import json
import os
from pathlib import Path
import sys
import time
import cv2

# Silence matplotlib and setup environment
os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')
os.environ.setdefault('DEEPFACE_HOME', '/home/anhbang/turtlebot4_lab_ws/.cache_emotion/deepface')

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
from std_msgs.msg import String

from .camera import CameraManager
from .emotion_detector import EmotionDetector, EmotionResult


class EmotionRecognitionNode(Node):
    """Captures camera frames, infers facial emotion in real time, and publishes to ROS 2."""

    def __init__(self):
        super().__init__('emotion_recognition_node')

        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_fps', 30)
        self.declare_parameter('backend', 'auto')
        self.declare_parameter('min_face_size', 60)
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('smooth_window', 5)
        self.declare_parameter('show_window', True)
        self.declare_parameter('mirror_camera', True)
        self.declare_parameter('emotion_topic', '/detected_emotion')

        # Load parameters
        camera_index = self.get_parameter('camera_index').value
        camera_width = self.get_parameter('camera_width').value
        camera_height = self.get_parameter('camera_height').value
        camera_fps = self.get_parameter('camera_fps').value
        backend = self.get_parameter('backend').value
        min_face_size = self.get_parameter('min_face_size').value
        confidence_threshold = self.get_parameter('confidence_threshold').value
        smooth_window = self.get_parameter('smooth_window').value
        self.show_window = self.get_parameter('show_window').value
        mirror_camera = self.get_parameter('mirror_camera').value
        emotion_topic = self.get_parameter('emotion_topic').value

        # Check DISPLAY environment
        has_display = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
        if not has_display:
            self.show_window = False
            self.get_logger().info("No graphical display detected ($DISPLAY empty). Running in headless mode.")

        # Initialize Camera Manager
        self.camera = CameraManager(
            camera_index=camera_index,
            width=camera_width,
            height=camera_height,
            fps=camera_fps,
            mirror=mirror_camera,
            use_mock_if_failed=True,
        )

        # Initialize Emotion Detector
        self.detector = EmotionDetector(
            backend=backend,
            min_face_size=min_face_size,
            confidence_threshold=confidence_threshold,
            smooth_window=smooth_window,
        )

        # Publisher
        self.emotion_pub = self.create_publisher(String, emotion_topic, 10)

        # FPS Tracking
        self.fps = 0.0
        self.last_frame_time = time.monotonic()
        self.frame_count = 0

        # Processing Timer (~30 FPS)
        timer_period = 1.0 / max(10, camera_fps)
        self.timer = self.create_timer(timer_period, self._process_frame)

        self.get_logger().info(
            f"EmotionRecognitionNode started (backend='{backend}', mock={self.camera.use_mock}, topic='{emotion_topic}')"
        )

    def _process_frame(self) -> None:
        """Capture frame, detect emotion, publish JSON, and display window."""
        ret, frame = self.camera.read_frame()
        if not ret or frame is None:
            return

        now = time.monotonic()
        dt = now - self.last_frame_time
        self.last_frame_time = now
        if dt > 0:
            inst_fps = 1.0 / dt
            self.fps = 0.9 * self.fps + 0.1 * inst_fps if self.fps > 0 else inst_fps

        # Analyze Emotion
        result: EmotionResult = self.detector.analyze(frame)

        # Publish Emotion JSON
        payload = result.to_dict()
        payload['fps'] = round(self.fps, 1)
        payload['timestamp'] = now
        msg = String()
        msg.data = json.dumps(payload)
        self.emotion_pub.publish(msg)

        # Periodic Log
        self.frame_count += 1
        if self.frame_count % 60 == 0:
            if result.face_detected:
                self.get_logger().info(
                    f"Face Detected: {result.emotion.upper()} ({result.vietnamese}) - {result.confidence*100:.1f}% -> {result.action}"
                )
            else:
                self.get_logger().info("Searching for face... (Robot stopped)")

        # Display window
        if self.show_window:
            annotated = self.detector.annotate_frame(frame, result, fps=self.fps)
            cv2.imshow("TurtleBot 4 - Facial Emotion Recognition", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                self.get_logger().info("Window closed by user.")
                rclpy.shutdown()

    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        self.camera.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EmotionRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
