#!/usr/bin/env python3
"""TB4 Emotion Robotics Studio - Live Facial Emotion Control Dashboard (Lab Assignment).
Modern, ultra-clean UI built with PyQt5, OpenCV, and ROS 2 Humble.
"""
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Dict, Optional

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

from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QBrush, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QProgressBar, QFrame, QGridLayout, QGroupBox
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .emotion_detector import TARGET_EMOTIONS, EMOTION_VIETNAMESE, ACTION_DESCRIPTIONS
from .camera import CameraManager


class RosBridge(QObject):
    """Bridges ROS 2 topics to Qt signals."""
    emotion_received = pyqtSignal(dict)
    status_received = pyqtSignal(dict)


class EmotionGuiNode(Node):
    """ROS 2 Node for GUI communication."""

    def __init__(self, bridge: RosBridge):
        super().__init__('emotion_gui_node')
        self.bridge = bridge

        self.override_pub = self.create_publisher(String, '/emotion/manual_override', 10)

        self.emotion_sub = self.create_subscription(String, '/detected_emotion', self._on_emotion, 10)
        self.status_sub = self.create_subscription(String, '/emotion/behavior_status', self._on_status, 10)

    def _on_emotion(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.bridge.emotion_received.emit(data)
        except Exception:
            pass

    def _on_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.bridge.status_received.emit(data)
        except Exception:
            pass

    def send_override(self, command: str):
        msg = String()
        msg.data = command
        self.override_pub.publish(msg)


class EmotionStudioWindow(QMainWindow):
    """Main Dashboard Window."""

    def __init__(self, ros_node: EmotionGuiNode, bridge: RosBridge):
        super().__init__()
        self.ros_node = ros_node
        self.bridge = bridge

        self.setWindowTitle("TurtleBot 4 — Nhận diện Biểu cảm & Điều khiển Robot")
        self.setMinimumSize(1080, 680)

        # Style palette
        self.setStyleSheet("""
            QMainWindow { background-color: #f4f6f9; }
            QLabel { color: #1c2833; font-family: 'Segoe UI', Arial, sans-serif; }
            QGroupBox {
                background-color: #ffffff;
                border: 1px solid #d5dbdb;
                border-radius: 6px;
                margin-top: 12px;
                font-weight: bold;
                color: #004e87;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 14px;
                padding: 0 4px;
            }
            QPushButton {
                background-color: #ffffff;
                border: 1px solid #bdc3c7;
                border-radius: 5px;
                padding: 7px 12px;
                font-weight: bold;
                font-size: 13px;
                color: #2c3e50;
            }
            QPushButton:hover {
                background-color: #e8f4fc;
                border-color: #004e87;
                color: #004e87;
            }
            QProgressBar {
                border: 1px solid #d5dbdb;
                border-radius: 4px;
                text-align: center;
                background-color: #ebedef;
                height: 16px;
                font-size: 11px;
                font-weight: bold;
            }
        """)

        # Main Layout
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(16, 16, 16, 16)
        main_layout.setSpacing(16)

        # LEFT PANE: Camera Stream
        left_pane = QVBoxLayout()
        cam_box = QGroupBox("Hình ảnh Camera & Nhận diện Thời gian thực")
        cam_layout = QVBoxLayout(cam_box)

        self.cam_label = QLabel()
        self.cam_label.setFixedSize(640, 480)
        self.cam_label.setStyleSheet("background-color: #1a1e24; border-radius: 6px;")
        self.cam_label.setAlignment(Qt.AlignCenter)
        cam_layout.addWidget(self.cam_label)

        # Camera Status Bar
        self.cam_status_lbl = QLabel("Camera: Sẵn sàng | FPS: 0.0")
        self.cam_status_lbl.setStyleSheet("font-size: 12px; color: #7f8c8d; font-weight: bold;")
        cam_layout.addWidget(self.cam_status_lbl)

        left_pane.addWidget(cam_box)
        main_layout.addLayout(left_pane, 6)

        # RIGHT PANE: Telemetry, Meters & Controls
        right_pane = QVBoxLayout()

        # Group 1: Dominant Emotion Badge
        badge_box = QGroupBox("Trạng thái Biểu cảm & Hành động")
        badge_layout = QVBoxLayout(badge_box)

        self.badge_lbl = QLabel("NEUTRAL — BÌNH THƯỜNG")
        self.badge_lbl.setAlignment(Qt.AlignCenter)
        self.badge_lbl.setStyleSheet("""
            background-color: #e8f8f5;
            color: #16a085;
            border: 2px solid #1abc9c;
            border-radius: 6px;
            padding: 8px;
            font-size: 17px;
            font-weight: bold;
        """)
        badge_layout.addWidget(self.badge_lbl)

        self.action_lbl = QLabel("Hành động: Dừng tại chỗ")
        self.action_lbl.setStyleSheet("font-size: 13px; color: #2c3e50; font-weight: bold; padding-top: 4px;")
        badge_layout.addWidget(self.action_lbl)

        right_pane.addWidget(badge_box)

        # Group 2: 6 Emotion Probability Gauges
        gauge_box = QGroupBox("Xác suất 6 Biểu cảm Khuôn mặt (AI)")
        gauge_layout = QGridLayout(gauge_box)
        gauge_layout.setSpacing(6)

        self.bars = {}
        self.bar_labels = {}
        row = 0
        colors = {
            'happy': '#27ae60',     # Green
            'sad': '#2980b9',       # Blue
            'neutral': '#7f8c8d',   # Gray
            'angry': '#c0392b',     # Crimson
            'surprise': '#d35400',  # Orange
            'fear': '#8e44ad',      # Purple
        }

        for emo in TARGET_EMOTIONS:
            vn = EMOTION_VIETNAMESE[emo]
            lbl = QLabel(f"{emo.capitalize()} ({vn}):")
            lbl.setFixedWidth(110)
            lbl.setStyleSheet("font-size: 12px;")

            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setValue(0)
            bar.setStyleSheet(f"""
                QProgressBar::chunk {{
                    background-color: {colors[emo]};
                    border-radius: 3px;
                }}
            """)

            val_lbl = QLabel("0.0%")
            val_lbl.setFixedWidth(45)
            val_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            val_lbl.setStyleSheet("font-size: 11px; font-weight: bold;")

            gauge_layout.addWidget(lbl, row, 0)
            gauge_layout.addWidget(bar, row, 1)
            gauge_layout.addWidget(val_lbl, row, 2)

            self.bars[emo] = bar
            self.bar_labels[emo] = val_lbl
            row += 1

        right_pane.addWidget(gauge_box)

        # Group 3: Robot Velocity Telemetry
        vel_box = QGroupBox("Vận tốc Điều khiển TurtleBot 4 (/cmd_vel)")
        vel_layout = QGridLayout(vel_box)

        self.lin_vel_lbl = QLabel("Vận tốc dài (vx): 0.00 m/s")
        self.lin_vel_lbl.setStyleSheet("font-size: 13px; font-weight: bold; color: #004e87;")
        self.ang_vel_lbl = QLabel("Vận tốc góc (wz): 0.00 rad/s")
        self.ang_vel_lbl.setStyleSheet("font-size: 13px; font-weight: bold; color: #004e87;")
        self.watchdog_lbl = QLabel("Watchdog: AN TOÀN")
        self.watchdog_lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #27ae60;")

        vel_layout.addWidget(self.lin_vel_lbl, 0, 0)
        vel_layout.addWidget(self.ang_vel_lbl, 0, 1)
        vel_layout.addWidget(self.watchdog_lbl, 1, 0, 1, 2)

        right_pane.addWidget(vel_box)

        # Group 4: Manual Test Buttons
        btn_box = QGroupBox("Thử nghiệm Nhanh Biểu cảm & Quỹ đạo Robot")
        btn_layout = QGridLayout(btn_box)
        btn_layout.setSpacing(6)

        buttons = [
            ("😄 Happy (Cong Trái)", "happy", 0, 0),
            ("😢 Sad (Cong Phải)", "sad", 0, 1),
            ("😐 Neutral (Dừng)", "neutral", 1, 0),
            ("😡 Angry (Lùi nhẹ)", "angry", 1, 1),
            ("😲 Surprise (Xoay 360°)", "surprise", 2, 0),
            ("😨 Fear (Tiến nhẹ)", "fear", 2, 1),
            ("🔄 Tự động (Camera)", "auto", 3, 0),
            ("🛑 DỪNG KHẨN CẤP", "estop", 3, 1),
        ]

        for text, cmd, r, c in buttons:
            btn = QPushButton(text)
            if cmd == "estop":
                btn.setStyleSheet("background-color: #fce4e4; color: #c0392b; border-color: #e74c3c;")
            elif cmd == "auto":
                btn.setStyleSheet("background-color: #e8f4fc; color: #004e87; border-color: #004e87;")
            btn.clicked.connect(lambda checked, c_name=cmd: self._send_override(c_name))
            btn_layout.addWidget(btn, r, c)

        right_pane.addWidget(btn_box)

        main_layout.addLayout(right_pane, 4)

        # Connect Signals
        self.bridge.emotion_received.connect(self._update_emotion_display)
        self.bridge.status_received.connect(self._update_status_display)

        # Local camera capture for smooth UI rendering
        self.camera = CameraManager(camera_index=0, width=640, height=480, fps=30, use_mock_if_failed=True)
        self.cam_timer = QTimer()
        self.cam_timer.timeout.connect(self._render_camera_frame)
        self.cam_timer.start(33)  # ~30 FPS

        self.last_annotated_result: Optional[dict] = None

    def _send_override(self, cmd: str):
        self.ros_node.send_override(cmd)

    def _update_emotion_display(self, data: dict):
        self.last_annotated_result = data

        emo = data.get('emotion', 'neutral')
        vn = data.get('vietnamese', 'Bình thường')
        conf = data.get('confidence', 0.0) * 100
        action_desc = data.get('action_desc', 'Dừng tại chỗ')
        face_detected = data.get('face_detected', False)

        badge_colors = {
            'happy': ('#eafaf1', '#27ae60', '#2ecc71'),
            'sad': ('#ebf5fb', '#2980b9', '#3498db'),
            'neutral': ('#f2f4f4', '#7f8c8d', '#95a5a6'),
            'angry': ('#fadbd8', '#c0392b', '#e74c3c'),
            'surprise': ('#fdebd0', '#d35400', '#e67e22'),
            'fear': ('#f5eef8', '#8e44ad', '#9b59b6'),
        }

        if face_detected:
            bg, text_c, border_c = badge_colors.get(emo, ('#ffffff', '#000000', '#cccccc'))
            self.badge_lbl.setText(f"{emo.upper()} — {vn.upper()} ({int(conf)}%)")
            self.badge_lbl.setStyleSheet(f"""
                background-color: {bg};
                color: {text_c};
                border: 2px solid {border_c};
                border-radius: 6px;
                padding: 8px;
                font-size: 16px;
                font-weight: bold;
            """)
            self.action_lbl.setText(f"Hành động: {action_desc}")
        else:
            self.badge_lbl.setText("KHÔNG TÌM THẤY MẶT (CHỜ CAMERA)")
            self.badge_lbl.setStyleSheet("""
                background-color: #fbeee6;
                color: #e67e22;
                border: 2px dashed #d35400;
                border-radius: 6px;
                padding: 8px;
                font-size: 15px;
                font-weight: bold;
            """)
            self.action_lbl.setText("Hành động: Dừng an toàn (Mất dấu)")

        # Update bars
        probs = data.get('probabilities', {})
        for k in TARGET_EMOTIONS:
            val = float(probs.get(k, 0.0))
            self.bars[k].setValue(int(val))
            self.bar_labels[k].setText(f"{val:.1f}%")

    def _update_status_display(self, data: dict):
        vx = data.get('current_linear_x', 0.0)
        wz = data.get('current_angular_z', 0.0)
        stopped = data.get('safety_stopped', False)
        override = data.get('manual_override', False)

        self.lin_vel_lbl.setText(f"Vận tốc dài (vx): {vx:+.2f} m/s")
        self.ang_vel_lbl.setText(f"Vận tốc góc (wz): {wz:+.2f} rad/s")

        if stopped:
            self.watchdog_lbl.setText("Watchdog: DỪNG AN TOÀN (Tín hiệu mất)")
            self.watchdog_lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #e74c3c;")
        elif override:
            self.watchdog_lbl.setText("Chế độ: ĐIỀU KHIỂN THỦ CÔNG (MANUAL OVERRIDE)")
            self.watchdog_lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #e67e22;")
        else:
            self.watchdog_lbl.setText("Watchdog: HOẠT ĐỘNG BÌNH THƯỜNG (TỰ ĐỘNG)")
            self.watchdog_lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #27ae60;")

    def _render_camera_frame(self):
        ret, frame = self.camera.read_frame()
        if not ret or frame is None:
            return

        h, w = frame.shape[:2]
        cam_type = "Mô phỏng (Mock Face)" if self.camera.use_mock else "Webcam Phần cứng"
        self.cam_status_lbl.setText(f"Camera: {cam_type} | Độ phân giải: {w}x{h}")

        # Convert to QImage
        import cv2
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        bytes_per_line = 3 * w
        q_img = QImage(rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_img).scaled(640, 480, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.cam_label.setPixmap(pixmap)

    def closeEvent(self, event):
        self.cam_timer.stop()
        self.camera.release()
        event.accept()


def _ros_spin(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    bridge = RosBridge()
    node = EmotionGuiNode(bridge)

    # Spin ROS 2 in background thread
    spin_thread = threading.Thread(target=_ros_spin, args=(node,), daemon=True)
    spin_thread.start()

    window = EmotionStudioWindow(node, bridge)
    window.show()

    exit_code = app.exec_()
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
