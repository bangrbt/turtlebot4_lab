#!/usr/bin/env python3
"""TB4 Robotics Studio - Autonomous Voice & Semantic Navigation (Lab 07).
Modern, ultra-clean Light Studio UI inspired by professional CAD & Topological Analysis platforms.
Built with PyQt5, ROS 2 Humble, Nav2, and Google Speech / Vosk Speech Recognition.
"""
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time

from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, pyqtSignal, QObject
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QBrush, QPolygonF, QIcon, QPainterPath
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QTextEdit, QProgressBar,
    QCheckBox, QGridLayout, QFrame, QSizePolicy, QScrollArea
)

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

# Lazy-loaded audio backends
sd = None
np = None
vosk = None
sr = None


def get_audio():
    global sd, np, vosk, sr
    if sd is None:
        try:
            import sounddevice as _sd
            import numpy as _np
            sd, np = _sd, _np
        except Exception:
            pass
    if vosk is None:
        try:
            import vosk as _vosk
            _vosk.SetLogLevel(-1)
            vosk = _vosk
        except Exception:
            pass
    if sr is None:
        try:
            import speech_recognition as _sr
            sr = _sr
        except Exception:
            pass
    return sd, np, vosk, sr


ZONE_CFG = {
    'A': {
        'name': 'Khu A: Phòng học',
        'sub': 'Classroom',
        'fill': QColor(240, 249, 255, 230),      # #f0f9ff
        'border': QColor(2, 132, 199),           # #0284c7
        'bounds': (-5.85, 0.875, -0.075, 4.425),
        'target': (-2.2, 2.0),
        'icon': '📚'
    },
    'B': {
        'name': 'Khu B: Phòng Lab',
        'sub': 'Laboratory',
        'fill': QColor(240, 253, 244, 230),      # #f0fdf4
        'border': QColor(22, 163, 74),           # #16a34a
        'bounds': (0.075, 0.875, 5.85, 4.425),
        'target': (2.2, 2.0),
        'icon': '🔬'
    },
    'C': {
        'name': 'Khu C: Văn phòng',
        'sub': 'Office',
        'fill': QColor(255, 247, 237, 230),      # #fff7ed
        'border': QColor(234, 88, 12),           # #ea580c
        'bounds': (0.075, -4.425, 5.85, -0.875),
        'target': (2.2, -2.0),
        'icon': '💼'
    },
    'D': {
        'name': 'Khu D: Thư viện',
        'sub': 'Library',
        'fill': QColor(250, 245, 255, 230),      # #faf5ff
        'border': QColor(147, 51, 234),          # #9333ea
        'bounds': (-5.85, -4.425, -0.075, -0.875),
        'target': (-2.2, -2.0),
        'icon': '📖'
    },
    'E': {
        'name': 'Khu E: Sảnh trung tâm',
        'sub': 'Corridor & Lobby',
        'fill': QColor(248, 250, 252, 220),      # #f8fafc
        'border': QColor(100, 116, 139),         # #64748b
        'bounds': (-5.85, -0.725, 5.85, 0.725),
        'target': (0.0, 0.0),
        'icon': '🏛️'
    },
}


class RosBridge(QObject):
    sig_status = pyqtSignal(str)
    sig_speech = pyqtSignal(str)
    sig_pose = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('voice_navigation_gui_node')
        self.cmd_pub = self.node.create_publisher(String, '/voice/command_text', 10)
        self.node.create_subscription(String, '/voice/navigation_status', self.on_nav_status, 10)
        self.node.create_subscription(String, '/voice/recognized_text', self.on_recognized_text, 10)
        self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, qos_profile_sensor_data)

    def on_nav_status(self, msg: String):
        self.sig_status.emit(msg.data)

    def on_recognized_text(self, msg: String):
        self.sig_speech.emit(msg.data)

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.sig_pose.emit(x, y, yaw)

    def send_command(self, cmd_text: str):
        self.cmd_pub.publish(String(data=cmd_text))


class StudioMapWidget(QWidget):
    """Architectural CAD-style 2D map viewport with clean light grid,
    rich topological zones, realistic obstacle symbols, and live robot telemetry.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(540, 520)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.target_x = None
        self.target_y = None
        self.show_grid = True
        self.show_labels = True
        self.show_obstacles = True
        self.lidar_angle = 0.0

        # Lidar animation timer
        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self._animate_lidar)
        self.anim_timer.start(50)

    def _animate_lidar(self):
        self.lidar_angle = (self.lidar_angle + 12.0) % 360.0
        self.update()

    def set_robot_pose(self, x, y, yaw):
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        self.update()

    def set_target(self, x, y):
        self.target_x = x
        self.target_y = y
        self.update()

    def reset_view(self):
        self.target_x = None
        self.target_y = None
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # 1. High-Tech Studio Canvas Background (#ffffff with subtle slate border)
        p.fillRect(0, 0, w, h, QColor("#ffffff"))

        pad = 28
        side = min(w, h) - 2 * pad
        cx = w / 2.0
        cy = h / 2.0
        scale = side / 13.0  # -6.5m to +6.5m span

        def to_screen(x, y):
            return cx + x * scale, cy - y * scale

        # 2. Subtle Architectural Grid (1.0m minor, 5.0m major)
        if self.show_grid:
            p.setPen(QPen(QColor("#f1f5f9"), 1))
            for m in range(-6, 7):
                gx0, gy0 = to_screen(m, -5.5)
                gx1, gy1 = to_screen(m, 5.5)
                p.drawLine(QPointF(gx0, gy0), QPointF(gx1, gy1))
                gx0, gy0 = to_screen(-6.5, m)
                gx1, gy1 = to_screen(6.5, m)
                p.drawLine(QPointF(gx0, gy0), QPointF(gx1, gy1))

            # Major axes X and Y with coordinate ticks
            p.setPen(QPen(QColor("#e2e8f0"), 1.5))
            ax0, ay0 = to_screen(0, -5.5)
            ax1, ay1 = to_screen(0, 5.5)
            p.drawLine(QPointF(ax0, ay0), QPointF(ax1, ay1))
            ax0, ay0 = to_screen(-6.5, 0)
            ax1, ay1 = to_screen(6.5, 0)
            p.drawLine(QPointF(ax0, ay0), QPointF(ax1, ay1))

        # 3. Outer Room Boundary Box (#cbd5e1 outline)
        ox0, oy0 = to_screen(-6.0, 4.5)
        ox1, oy1 = to_screen(6.0, -4.5)
        p.setPen(QPen(QColor("#94a3b8"), 1, Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        p.drawRect(QRectF(ox0, oy0, ox1 - ox0, oy1 - oy0))

        # 4. Draw 5 Semantic Zones with Elegant Pastel Fills
        for zid, z in ZONE_CFG.items():
            min_x, min_y, max_x, max_y = z['bounds']
            sx0, sy0 = to_screen(min_x, max_y)
            sx1, sy1 = to_screen(max_x, min_y)

            # Zone background fill & border
            p.setPen(QPen(z['border'], 1.5, Qt.SolidLine if zid != 'E' else Qt.DashLine))
            p.setBrush(QBrush(z['fill']))
            p.drawRoundedRect(QRectF(sx0, sy0, sx1 - sx0, sy1 - sy0), 6, 6)

            # Zone Badge Label
            if self.show_labels:
                tx, ty = z['target']
                stx, sty = to_screen(tx, ty)

                # Badge Pill
                badge_w = 126
                badge_h = 24
                badge_rect = QRectF(stx - badge_w / 2.0, sty - badge_h - 10, badge_w, badge_h)
                p.setPen(QPen(z['border'], 1))
                p.setBrush(QBrush(QColor("#ffffff")))
                p.drawRoundedRect(badge_rect, 12, 12)

                # Badge text
                p.setFont(QFont("Segoe UI", 8, QFont.Bold))
                p.setPen(QPen(z['border']))
                p.drawText(badge_rect, Qt.AlignCenter, f"{z['icon']} {z['name']}")

                # Target waypoint golden beacon
                p.setPen(QPen(QColor("#f59e0b"), 1.5))
                p.setBrush(QBrush(QColor(254, 240, 138, 120)))
                p.drawEllipse(QPointF(stx, sty), 8, 8)
                p.setBrush(QBrush(QColor("#d97706")))
                p.drawEllipse(QPointF(stx, sty), 3, 3)

        # 5. Draw Architectural Walls & Doorways (Dark Charcoal #1e293b, width 4px)
        walls = [
            # Outer Walls
            (-6.0, 4.5, 6.0, 4.5),
            (-6.0, -4.5, 6.0, -4.5),
            (-6.0, -4.5, -6.0, 4.5),
            (6.0, -4.5, 6.0, 4.5),

            # Vertical dividing walls
            (0.0, 0.8, 0.0, 4.5),
            (0.0, -4.5, 0.0, -0.8),

            # North corridor wall with doorways
            (-6.0, 0.8, -3.8, 0.8),
            (-2.6, 0.8, 2.6, 0.8),
            (3.8, 0.8, 6.0, 0.8),

            # South corridor wall with doorways
            (-6.0, -0.8, -3.8, -0.8),
            (-2.6, -0.8, 2.6, -0.8),
            (3.8, -0.8, 6.0, -0.8),
        ]
        p.setPen(QPen(QColor("#1e293b"), 4, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        for wx0, wy0, wx1, wy1 in walls:
            px0, py0 = to_screen(wx0, wy0)
            px1, py1 = to_screen(wx1, wy1)
            p.drawLine(QPointF(px0, py0), QPointF(px1, py1))

        # 6. Realistic Obstacles & Furniture Symbols
        if self.show_obstacles:
            # Tables
            tables = [
                (-3.6, 2.7, 0.7, 1.4, QColor("#fed7aa"), "Bàn GV"),
                (3.6, 2.7, 0.8, 1.6, QColor("#bae6fd"), "Lab Bench"),
                (3.6, -2.7, 0.8, 1.6, QColor("#fed7aa"), "Bàn VP"),
                (-3.6, -2.7, 0.8, 1.4, QColor("#fed7aa"), "Bàn Đọc"),
            ]
            for tox, toy, tdx, tdy, tcol, tname in tables:
                t0x, t0y = to_screen(tox - tdx/2.0, toy + tdy/2.0)
                t1x, t1y = to_screen(tox + tdx/2.0, toy - tdy/2.0)
                p.setPen(QPen(QColor("#9a3412"), 1.5))
                p.setBrush(QBrush(tcol))
                p.drawRoundedRect(QRectF(t0x, t0y, t1x - t0x, t1y - t0y), 3, 3)

            # Indoor Potted Plant (Zone D)
            px, py = to_screen(-4.6, -1.8)
            p.setPen(QPen(QColor("#15803d"), 1.5))
            p.setBrush(QBrush(QColor("#86efac")))
            p.drawEllipse(QPointF(px, py), 9, 9)
            p.setBrush(QBrush(QColor("#ea580c")))
            p.drawEllipse(QPointF(px, py), 4, 4)

            # People (Standing icons)
            people = [
                (-1.8, 3.0, "HS"), (1.8, 3.0, "NCV"),
                (1.8, -3.0, "NV"), (-1.8, -3.0, "ĐG")
            ]
            for hx, hy, htag in people:
                hpx, hpy = to_screen(hx, hy)
                p.setPen(QPen(QColor("#334155"), 1.5))
                p.setBrush(QBrush(QColor("#e2e8f0")))
                p.drawEllipse(QPointF(hpx, hpy), 7, 7)
                p.setFont(QFont("Segoe UI", 6, QFont.Bold))
                p.setPen(QPen(QColor("#0f172a")))
                p.drawText(int(hpx - 8), int(hpy - 5), 16, 10, Qt.AlignCenter, htag)

            # Small Obstacles in Corridor & Doorways
            # 1. Wet Floor Sign in corridor
            wfx, wfy = to_screen(-1.2, 0.35)
            p.setPen(QPen(QColor("#ca8a04"), 1.5))
            p.setBrush(QBrush(QColor("#fde047")))
            poly_wfs = QPolygonF([QPointF(wfx, wfy - 6), QPointF(wfx - 5, wfy + 5), QPointF(wfx + 5, wfy + 5)])
            p.drawPolygon(poly_wfs)

            # 2. Caution Cone in corridor
            ccx, ccy = to_screen(1.2, -0.35)
            p.setPen(QPen(QColor("#c2410c"), 1.5))
            p.setBrush(QBrush(QColor("#fb923c")))
            p.drawEllipse(QPointF(ccx, ccy), 5, 5)
            p.setBrush(QBrush(Qt.white))
            p.drawEllipse(QPointF(ccx, ccy), 2, 2)

            # 3. Recycling Bin at Door A
            rbx, rby = to_screen(-2.6, 1.25)
            p.setPen(QPen(QColor("#1d4ed8"), 1.5))
            p.setBrush(QBrush(QColor("#93c5fd")))
            p.drawEllipse(QPointF(rbx, rby), 5, 5)

            # 4. Parcel Box at Door B
            pbx, pby = to_screen(2.6, 1.25)
            p.setPen(QPen(QColor("#b45309"), 1.5))
            p.setBrush(QBrush(QColor("#fcd34d")))
            p.drawRect(QRectF(pbx - 4, pby - 4, 8, 8))

            # 5. Caution Cone at Door C
            cc2x, cc2y = to_screen(2.6, -1.25)
            p.setPen(QPen(QColor("#c2410c"), 1.5))
            p.setBrush(QBrush(QColor("#fb923c")))
            p.drawEllipse(QPointF(cc2x, cc2y), 5, 5)

            # 6. Supply Box at Door D
            sbx, sby = to_screen(-2.6, -1.25)
            p.setPen(QPen(QColor("#b45309"), 1.5))
            p.setBrush(QBrush(QColor("#fcd34d")))
            p.drawRect(QRectF(sbx - 4, sby - 4, 8, 8))

        # 7. Draw Active Planned Path & Goal Target Marker
        if self.target_x is not None and self.target_y is not None:
            gx, gy = to_screen(self.target_x, self.target_y)
            rx, ry = to_screen(self.robot_x, self.robot_y)

            # Path line from robot to target
            p.setPen(QPen(QColor("#2563eb"), 2, Qt.DashLine))
            p.drawLine(QPointF(rx, ry), QPointF(gx, gy))

            # Target Crosshair Marker
            p.setPen(QPen(QColor("#ef4444"), 2))
            p.setBrush(QBrush(QColor(239, 68, 68, 40)))
            p.drawEllipse(QPointF(gx, gy), 14, 14)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(gx, gy), 6, 6)
            p.drawLine(QPointF(gx - 18, gy), QPointF(gx + 18, gy))
            p.drawLine(QPointF(gx, gy - 18), QPointF(gx, gy + 18))

        # 8. High-Tech TurtleBot 4 Visual (Metallic chassis, LiDAR arc, heading vector, pulse halo)
        rx, ry = to_screen(self.robot_x, self.robot_y)
        r_rad = 13.0

        # Pulsing blue halo
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(37, 99, 235, 45)))
        p.drawEllipse(QPointF(rx, ry), r_rad + 8, r_rad + 8)

        # Rotating LiDAR scan wedge
        p.setBrush(QBrush(QColor(16, 185, 129, 35)))
        p.drawPie(QRectF(rx - 38, ry - 38, 76, 76), int(self.lidar_angle * 16), int(55 * 16))

        # Main robot chassis (Dark Titanium #0f172a with silver rim)
        p.setPen(QPen(QColor("#3b82f6"), 2))
        p.setBrush(QBrush(QColor("#0f172a")))
        p.drawEllipse(QPointF(rx, ry), r_rad, r_rad)

        # Lidar top sensor puck
        p.setPen(QPen(QColor("#0284c7"), 1))
        p.setBrush(QBrush(QColor("#1e293b")))
        p.drawEllipse(QPointF(rx, ry), 5, 5)

        # Heading vector arrow
        head_len = r_rad + 14
        hx = rx + head_len * math.cos(self.robot_yaw)
        hy = ry - head_len * math.sin(self.robot_yaw)
        p.setPen(QPen(QColor("#2563eb"), 3, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(rx, ry), QPointF(hx, hy))

        # Arrowhead
        arr_angle = math.atan2(ry - hy, hx - rx)
        p.setBrush(QBrush(QColor("#2563eb")))
        arrow_p1 = QPointF(hx - 8 * math.cos(arr_angle - 0.5), hy + 8 * math.sin(arr_angle - 0.5))
        arrow_p2 = QPointF(hx - 8 * math.cos(arr_angle + 0.5), hy + 8 * math.sin(arr_angle + 0.5))
        p.drawPolygon(QPolygonF([QPointF(hx, hy), arrow_p1, arrow_p2]))

        # Robot Tag
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        p.setPen(QPen(QColor("#0f172a")))
        p.drawText(int(rx - 55), int(ry - r_rad - 16), 110, 15, Qt.AlignCenter, "TURTLEBOT 4")


class VoiceNavigationMainWindow(QMainWindow):
    sig_update_vu = pyqtSignal(float)
    sig_recognized = pyqtSignal(str)

    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("3D ROBOTICS STUDIO • TB4 Autonomous Voice & Semantic Navigation")
        self.resize(1360, 880)
        self.setMinimumSize(1180, 760)

        # Apply ultra-clean Light Studio Styling inspired by modern CAD suites
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f8fafc;
                font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
            }
            QWidget {
                color: #0f172a;
                font-size: 12px;
            }
            QFrame.card {
                background-color: #ffffff;
                border: 1px solid #e2e8f0;
                border-radius: 8px;
            }
            QLabel.section-header {
                font-size: 11px;
                font-weight: 700;
                color: #1e40af;
                text-transform: uppercase;
                letter-spacing: 0.6px;
            }
            QPushButton.zone-btn {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                padding: 10px 8px;
                font-weight: 600;
                text-align: left;
                color: #1e293b;
            }
            QPushButton.zone-btn:hover {
                background-color: #f0fdf4;
                border-color: #16a34a;
                color: #15803d;
            }
            QPushButton.zone-btn:pressed {
                background-color: #dcfce7;
            }
            QLineEdit {
                background-color: #ffffff;
                border: 1px solid #cbd5e1;
                border-radius: 6px;
                padding: 8px 12px;
                font-size: 13px;
                color: #0f172a;
            }
            QLineEdit:focus {
                border: 1px solid #2563eb;
                background-color: #f8fafc;
            }
            QTextEdit {
                background-color: #f8fafc;
                border: 1px solid #e2e8f0;
                border-radius: 6px;
                padding: 6px;
                font-family: "Consolas", "Courier New", monospace;
                font-size: 11px;
                color: #334155;
            }
        """)

        self.is_recording = False
        self.is_continuous = False
        self.vosk_model = None

        self._build_ui()
        self._wire_signals()

        # Spin ROS 2 events cleanly every 20ms in main thread
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros)
        self.ros_timer.start(20)

        # Background Vosk loader
        threading.Thread(target=self._load_vosk_worker, daemon=True).start()

    def _spin_ros(self):
        if rclpy.ok():
            rclpy.spin_once(self.bridge.node, timeout_sec=0.0)

    def _load_vosk_worker(self):
        _, _, v, _ = get_audio()
        if v:
            paths = [
                Path.cwd() / '.cache_voice/vosk-model-small-vn',
                Path.home() / '.cache/turtlebot4_voice/vosk-model-small-vn',
                Path.cwd() / '.cache_voice/vosk-model-small-en-us',
            ]
            for p in paths:
                if p.is_dir():
                    try:
                        self.vosk_model = v.Model(str(p))
                        break
                    except Exception:
                        pass

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QVBoxLayout(central)
        root_layout.setContentsMargins(16, 12, 16, 12)
        root_layout.setSpacing(10)

        # =========================================================================
        # 1. TOP APP BAR (Modern Studio Branding, Status Pills, Viewport Actions)
        # =========================================================================
        app_bar = QFrame()
        app_bar.setProperty("class", "card")
        app_bar_layout = QHBoxLayout(app_bar)
        app_bar_layout.setContentsMargins(14, 8, 14, 8)
        app_bar_layout.setSpacing(12)

        # Left: Logo & Studio Title
        title_box = QHBoxLayout()
        title_box.setSpacing(8)
        lbl_logo = QLabel("🤖")
        lbl_logo.setStyleSheet("font-size: 20px;")
        title_box.addWidget(lbl_logo)

        lbl_app = QLabel("TB4 ROBOTICS STUDIO")
        lbl_app.setStyleSheet("font-size: 15px; font-weight: 800; color: #0f172a; letter-spacing: 0.5px;")
        title_box.addWidget(lbl_app)

        lbl_pill_world = QLabel("smart_campus_v2.world")
        lbl_pill_world.setStyleSheet("background-color: #f1f5f9; color: #475569; font-size: 11px; font-weight: 600; padding: 3px 8px; border-radius: 4px; border: 1px solid #e2e8f0;")
        title_box.addWidget(lbl_pill_world)
        app_bar_layout.addLayout(title_box)

        app_bar_layout.addStretch(1)

        # Center: Telemetry Status Pills
        self.pill_ros = QLabel("● ROS 2 Humble (Domain 77)")
        self.pill_ros.setStyleSheet("color: #15803d; font-size: 11px; font-weight: 600; background-color: #f0fdf4; padding: 4px 10px; border-radius: 12px; border: 1px solid #bbf7d0;")
        app_bar_layout.addWidget(self.pill_ros)

        self.pill_nav2 = QLabel("● Nav2: SẴN SÀNG")
        self.pill_nav2.setStyleSheet("color: #0369a1; font-size: 11px; font-weight: 600; background-color: #f0f9ff; padding: 4px 10px; border-radius: 12px; border: 1px solid #bae6fd;")
        app_bar_layout.addWidget(self.pill_nav2)

        self.pill_loc = QLabel("● AMCL: ĐÃ ĐỊNH VỊ")
        self.pill_loc.setStyleSheet("color: #7e22ce; font-size: 11px; font-weight: 600; background-color: #faf5ff; padding: 4px 10px; border-radius: 12px; border: 1px solid #e9d5ff;")
        app_bar_layout.addWidget(self.pill_loc)

        # Right: Quick Viewport Action Buttons
        self.btn_reset_view = QPushButton("🔄 Đặt lại góc nhìn")
        self.btn_reset_view.setStyleSheet("""
            QPushButton { background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 6px; padding: 5px 12px; font-weight: 600; color: #334155; }
            QPushButton:hover { background-color: #f8fafc; border-color: #94a3b8; }
        """)
        self.btn_reset_view.clicked.connect(self._on_reset_view)
        app_bar_layout.addWidget(self.btn_reset_view)

        root_layout.addWidget(app_bar)

        # =========================================================================
        # 2. MAIN 3-COLUMN STUDIO LAYOUT
        # =========================================================================
        studio_layout = QHBoxLayout()
        studio_layout.setSpacing(12)
        root_layout.addLayout(studio_layout, stretch=1)

        # -------------------------------------------------------------------------
        # LEFT COLUMN: VOICE COMMAND & CONTROL DISPATCH (Width: 340px)
        # -------------------------------------------------------------------------
        left_panel = QWidget()
        left_panel.setFixedWidth(340)
        left_vbox = QVBoxLayout(left_panel)
        left_vbox.setContentsMargins(0, 0, 0, 0)
        left_vbox.setSpacing(10)
        studio_layout.addWidget(left_panel)

        # Card 1: Voice Interaction & Microphone
        card_voice = QFrame()
        card_voice.setProperty("class", "card")
        cv_layout = QVBoxLayout(card_voice)
        cv_layout.setContentsMargins(14, 12, 14, 12)
        cv_layout.setSpacing(8)

        lbl_v_header = QLabel("⚡ ĐIỀU KHIỂN GIỌNG NÓI & MICRO")
        lbl_v_header.setProperty("class", "section-header")
        cv_layout.addWidget(lbl_v_header)

        # Push to Talk Button
        self.btn_ptt = QPushButton("🎙️  BẤM ĐỂ NÓI (PUSH TO TALK)")
        self.btn_ptt.setStyleSheet("""
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2563eb, stop:1 #1d4ed8);
                color: #ffffff;
                font-size: 13px;
                font-weight: 700;
                border-radius: 7px;
                padding: 12px 10px;
                border: 1px solid #1e40af;
            }
            QPushButton:hover {
                background: #1d4ed8;
            }
            QPushButton:pressed {
                background: #1e3a8a;
            }
        """)
        self.btn_ptt.clicked.connect(self._on_toggle_mic_recording)
        cv_layout.addWidget(self.btn_ptt)

        # Audio Volume Meter
        vu_box = QHBoxLayout()
        vu_box.setSpacing(6)
        lbl_vu = QLabel("Cường độ mic:")
        lbl_vu.setStyleSheet("font-size: 11px; color: #64748b;")
        vu_box.addWidget(lbl_vu)

        self.vu_bar = QProgressBar()
        self.vu_bar.setRange(0, 100)
        self.vu_bar.setValue(0)
        self.vu_bar.setTextVisible(False)
        self.vu_bar.setFixedHeight(8)
        self.vu_bar.setStyleSheet("""
            QProgressBar { background-color: #f1f5f9; border-radius: 4px; border: 1px solid #e2e8f0; }
            QProgressBar::chunk { background-color: #10b981; border-radius: 3px; }
        """)
        vu_box.addWidget(self.vu_bar, stretch=1)
        cv_layout.addLayout(vu_box)

        # Audio Status Pill
        self.lbl_mic_status = QLabel("✓ Micro sẵn sàng: Nói trực tiếp hoặc chọn nút")
        self.lbl_mic_status.setStyleSheet("background-color: #f0fdf4; color: #166534; padding: 6px 10px; border-radius: 5px; font-size: 11px; font-weight: 600; border: 1px solid #bbf7d0;")
        self.lbl_mic_status.setWordWrap(True)
        cv_layout.addWidget(self.lbl_mic_status)

        # Continuous Listening Checkbox
        self.chk_continuous = QCheckBox("Lắng nghe liên tục trong môi trường (Continuous VAD)")
        self.chk_continuous.setStyleSheet("font-size: 11px; color: #475569;")
        self.chk_continuous.toggled.connect(self._on_continuous_toggle)
        cv_layout.addWidget(self.chk_continuous)

        left_vbox.addWidget(card_voice)

        # Card 2: Quick Zone Dispatch & Multi-Hop
        card_dispatch = QFrame()
        card_dispatch.setProperty("class", "card")
        cd_layout = QVBoxLayout(card_dispatch)
        cd_layout.setContentsMargins(14, 12, 14, 12)
        cd_layout.setSpacing(8)

        lbl_d_header = QLabel("📍 ĐIỀU HƯỚNG NHANH CÁC KHU VỰC")
        lbl_d_header.setProperty("class", "section-header")
        cd_layout.addWidget(lbl_d_header)

        # 2-Column Grid of Quick Zone Buttons
        grid_zones = QGridLayout()
        grid_zones.setSpacing(6)

        zone_buttons = [
            ("📚 [A] Phòng học", "Đi đến khu vực A", "#0284c7", 0, 0),
            ("🔬 [B] Phòng Lab", "Đi đến phòng thí nghiệm", "#16a34a", 0, 1),
            ("💼 [C] Văn phòng", "Đi đến văn phòng", "#ea580c", 1, 0),
            ("📖 [D] Thư viện", "Đi đến thư viện", "#9333ea", 1, 1),
            ("🏛️ [E] Sảnh chờ", "Đi đến hành lang", "#475569", 2, 0),
            ("🔀 [A ➔ B] Liên phòng", "Di chuyển từ khu vực A đến khu vực B", "#d97706", 2, 1),
            ("🧭 Đa chặng (A->E->C)", "Đi từ phòng học qua sảnh rồi đến văn phòng", "#4f46e5", 3, 0),
            ("⚠️ Điểm không tồn tại", "Đi đến khu vực Z", "#dc2626", 3, 1),
        ]

        for text, cmd, col, r, c in zone_buttons:
            btn = QPushButton(text)
            btn.setProperty("class", "zone-btn")
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: #ffffff;
                    border: 1px solid #e2e8f0;
                    border-left: 3px solid {col};
                    border-radius: 6px;
                    padding: 8px 6px;
                    font-weight: 600;
                    font-size: 11px;
                    color: #1e293b;
                }}
                QPushButton:hover {{
                    background-color: #f8fafc;
                    border-color: {col};
                    color: {col};
                }}
            """)
            btn.clicked.connect(lambda ch, c_text=cmd: self._send_command_text(c_text))
            grid_zones.addWidget(btn, r, c)

        cd_layout.addLayout(grid_zones)

        # Big Emergency Stop Button
        self.btn_estop = QPushButton("🛑  DỪNG KHẨN CẤP (EMERGENCY STOP)")
        self.btn_estop.setStyleSheet("""
            QPushButton {
                background-color: #fee2e2;
                color: #b91c1c;
                border: 1px solid #fca5a5;
                border-radius: 6px;
                padding: 10px;
                font-weight: 700;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #dc2626;
                color: #ffffff;
                border-color: #b91c1c;
            }
        """)
        self.btn_estop.clicked.connect(lambda: self._send_command_text("Dừng lại"))
        cd_layout.addWidget(self.btn_estop)

        left_vbox.addWidget(card_dispatch)

        # Card 3: Custom Text Command Input
        card_text_cmd = QFrame()
        card_text_cmd.setProperty("class", "card")
        ct_layout = QVBoxLayout(card_text_cmd)
        ct_layout.setContentsMargins(14, 10, 14, 10)
        ct_layout.setSpacing(6)

        lbl_t_header = QLabel("💬 NHẬP CÂU LỆNH TÙY Ý")
        lbl_t_header.setProperty("class", "section-header")
        ct_layout.addWidget(lbl_t_header)

        cmd_input_box = QHBoxLayout()
        cmd_input_box.setSpacing(6)
        self.txt_cmd = QLineEdit()
        self.txt_cmd.setPlaceholderText("Nhập câu lệnh (vd: robot đang ở đâu?)...")
        self.txt_cmd.returnPressed.connect(self._on_send_text_command)
        cmd_input_box.addWidget(self.txt_cmd, stretch=1)

        self.btn_send = QPushButton("Gửi ➔")
        self.btn_send.setStyleSheet("""
            QPushButton {
                background-color: #2563eb;
                color: white;
                font-weight: 700;
                border-radius: 6px;
                padding: 8px 14px;
                border: none;
            }
            QPushButton:hover { background-color: #1d4ed8; }
        """)
        self.btn_send.clicked.connect(self._on_send_text_command)
        cmd_input_box.addWidget(self.btn_send)
        ct_layout.addLayout(cmd_input_box)

        left_vbox.addWidget(card_text_cmd)
        left_vbox.addStretch(1)

        # -------------------------------------------------------------------------
        # CENTER COLUMN: ARCHITECTURAL CAD 2D MAP VIEWPORT
        # -------------------------------------------------------------------------
        center_frame = QFrame()
        center_frame.setProperty("class", "card")
        center_layout = QVBoxLayout(center_frame)
        center_layout.setContentsMargins(10, 10, 10, 10)
        center_layout.setSpacing(6)

        # Map Viewport Header
        map_header = QHBoxLayout()
        map_header.setSpacing(8)

        lbl_map_title = QLabel("BẢN ĐỒ PHÂN ĐOẠN 2D • SMART CAMPUS")
        lbl_map_title.setProperty("class", "section-header")
        map_header.addWidget(lbl_map_title)

        map_header.addStretch(1)

        self.chk_show_grid = QCheckBox("Lưới 1.0m")
        self.chk_show_grid.setChecked(True)
        self.chk_show_grid.setStyleSheet("font-size: 11px; color: #64748b;")
        self.chk_show_grid.toggled.connect(self._toggle_grid)
        map_header.addWidget(self.chk_show_grid)

        self.chk_show_obs = QCheckBox("Vật cản")
        self.chk_show_obs.setChecked(True)
        self.chk_show_obs.setStyleSheet("font-size: 11px; color: #64748b;")
        self.chk_show_obs.toggled.connect(self._toggle_obstacles)
        map_header.addWidget(self.chk_show_obs)

        center_layout.addLayout(map_header)

        # Map Canvas Widget
        self.map_widget = StudioMapWidget()
        center_layout.addWidget(self.map_widget, stretch=1)

        studio_layout.addWidget(center_frame, stretch=1)

        # -------------------------------------------------------------------------
        # RIGHT COLUMN: TELEMETRY & TOPOLOGICAL CAD ANALYTICS (Width: 330px)
        # -------------------------------------------------------------------------
        right_panel = QWidget()
        right_panel.setFixedWidth(330)
        right_vbox = QVBoxLayout(right_panel)
        right_vbox.setContentsMargins(0, 0, 0, 0)
        right_vbox.setSpacing(10)
        studio_layout.addWidget(right_panel)

        # Card 1: Live Robot Coordinate Telemetry
        card_telemetry = QFrame()
        card_telemetry.setProperty("class", "card")
        ctl_layout = QVBoxLayout(card_telemetry)
        ctl_layout.setContentsMargins(14, 12, 14, 12)
        ctl_layout.setSpacing(8)

        lbl_t_head = QLabel("CHỈ SỐ ĐỊNH VỊ ROBOT (ODOM & MAP)")
        lbl_t_head.setProperty("class", "section-header")
        ctl_layout.addWidget(lbl_t_head)

        # Metric Stats Row (Big bold numbers like reference CAD studio)
        metrics_row = QHBoxLayout()
        metrics_row.setSpacing(8)

        # Metric 1: X, Y Coordinates
        box_pos = QFrame()
        box_pos.setStyleSheet("background-color: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; padding: 6px;")
        bp_layout = QVBoxLayout(box_pos)
        bp_layout.setSpacing(2)
        lbl_bp_title = QLabel("TỌA ĐỘ (X, Y)")
        lbl_bp_title.setStyleSheet("font-size: 10px; font-weight: 700; color: #64748b;")
        bp_layout.addWidget(lbl_bp_title)
        self.lbl_pos_val = QLabel("0.00, 0.00")
        self.lbl_pos_val.setStyleSheet("font-size: 17px; font-weight: 800; color: #0f172a;")
        bp_layout.addWidget(self.lbl_pos_val)
        lbl_bp_unit = QLabel("mét (map frame)")
        lbl_bp_unit.setStyleSheet("font-size: 9px; color: #94a3b8;")
        bp_layout.addWidget(lbl_bp_unit)
        metrics_row.addWidget(box_pos)

        # Metric 2: Heading Yaw
        box_yaw = QFrame()
        box_yaw.setStyleSheet("background-color: #f8fafc; border: 1px solid #e2e8f0; border-radius: 6px; padding: 6px;")
        by_layout = QVBoxLayout(box_yaw)
        by_layout.setSpacing(2)
        lbl_by_title = QLabel("GÓC QUAY (YAW)")
        lbl_by_title.setStyleSheet("font-size: 10px; font-weight: 700; color: #64748b;")
        by_layout.addWidget(lbl_by_title)
        self.lbl_yaw_val = QLabel("0.0°")
        self.lbl_yaw_val.setStyleSheet("font-size: 17px; font-weight: 800; color: #2563eb;")
        by_layout.addWidget(self.lbl_yaw_val)
        lbl_by_unit = QLabel("độ (East = 0°)")
        lbl_by_unit.setStyleSheet("font-size: 9px; color: #94a3b8;")
        by_layout.addWidget(lbl_by_unit)
        metrics_row.addWidget(box_yaw)

        ctl_layout.addLayout(metrics_row)
        right_vbox.addWidget(card_telemetry)

        # Card 2: Current Semantic Zone Badge
        card_zone = QFrame()
        card_zone.setProperty("class", "card")
        cz_layout = QVBoxLayout(card_zone)
        cz_layout.setContentsMargins(14, 12, 14, 12)
        cz_layout.setSpacing(6)

        lbl_z_head = QLabel("KHU VỰC HIỆN TẠI (CURRENT ZONE)")
        lbl_z_head.setProperty("class", "section-header")
        cz_layout.addWidget(lbl_z_head)

        self.lbl_cur_zone = QLabel("🏛️ Khu E: Sảnh trung tâm")
        self.lbl_cur_zone.setStyleSheet("""
            background-color: #f1f5f9;
            color: #0f172a;
            border: 1px solid #cbd5e1;
            border-radius: 6px;
            padding: 8px 10px;
            font-size: 13px;
            font-weight: 700;
        """)
        cz_layout.addWidget(self.lbl_cur_zone)

        self.lbl_zone_desc = QLabel("Trục giao thông chính kết nối 4 phòng chức năng.")
        self.lbl_zone_desc.setStyleSheet("font-size: 11px; color: #64748b;")
        self.lbl_zone_desc.setWordWrap(True)
        cz_layout.addWidget(self.lbl_zone_desc)

        right_vbox.addWidget(card_zone)

        # Card 3: Nav2 State & Voice Transcript
        card_nav_status = QFrame()
        card_nav_status.setProperty("class", "card")
        cns_layout = QVBoxLayout(card_nav_status)
        cns_layout.setContentsMargins(14, 12, 14, 12)
        cns_layout.setSpacing(6)

        lbl_ns_head = QLabel("TRẠNG THÁI NAV2 & GIỌNG NÓI")
        lbl_ns_head.setProperty("class", "section-header")
        cns_layout.addWidget(lbl_ns_head)

        self.lbl_nav_badge = QLabel("IDLE • Sẵn sàng tiếp nhận lệnh")
        self.lbl_nav_badge.setStyleSheet("""
            background-color: #f0fdf4;
            color: #166534;
            border: 1px solid #bbf7d0;
            border-radius: 5px;
            padding: 6px 8px;
            font-size: 12px;
            font-weight: 700;
        """)
        cns_layout.addWidget(self.lbl_nav_badge)

        self.lbl_last_voice = QLabel("Lệnh gần nhất: Chưa có")
        self.lbl_last_voice.setStyleSheet("font-size: 11px; color: #475569; font-style: italic;")
        self.lbl_last_voice.setWordWrap(True)
        cns_layout.addWidget(self.lbl_last_voice)

        right_vbox.addWidget(card_nav_status)

        # Card 4: Real-time Event & Mission Log
        card_log = QFrame()
        card_log.setProperty("class", "card")
        cl_layout = QVBoxLayout(card_log)
        cl_layout.setContentsMargins(14, 10, 14, 10)
        cl_layout.setSpacing(6)

        lbl_l_head = QLabel("NHẬT KÝ SỰ KIỆN THỜI GIAN THỰC")
        lbl_l_head.setProperty("class", "section-header")
        cl_layout.addWidget(lbl_l_head)

        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        cl_layout.addWidget(self.txt_log, stretch=1)
        right_vbox.addWidget(card_log, stretch=1)

        # Bottom Toast Notification (Like in reference CAD app)
        self.toast_pill = QLabel("✓ Môi trường Smart Campus v2.4 đã sẵn sàng • Micro & Nav2 OK")
        self.toast_pill.setStyleSheet("""
            background-color: #ecfdf5;
            color: #065f46;
            border: 1px solid #a7f3d0;
            border-radius: 6px;
            padding: 6px 10px;
            font-size: 11px;
            font-weight: 600;
        """)
        right_vbox.addWidget(self.toast_pill)

    def _wire_signals(self):
        self.bridge.sig_pose.connect(self._on_pose_received)
        self.bridge.sig_status.connect(self._on_status_received)
        self.bridge.sig_speech.connect(self._on_speech_received)
        self.sig_update_vu.connect(self._on_vu_update)
        self.sig_recognized.connect(self._on_speech_transcribed)

        self.log("[KHỞI ĐỘNG]: Giao diện 3D Robotics Studio - TurtleBot 4 Lab 07 sẵn sàng.")
        self.log("[CẤU HÌNH]: 5 Khu vực phân vùng A, B, C, D, E | Nav2 Action Client kết nối.")

    def _on_pose_received(self, x, y, yaw):
        self.map_widget.set_robot_pose(x, y, yaw)
        self.lbl_pos_val.setText(f"{x:+.2f}, {y:+.2f}")
        yaw_deg = math.degrees(yaw)
        self.lbl_yaw_val.setText(f"{yaw_deg:+.1f}°")

        zone_id = self._determine_zone(x, y)
        if zone_id in ZONE_CFG:
            z = ZONE_CFG[zone_id]
            self.lbl_cur_zone.setText(f"{z['icon']} {z['name']}")
            self.lbl_zone_desc.setText(z['sub'])

    def _determine_zone(self, x, y):
        for zid, z in ZONE_CFG.items():
            min_x, min_y, max_x, max_y = z['bounds']
            if min_x <= x <= max_x and min_y <= y <= max_y:
                return zid
        return 'E'

    def _on_status_received(self, status_json: str):
        try:
            data = json.loads(status_json)
            st = data.get("status", "UNKNOWN")
            msg = data.get("message", "")
            target = data.get("target")

            badge_styles = {
                "IDLE": "background-color: #f1f5f9; color: #475569; border: 1px solid #cbd5e1;",
                "NAVIGATING": "background-color: #eff6ff; color: #1d4ed8; border: 1px solid #bfdbfe;",
                "WAYPOINT_REACHED": "background-color: #fefce8; color: #a16207; border: 1px solid #fef08a;",
                "COMPLETED": "background-color: #f0fdf4; color: #15803d; border: 1px solid #bbf7d0;",
                "CANCELLED": "background-color: #fff7ed; color: #c2410c; border: 1px solid #fed7aa;",
                "FAILED": "background-color: #fef2f2; color: #b91c1c; border: 1px solid #fecaca;",
                "REJECTED": "background-color: #fef2f2; color: #b91c1c; border: 1px solid #fecaca;",
            }

            style = badge_styles.get(st, "background-color: #f1f5f9; color: #0f172a;")
            self.lbl_nav_badge.setText(f"{st} • {msg[:28]}")
            self.lbl_nav_badge.setStyleSheet(f"{style} border-radius: 5px; padding: 6px 8px; font-size: 11px; font-weight: 700;")

            if target and isinstance(target, dict):
                tx = target.get("x")
                ty = target.get("y")
                if tx is not None and ty is not None:
                    self.map_widget.set_target(tx, ty)

            if st == "COMPLETED":
                self.map_widget.target_x = None
                self.map_widget.target_y = None
                self.map_widget.update()

            self.log(f"[{st}]: {msg}")
        except Exception:
            pass

    def _on_speech_received(self, text: str):
        self.lbl_last_voice.setText(f"Lệnh nhận diện: \"{text}\"")
        self.log(f"[GIỌNG NÓI]: Nhận diện thành công: '{text}'")

    def _on_speech_transcribed(self, recognized_text: str):
        self.log(f"[MICRO TRANSCRIPT]: '{recognized_text}'")
        self.lbl_last_voice.setText(f"Đã nói: \"{recognized_text}\"")
        self.bridge.send_command(recognized_text)

    def _send_command_text(self, text: str):
        self.log(f"[PHÁT LỆNH]: '{text}'")
        self.bridge.send_command(text)

    def _on_send_text_command(self):
        text = self.txt_cmd.text().strip()
        if text:
            self._send_command_text(text)
            self.txt_cmd.clear()

    def _on_reset_view(self):
        self.map_widget.reset_view()
        self.log("[VIEWPORT]: Đã đặt lại góc nhìn bản đồ.")

    def _toggle_grid(self, checked):
        self.map_widget.show_grid = checked
        self.map_widget.update()

    def _toggle_obstacles(self, checked):
        self.map_widget.show_obstacles = checked
        self.map_widget.update()

    def _on_continuous_toggle(self, checked):
        self.is_continuous = checked
        if checked:
            self.log("[MICRO]: Bật chế độ lắng nghe liên tục.")
            threading.Thread(target=self._continuous_mic_worker, daemon=True).start()
        else:
            self.log("[MICRO]: Tắt chế độ lắng nghe liên tục.")

    def _on_toggle_mic_recording(self):
        if self.is_recording:
            return
        threading.Thread(target=self._record_worker, daemon=True).start()

    def _record_worker(self):
        s_device, n_py, _, _ = get_audio()
        if s_device is None or n_py is None:
            self.log("[!] Microphone chưa sẵn sàng hoặc thiếu sounddevice/numpy.")
            return

        self.is_recording = True
        self.btn_ptt.setText("🔴  ĐANG LẮNG NGHE... HÃY NÓI!")
        self.btn_ptt.setStyleSheet("""
            QPushButton {
                background-color: #dc2626;
                color: #ffffff;
                font-size: 13px;
                font-weight: 700;
                border-radius: 7px;
                padding: 12px 10px;
                border: 1px solid #b91c1c;
            }
        """)
        self.lbl_mic_status.setText("Trạng thái: Đang thu âm... HÃY NÓI CÂU LỆNH!")
        self.lbl_mic_status.setStyleSheet("background-color: #fef2f2; color: #b91c1c; padding: 6px 10px; border-radius: 5px; font-weight: 600; border: 1px solid #fecaca;")

        rate = 16000
        duration = 3.5
        audio_buffer = []

        def callback(indata, frames, time_info, status):
            vol = float(n_py.linalg.norm(indata) * 10)
            level = min(100, int(vol * 5.0))
            self.sig_update_vu.emit(level)
            audio_buffer.append(indata.copy())

        try:
            with s_device.InputStream(samplerate=rate, channels=1, dtype='int16', callback=callback):
                for _ in range(int(duration * 20)):
                    time.sleep(0.05)

            self.lbl_mic_status.setText("Trạng thái: Đang phân tích nhận dạng giọng nói...")
            self.lbl_mic_status.setStyleSheet("background-color: #fefce8; color: #854d0e; padding: 6px 10px; border-radius: 5px; font-weight: 600; border: 1px solid #fef08a;")
            self.sig_update_vu.emit(0.0)

            recognized_txt = ""
            if audio_buffer:
                raw_bytes = n_py.concatenate(audio_buffer, axis=0).tobytes()
                # 1. Try offline Vosk if model exists
                if self.vosk_model:
                    try:
                        _, _, v, _ = get_audio()
                        rec = v.KaldiRecognizer(self.vosk_model, rate)
                        rec.AcceptWaveform(raw_bytes)
                        res = json.loads(rec.FinalResult())
                        recognized_txt = res.get('text', '').strip()
                    except Exception:
                        pass

                # 2. Try online Google Speech Recognition (Vietnamese / English)
                _, _, _, s_rec = get_audio()
                if not recognized_txt and s_rec is not None:
                    try:
                        r = s_rec.Recognizer()
                        audio_data = s_rec.AudioData(raw_bytes, rate, 2)
                        recognized_txt = r.recognize_google(audio_data, language='vi-VN').strip()
                    except Exception:
                        try:
                            recognized_txt = r.recognize_google(audio_data, language='en-US').strip()
                        except Exception:
                            pass

                if recognized_txt:
                    self.lbl_mic_status.setText(f"✓ Đã nhận diện: '{recognized_txt}'")
                    self.lbl_mic_status.setStyleSheet("background-color: #f0fdf4; color: #166534; padding: 6px 10px; border-radius: 5px; font-weight: 700; border: 1px solid #bbf7d0;")
                    self.sig_recognized.emit(recognized_txt)
                else:
                    self.lbl_mic_status.setText("Không nghe rõ giọng nói. Hãy nói to hoặc bấm lại!")
                    self.lbl_mic_status.setStyleSheet("background-color: #fef2f2; color: #b91c1c; padding: 6px 10px; border-radius: 5px; border: 1px solid #fecaca;")
            else:
                self.lbl_mic_status.setText("Không thu nhận được âm thanh từ micro.")
                self.lbl_mic_status.setStyleSheet("background-color: #fef2f2; color: #b91c1c; padding: 6px 10px; border-radius: 5px; border: 1px solid #fecaca;")

        except Exception as e:
            self.log(f"[LỖI MICRO]: {e}")
            self.lbl_mic_status.setText(f"Lỗi Micro: {e}")
        finally:
            self.is_recording = False
            self.btn_ptt.setText("🎙️  BẤM ĐỂ NÓI (PUSH TO TALK)")
            self.btn_ptt.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #2563eb, stop:1 #1d4ed8);
                    color: #ffffff;
                    font-size: 13px;
                    font-weight: 700;
                    border-radius: 7px;
                    padding: 12px 10px;
                    border: 1px solid #1e40af;
                }
                QPushButton:hover { background: #1d4ed8; }
            """)

    def _continuous_mic_worker(self):
        s_device, n_py, v, _ = get_audio()
        if not s_device or not n_py or not self.vosk_model:
            return
        rate = 16000
        rec = v.KaldiRecognizer(self.vosk_model, rate)

        def cb(indata, frames, time_info, status):
            if not self.is_continuous or self.is_recording:
                return
            vol = float(n_py.linalg.norm(indata) * 10)
            self.sig_update_vu.emit(min(100, int(vol * 5.0)))
            if rec.AcceptWaveform(indata.tobytes()):
                res = json.loads(rec.Result())
                txt = res.get('text', '').strip()
                if txt:
                    self.sig_recognized.emit(txt)

        try:
            with s_device.InputStream(samplerate=rate, channels=1, dtype='int16', callback=cb):
                while self.is_continuous:
                    time.sleep(0.1)
        except Exception:
            pass
        finally:
            self.sig_update_vu.emit(0.0)

    def _on_vu_update(self, val):
        self.vu_bar.setValue(int(val))

    def log(self, text):
        t_str = time.strftime("[%H:%M:%S]")
        self.txt_log.append(f"<span style='color: #64748b;'>{t_str}</span> <span style='color: #0f172a;'>{text}</span>")


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    bridge = RosBridge()
    window = VoiceNavigationMainWindow(bridge)
    window.show()

    ret = app.exec_()
    window.is_continuous = False
    window.is_recording = False
    bridge.node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
