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

# Automatically inject workspace virtual environment site-packages (sounddevice, vosk, speech_recognition, etc.)
for _venv_cand in [
    Path('/home/anhbang/turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages'),
    Path.home() / 'turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages',
    Path(__file__).resolve().parents[4] / '.venv_gesture/lib/python3.10/site-packages',
    Path(__file__).resolve().parents[3] / '.venv_gesture/lib/python3.10/site-packages',
]:
    if _venv_cand.is_dir() and str(_venv_cand) not in sys.path:
        sys.path.insert(0, str(_venv_cand))
        break

from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, pyqtSignal, QObject
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QBrush, QPolygonF, QIcon, QPainterPath, QImage, QPixmap
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QTextEdit, QProgressBar,
    QCheckBox, QGridLayout, QFrame, QSizePolicy, QScrollArea
)

from typing import Any, List, Optional, Tuple, Dict

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovarianceStamped
    ROS2_AVAILABLE = True
except ImportError:
    rclpy = None
    Node = object
    String = Any
    PoseWithCovarianceStamped = Any
    ROS2_AVAILABLE = False

try:
    from nav_msgs.msg import Path as RosNavPath
except ImportError:
    RosNavPath = None

# Lazy-loaded audio backends
sd = None
np = None
vosk = None
sr = None


def get_audio():
    global sd, np, vosk, sr
    for _venv_cand in [
        Path('/home/anhbang/turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages'),
        Path.home() / 'turtlebot4_lab_ws/.venv_gesture/lib/python3.10/site-packages',
    ]:
        if _venv_cand.is_dir() and str(_venv_cand) not in sys.path:
            sys.path.insert(0, str(_venv_cand))
            break

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
        'fill': QColor(2, 132, 199, 35),
        'border': QColor(2, 132, 199),
        'bounds': (-5.85, 0.875, -0.075, 4.425),
        'target': (-2.2, 2.0),
        'badge_pos': (-3.0, 3.85),
        'icon': '📚'
    },
    'B': {
        'name': 'Khu B: Phòng Lab',
        'sub': 'Laboratory',
        'fill': QColor(22, 163, 74, 35),
        'border': QColor(22, 163, 74),
        'bounds': (0.075, 0.875, 5.85, 4.425),
        'target': (2.2, 2.0),
        'badge_pos': (3.0, 3.85),
        'icon': '🔬'
    },
    'C': {
        'name': 'Khu C: Văn phòng',
        'sub': 'Office',
        'fill': QColor(234, 88, 12, 35),
        'border': QColor(234, 88, 12),
        'bounds': (0.075, -4.425, 5.85, -0.875),
        'target': (2.2, -2.0),
        'badge_pos': (3.0, -1.25),
        'icon': '💼'
    },
    'D': {
        'name': 'Khu D: Thư viện',
        'sub': 'Library',
        'fill': QColor(147, 51, 234, 35),
        'border': QColor(147, 51, 234),
        'bounds': (-5.85, -4.425, -0.075, -0.875),
        'target': (-2.2, -2.0),
        'badge_pos': (-3.0, -1.25),
        'icon': '📖'
    },
    'E': {
        'name': 'Khu E: Sảnh trung tâm',
        'sub': 'Corridor & Lobby',
        'fill': QColor(100, 116, 139, 30),
        'border': QColor(100, 116, 139),
        'bounds': (-5.85, -0.725, 5.85, 0.725),
        'target': (0.0, 0.0),
        'badge_pos': (0.0, 0.45),
        'icon': '🏛️'
    },
}


class RosBridge(QObject):
    sig_status = pyqtSignal(str)
    sig_speech = pyqtSignal(str)
    sig_pose = pyqtSignal(float, float, float)
    sig_plan = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        if ROS2_AVAILABLE and rclpy.ok():
            self.node = rclpy.create_node('voice_navigation_gui_node')
            self.cmd_pub = self.node.create_publisher(String, '/voice/command_text', 10)
            self.node.create_subscription(String, '/voice/navigation_status', self.on_nav_status, 10)
            self.node.create_subscription(String, '/voice/recognized_text', self.on_recognized_text, 10)
            self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, qos_profile_sensor_data)
            if RosNavPath is not None:
                self.node.create_subscription(RosNavPath, '/plan', self.on_plan, 10)
        else:
            self.node = None

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

    def on_plan(self, msg):
        try:
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.sig_plan.emit(pts)
        except Exception:
            pass

    def send_command(self, cmd_text: str):
        if self.node is not None:
            self.cmd_pub.publish(String(data=cmd_text))


class StudioMapWidget(QWidget):
    """Architectural & Simulation 2D Viewport:
    - Renders the real Occupancy Grid map (map.pgm) with millimetric physical alignment.
    - Displays live Nav2 global plan trajectory (/plan) with glowing vector path.
    - Renders topological semantic zones, non-overlapping bilingual badges, and multi-hop routes.
    - Displays high-fidelity TurtleBot 4 telemetry with rotating LiDAR arc and heading beam.
    - Supports interactive drag-to-pan, scroll-to-zoom, and instant CAD Studio / RViz Dark mode switching.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(540, 500)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.target_x = None
        self.target_y = None
        self.show_grid = True
        self.show_labels = True
        self.show_plan = True
        self.show_obstacles = True
        self.view_mode = "CAD"  # "CAD" (Light) or "RVIZ" (Dark)
        self.lidar_angle = 0.0

        # Pan and Zoom state
        self.zoom_level = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.is_panning = False
        self.last_mouse_pos = None

        # Real Nav2 path & multi-hop route
        self.nav_path = []
        self.active_route = []
        self.active_waypoint_idx = 0

        # Occupancy maps
        self.pixmap_cad = None
        self.pixmap_rviz = None
        self._init_occupancy_maps()

        # LiDAR animation timer
        self.anim_timer = QTimer(self)
        self.anim_timer.timeout.connect(self._animate_lidar)
        self.anim_timer.start(50)

    def _init_occupancy_maps(self):
        pkg_root = Path(__file__).resolve().parent.parent
        pgm_path = pkg_root / 'maps/map.pgm'
        if not pgm_path.is_file():
            pgm_path = Path.cwd() / 'src/turtlebot4_voice_navigation/maps/map.pgm'

        if pgm_path.is_file():
            raw_img = QImage(str(pgm_path))
            if not raw_img.isNull():
                # 1. CAD Studio Color Table (Crisp clean white floors, deep slate walls & obstacles)
                indexed_cad = raw_img.convertToFormat(QImage.Format_Indexed8)
                cad_table = []
                for i in range(256):
                    if i <= 50:
                        cad_table.append(QColor(15, 23, 42, 255).rgb())      # Charcoal walls & obstacles
                    elif i >= 240:
                        cad_table.append(QColor(255, 255, 255, 255).rgb())  # Pure White floor
                    else:
                        cad_table.append(QColor(241, 245, 249, 255).rgb())  # Soft boundary gray
                indexed_cad.setColorTable(cad_table)
                self.pixmap_cad = QPixmap.fromImage(indexed_cad)

                # 2. RViz Dark Grid Color Table (Deep obsidian floor, glowing silver/cyan walls)
                indexed_rviz = raw_img.convertToFormat(QImage.Format_Indexed8)
                rviz_table = []
                for i in range(256):
                    if i <= 50:
                        rviz_table.append(QColor(241, 245, 249, 255).rgb())  # Bright walls / obstacles
                    elif i >= 240:
                        rviz_table.append(QColor(15, 23, 42, 255).rgb())     # Dark floor
                    else:
                        rviz_table.append(QColor(3, 7, 18, 255).rgb())       # Deep obsidian
                indexed_rviz.setColorTable(rviz_table)
                self.pixmap_rviz = QPixmap.fromImage(indexed_rviz)

    def toggle_view_mode(self):
        self.view_mode = "RVIZ" if self.view_mode == "CAD" else "CAD"
        self.update()
        return self.view_mode

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

    def set_nav_path(self, points):
        self.nav_path = points
        self.update()

    def clear_nav_path(self):
        self.nav_path.clear()
        self.update()

    def set_active_route(self, route, current_idx=0):
        self.active_route = route
        self.active_waypoint_idx = current_idx
        self.update()

    def zoom_in(self):
        self.zoom_level = min(3.0, self.zoom_level * 1.2)
        self.update()

    def zoom_out(self):
        self.zoom_level = max(0.5, self.zoom_level / 1.2)
        self.update()

    def reset_view(self):
        self.zoom_level = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.target_x = None
        self.target_y = None
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_panning = True
            self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.is_panning and self.last_mouse_pos is not None:
            delta = event.pos() - self.last_mouse_pos
            self.pan_x += delta.x()
            self.pan_y += delta.y()
            self.last_mouse_pos = event.pos()
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.is_panning = False
            self.last_mouse_pos = None

    def wheelEvent(self, event):
        angle = event.angleDelta().y()
        if angle > 0:
            self.zoom_level = min(3.2, self.zoom_level * 1.12)
        else:
            self.zoom_level = max(0.5, self.zoom_level / 1.12)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.SmoothPixmapTransform)

        w = self.width()
        h = self.height()
        is_dark = (self.view_mode == "RVIZ")

        # 1. Canvas Background Fill
        bg_col = QColor("#090d16") if is_dark else QColor("#ffffff")
        p.fillRect(0, 0, w, h, bg_col)

        # Coordinate transformation (World physical coordinates [-7.0, -5.5] to [+7.0, +5.5])
        side = min(w, h) - 40
        scale = (side / 14.0) * self.zoom_level
        cx = w / 2.0 + self.pan_x
        cy = h / 2.0 + self.pan_y

        def to_screen(x, y):
            return cx + x * scale, cy - y * scale

        # 2. Architectural / RViz Coordinate Grid (1.0m intervals)
        if self.show_grid:
            grid_pen = QPen(QColor(30, 41, 59, 120) if is_dark else QColor(241, 245, 249), 1)
            p.setPen(grid_pen)
            for m in range(-7, 8):
                gx0, gy0 = to_screen(m, -5.5)
                gx1, gy1 = to_screen(m, 5.5)
                p.drawLine(QPointF(gx0, gy0), QPointF(gx1, gy1))
                gx0, gy0 = to_screen(-7.0, m)
                gx1, gy1 = to_screen(7.0, m)
                p.drawLine(QPointF(gx0, gy0), QPointF(gx1, gy1))

            # Major axes X and Y with coordinate ticks
            axis_pen = QPen(QColor(51, 65, 85, 180) if is_dark else QColor(226, 232, 240), 1.5)
            p.setPen(axis_pen)
            ax0, ay0 = to_screen(0, -5.5)
            ax1, ay1 = to_screen(0, 5.5)
            p.drawLine(QPointF(ax0, ay0), QPointF(ax1, ay1))
            ax0, ay0 = to_screen(-7.0, 0)
            ax1, ay1 = to_screen(7.0, 0)
            p.drawLine(QPointF(ax0, ay0), QPointF(ax1, ay1))

        # 3. Draw Real Occupancy Grid Raster (map.pgm scaled physically)
        active_pixmap = self.pixmap_rviz if is_dark else self.pixmap_cad
        if active_pixmap and not active_pixmap.isNull():
            sx0, sy0 = to_screen(-7.0, 5.5)
            sx1, sy1 = to_screen(7.0, -5.5)
            map_rect = QRectF(sx0, sy0, sx1 - sx0, sy1 - sy0)
            p.drawPixmap(map_rect.toRect(), active_pixmap)

        # 4. Draw 5 Semantic Zone Polygons with Translucent Tint
        for zid, z in ZONE_CFG.items():
            min_x, min_y, max_x, max_y = z['bounds']
            sx0, sy0 = to_screen(min_x, max_y)
            sx1, sy1 = to_screen(max_x, min_y)
            zrect = QRectF(sx0, sy0, sx1 - sx0, sy1 - sy0)

            # Translucent fill
            fill_col = QColor(z['border'].red(), z['border'].green(), z['border'].blue(), 30 if is_dark else 25)
            p.setPen(QPen(z['border'], 1.5, Qt.SolidLine if zid != 'E' else Qt.DashLine))
            p.setBrush(QBrush(fill_col))
            p.drawRoundedRect(zrect, 8, 8)

        # 5. Draw Nav2 Live Global Plan Path (/plan)
        if self.show_plan and len(self.nav_path) >= 2:
            # Outer cyan glow
            p.setPen(QPen(QColor(6, 182, 212, 110), 6, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            p.setBrush(Qt.NoBrush)
            path_glow = QPainterPath()
            first = True
            for wx, wy in self.nav_path:
                px, py = to_screen(wx, wy)
                if first:
                    path_glow.moveTo(px, py)
                    first = False
                else:
                    path_glow.lineTo(px, py)
            p.drawPath(path_glow)

            # Inner bright blue/cyan core line
            p.setPen(QPen(QColor(2, 132, 199, 240) if not is_dark else QColor(34, 211, 238, 240), 2.5, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            p.drawPath(path_glow)

            # Directional chevrons along the path
            for i in range(15, len(self.nav_path), 20):
                wx, wy = self.nav_path[i]
                prev_wx, prev_wy = self.nav_path[i - 5]
                px, py = to_screen(wx, wy)
                ang = math.atan2(wy - prev_wy, wx - prev_wx)
                p.setPen(QPen(QColor(255, 255, 255), 2))
                p.drawLine(QPointF(px, py), QPointF(px - 6 * math.cos(ang - 0.6), py + 6 * math.sin(ang - 0.6)))
                p.drawLine(QPointF(px, py), QPointF(px - 6 * math.cos(ang + 0.6), py + 6 * math.sin(ang + 0.6)))

        # 6. Draw Multi-Hop Route Sequence Connections (if active_route is present)
        if len(self.active_route) >= 2:
            p.setPen(QPen(QColor(245, 158, 11, 220), 2, Qt.DashLine))
            for i in range(len(self.active_route) - 1):
                z1_id = self.active_route[i]
                z2_id = self.active_route[i+1]
                if z1_id in ZONE_CFG and z2_id in ZONE_CFG:
                    p1x, p1y = to_screen(*ZONE_CFG[z1_id]['target'])
                    p2x, p2y = to_screen(*ZONE_CFG[z2_id]['target'])
                    p.drawLine(QPointF(p1x, p1y), QPointF(p2x, p2y))

        # 7. Draw Target Waypoints & Golden Crosshair Beacons
        for zid, z in ZONE_CFG.items():
            tx, ty = z['target']
            stx, sty = to_screen(tx, ty)

            # Golden concentric rings
            p.setPen(QPen(QColor(245, 158, 11), 1.5))
            p.setBrush(QBrush(QColor(254, 240, 138, 90)))
            p.drawEllipse(QPointF(stx, sty), 10, 10)
            p.setBrush(QBrush(QColor(217, 119, 6)))
            p.drawEllipse(QPointF(stx, sty), 3.5, 3.5)
            # Target Crosshairs
            p.drawLine(QPointF(stx - 13, sty), QPointF(stx + 13, sty))
            p.drawLine(QPointF(stx, sty - 13), QPointF(stx, sty + 13))

            # If this zone is an active route stop, draw sequence badge [1], [2], [3]
            if zid in self.active_route:
                hop_num = self.active_route.index(zid) + 1
                p.setPen(QPen(QColor(255, 255, 255), 1.5))
                p.setBrush(QBrush(QColor(220, 38, 38) if hop_num == self.active_waypoint_idx + 1 else QColor(37, 99, 235)))
                p.drawEllipse(QPointF(stx + 14, sty - 14), 10, 10)
                p.setFont(QFont("Segoe UI", 8, QFont.Bold))
                p.setPen(QPen(QColor(255, 255, 255)))
                p.drawText(QRectF(stx + 4, sty - 24, 20, 20), Qt.AlignCenter, str(hop_num))

        # 8. Draw Clean Floating Badges for Zones (Positioned in open quadrant, non-overlapping)
        if self.show_labels:
            for zid, z in ZONE_CFG.items():
                bx, by = z['badge_pos']
                sbx, sby = to_screen(bx, by)

                badge_w = 175
                badge_h = 30
                badge_rect = QRectF(sbx - badge_w / 2.0, sby - badge_h / 2.0, badge_w, badge_h)

                # Soft Drop Shadow
                p.setPen(Qt.NoPen)
                p.setBrush(QBrush(QColor(0, 0, 0, 30 if is_dark else 15)))
                p.drawRoundedRect(badge_rect.adjusted(2, 2, 2, 2), 6, 6)

                # Badge Container
                badge_bg = QColor(15, 23, 42, 230) if is_dark else QColor(255, 255, 255, 240)
                p.setPen(QPen(z['border'], 1.2))
                p.setBrush(QBrush(badge_bg))
                p.drawRoundedRect(badge_rect, 6, 6)

                # Text: Line 1 = Name, Line 2 = English
                p.setFont(QFont("Segoe UI", 8, QFont.Bold))
                p.setPen(QPen(QColor(241, 245, 249) if is_dark else z['border']))
                p.drawText(QRectF(badge_rect.left() + 6, badge_rect.top() + 2, badge_w - 12, 14), Qt.AlignLeft | Qt.AlignVCenter, f"{z['icon']} {z['name']}")

                p.setFont(QFont("Segoe UI", 7))
                p.setPen(QPen(QColor(148, 163, 184)))
                p.drawText(QRectF(badge_rect.left() + 24, badge_rect.top() + 15, badge_w - 30, 12), Qt.AlignLeft | Qt.AlignVCenter, z['sub'])

        # 9. Draw Target Crosshair Marker if a single goal is dispatched
        if self.target_x is not None and self.target_y is not None:
            gx, gy = to_screen(self.target_x, self.target_y)
            p.setPen(QPen(QColor("#ef4444"), 2))
            p.setBrush(QBrush(QColor(239, 68, 68, 40)))
            p.drawEllipse(QPointF(gx, gy), 16, 16)
            p.drawLine(QPointF(gx - 20, gy), QPointF(gx + 20, gy))
            p.drawLine(QPointF(gx, gy - 20), QPointF(gx, gy + 20))

        # 10. TurtleBot 4 Robot Representation
        rx, ry = to_screen(self.robot_x, self.robot_y)
        r_rad = 14.0 * self.zoom_level

        # Pulsing radar halo
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(37, 99, 235, 45)))
        p.drawEllipse(QPointF(rx, ry), r_rad + 9, r_rad + 9)

        # Rotating LiDAR scan arc
        p.setBrush(QBrush(QColor(16, 185, 129, 35)))
        p.drawPie(QRectF(rx - 42 * self.zoom_level, ry - 42 * self.zoom_level, 84 * self.zoom_level, 84 * self.zoom_level), int(self.lidar_angle * 16), int(55 * 16))

        # Robot Chassis
        p.setPen(QPen(QColor("#3b82f6"), 2))
        p.setBrush(QBrush(QColor("#0f172a") if not is_dark else QColor("#1e293b")))
        p.drawEllipse(QPointF(rx, ry), r_rad, r_rad)

        # LiDAR puck in center
        p.setPen(QPen(QColor("#0284c7"), 1))
        p.setBrush(QBrush(QColor("#38bdf8")))
        p.drawEllipse(QPointF(rx, ry), 5 * self.zoom_level, 5 * self.zoom_level)

        # Heading vector arrow
        head_len = r_rad + 15
        hx = rx + head_len * math.cos(self.robot_yaw)
        hy = ry - head_len * math.sin(self.robot_yaw)
        p.setPen(QPen(QColor("#2563eb") if not is_dark else QColor("#38bdf8"), 3, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(rx, ry), QPointF(hx, hy))

        # Arrowhead
        arr_angle = math.atan2(ry - hy, hx - rx)
        p.setBrush(QBrush(QColor("#2563eb") if not is_dark else QColor("#38bdf8")))
        arrow_p1 = QPointF(hx - 8 * math.cos(arr_angle - 0.5), hy + 8 * math.sin(arr_angle - 0.5))
        arrow_p2 = QPointF(hx - 8 * math.cos(arr_angle + 0.5), hy + 8 * math.sin(arr_angle + 0.5))
        p.drawPolygon(QPolygonF([QPointF(hx, hy), arrow_p1, arrow_p2]))

        # Robot live label
        p.setFont(QFont("Segoe UI", 7, QFont.Bold))
        p.setPen(QPen(QColor("#0f172a") if not is_dark else QColor("#f1f5f9")))
        p.drawText(int(rx - 60), int(ry - r_rad - 18), 120, 16, Qt.AlignCenter, f"TB4 ({self.robot_x:+.2f}, {self.robot_y:+.2f})")


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
            ws_root = Path(__file__).resolve().parents[4]
            paths = [
                Path('/home/anhbang/turtlebot4_lab_ws/.cache_voice/vosk-model-small-vn'),
                ws_root / '.cache_voice/vosk-model-small-vn',
                Path.home() / 'turtlebot4_lab_ws/.cache_voice/vosk-model-small-vn',
                Path.cwd() / '.cache_voice/vosk-model-small-vn',
                Path.home() / '.cache/turtlebot4_voice/vosk-model-small-vn',
                ws_root / '.cache_voice/vosk-model-small-en-us',
            ]
            for p in paths:
                if p.is_dir():
                    try:
                        self.vosk_model = v.Model(str(p))
                        self.log(f"[AI VOSK]: Đã nạp thành công mô hình nhận dạng tiếng Việt: {p.name}")
                        break
                    except Exception as ex:
                        pass
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
            ("🧭 Đa chặng (A➔B➔C)", "Đi lần lượt từ A đến B đến C", "#2563eb", 3, 0),
            ("⚡ Xuyên sảnh (A➔E➔C)", "Đi từ phòng học qua sảnh rồi đến văn phòng", "#4f46e5", 3, 1),
            ("🔄 Vòng cung (B➔D➔E)", "Đi từ phòng lab qua thư viện rồi đến sảnh", "#059669", 4, 0),
            ("⚠️ Điểm không tồn tại", "Đi đến khu vực Z", "#dc2626", 4, 1),
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

        # Map Canvas Widget (instantiate first)
        self.map_widget = StudioMapWidget()

        # Map Viewport Header
        map_header = QHBoxLayout()
        map_header.setSpacing(8)

        lbl_map_title = QLabel("BẢN ĐỒ OCCUPANCY & TOPOLOGY 2D")
        lbl_map_title.setProperty("class", "section-header")
        map_header.addWidget(lbl_map_title)

        map_header.addStretch(1)

        self.btn_view_mode = QPushButton("🎨 Đổi: RViz Dark / CAD")
        self.btn_view_mode.setStyleSheet("""
            QPushButton { background-color: #ffffff; border: 1px solid #cbd5e1; border-radius: 5px; padding: 4px 10px; font-size: 11px; font-weight: 600; color: #334155; }
            QPushButton:hover { background-color: #f1f5f9; border-color: #64748b; }
        """)
        self.btn_view_mode.clicked.connect(self._on_toggle_view_mode)
        map_header.addWidget(self.btn_view_mode)

        self.chk_show_grid = QCheckBox("Lưới 1.0m")
        self.chk_show_grid.setChecked(True)
        self.chk_show_grid.setStyleSheet("font-size: 11px; color: #64748b;")
        self.chk_show_grid.toggled.connect(self._toggle_grid)
        map_header.addWidget(self.chk_show_grid)

        self.chk_show_plan = QCheckBox("Đường /plan")
        self.chk_show_plan.setChecked(True)
        self.chk_show_plan.setStyleSheet("font-size: 11px; color: #64748b;")
        self.chk_show_plan.toggled.connect(self._toggle_plan)
        map_header.addWidget(self.chk_show_plan)

        self.chk_show_labels = QCheckBox("Nhãn vùng")
        self.chk_show_labels.setChecked(True)
        self.chk_show_labels.setStyleSheet("font-size: 11px; color: #64748b;")
        self.chk_show_labels.toggled.connect(self._toggle_labels)
        map_header.addWidget(self.chk_show_labels)

        btn_in = QPushButton("+")
        btn_in.setFixedSize(24, 24)
        btn_in.setStyleSheet("font-weight: bold; border: 1px solid #cbd5e1; border-radius: 4px; background: white;")
        btn_in.clicked.connect(self.map_widget.zoom_in)
        map_header.addWidget(btn_in)

        btn_out = QPushButton("-")
        btn_out.setFixedSize(24, 24)
        btn_out.setStyleSheet("font-weight: bold; border: 1px solid #cbd5e1; border-radius: 4px; background: white;")
        btn_out.clicked.connect(self.map_widget.zoom_out)
        map_header.addWidget(btn_out)

        center_layout.addLayout(map_header)
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
        self.bridge.sig_plan.connect(self._on_plan_received)
        self.sig_update_vu.connect(self._on_vu_update)
        self.sig_recognized.connect(self._on_speech_transcribed)

        self.log("[KHỞI ĐỘNG]: Giao diện 3D Robotics Studio - TurtleBot 4 Lab 07 sẵn sàng.")
        self.log("[CẤU HÌNH]: 5 Khu vực phân vùng A, B, C, D, E | Nav2 Action Client kết nối.")

    def _on_plan_received(self, points):
        self.map_widget.set_nav_path(points)

    def _on_toggle_view_mode(self):
        mode = self.map_widget.toggle_view_mode()
        btn_txt = "🎨 Chế độ: RViz Dark" if mode == "RVIZ" else "🎨 Chế độ: CAD Studio"
        self.btn_view_mode.setText(btn_txt)
        self.log(f"[VIEWPORT]: Đã chuyển sang chế độ {mode}.")

    def _toggle_plan(self, checked):
        self.map_widget.show_plan = checked
        self.map_widget.update()

    def _toggle_labels(self, checked):
        self.map_widget.show_labels = checked
        self.map_widget.update()

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
            if not status_json or not status_json.strip():
                return
            data = json.loads(status_json)
            st = data.get("state", data.get("status", "IDLE"))
            msg = data.get("feedback", data.get("message", ""))
            cur_zone = data.get("current_zone", "")
            route = data.get("active_route", [])
            wp_idx = data.get("waypoint_index", 0)

            if not msg and st == "IDLE":
                msg = "Hệ thống sẵn sàng"

            badge_styles = {
                "IDLE": "background-color: #f1f5f9; color: #475569; border: 1px solid #cbd5e1;",
                "NAVIGATING": "background-color: #eff6ff; color: #1d4ed8; border: 1px solid #bfdbfe;",
                "WAYPOINT_REACHED": "background-color: #fefce8; color: #a16207; border: 1px solid #fef08a;",
                "COMPLETED": "background-color: #f0fdf4; color: #15803d; border: 1px solid #bbf7d0;",
                "CANCELLED": "background-color: #fff7ed; color: #c2410c; border: 1px solid #fed7aa;",
                "CANCELED": "background-color: #fff7ed; color: #c2410c; border: 1px solid #fed7aa;",
                "FAILED": "background-color: #fef2f2; color: #b91c1c; border: 1px solid #fecaca;",
                "REJECTED": "background-color: #fef2f2; color: #b91c1c; border: 1px solid #fecaca;",
                "QUERY_RESULT": "background-color: #f5f3ff; color: #6d28d9; border: 1px solid #ddd6fe;",
            }

            style = badge_styles.get(st, "background-color: #f1f5f9; color: #0f172a; border: 1px solid #cbd5e1;")
            badge_text = f"{st} • {msg[:36]}" if msg else st
            self.lbl_nav_badge.setText(badge_text)
            self.lbl_nav_badge.setStyleSheet(f"{style} border-radius: 5px; padding: 6px 8px; font-size: 11px; font-weight: 700;")

            if cur_zone and cur_zone != "Chưa xác định":
                self.lbl_cur_zone.setText(f"📍 {cur_zone}")

            if route:
                self.map_widget.set_active_route(route, wp_idx)

            if st in ("COMPLETED", "IDLE", "FAILED", "CANCELLED", "CANCELED", "REJECTED"):
                self.map_widget.clear_nav_path()
                if st == "COMPLETED":
                    self.map_widget.target_x = None
                    self.map_widget.target_y = None
                    self.map_widget.active_route.clear()

            if msg:
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
        s_device, n_py, v_mod, s_rec = get_audio()

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
        raw_bytes = None

        # 1. Try sounddevice high-precision stream
        if s_device is not None and n_py is not None:
            def callback(indata, frames, time_info, status):
                vol = float(n_py.linalg.norm(indata) * 10)
                level = min(100, int(vol * 5.0))
                self.sig_update_vu.emit(level)
                audio_buffer.append(indata.copy())

            try:
                with s_device.InputStream(samplerate=rate, channels=1, dtype='int16', callback=callback):
                    for _ in range(int(duration * 20)):
                        time.sleep(0.05)
                if audio_buffer:
                    raw_bytes = n_py.concatenate(audio_buffer, axis=0).tobytes()
            except Exception as ex:
                self.log(f"[CẢNH BÁO sounddevice]: {ex}. Chuyển sang thu âm ALSA arecord...")

        # 2. Seamless arecord ALSA hardware capture fallback
        if not raw_bytes:
            import subprocess
            try:
                self.log("[MICRO]: Đang thu âm trực tiếp qua arecord ALSA...")
                cmd = ["arecord", "-d", str(int(math.ceil(duration))), "-r", str(rate), "-c", "1", "-f", "S16_LE", "-t", "raw", "-q"]
                p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                raw_bytes, _ = p.communicate(timeout=duration + 1.5)
            except Exception as e_ar:
                self.log(f"[!] Lỗi thu âm arecord: {e_ar}")

        self.lbl_mic_status.setText("Trạng thái: Đang phân tích nhận dạng giọng nói...")
        self.lbl_mic_status.setStyleSheet("background-color: #fefce8; color: #854d0e; padding: 6px 10px; border-radius: 5px; font-weight: 600; border: 1px solid #fef08a;")
        self.sig_update_vu.emit(0.0)

        recognized_txt = ""
        try:
            if raw_bytes and len(raw_bytes) > 2000:
                # 1. Try offline Vosk model (Vietnamese)
                if self.vosk_model and v_mod is not None:
                    try:
                        rec = v_mod.KaldiRecognizer(self.vosk_model, rate)
                        rec.AcceptWaveform(raw_bytes)
                        res = json.loads(rec.FinalResult())
                        recognized_txt = res.get('text', '').strip()
                    except Exception as e_vosk:
                        self.log(f"[!] Lỗi Vosk: {e_vosk}")

                # 2. Try online Google Speech Recognition (Vietnamese / English)
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
