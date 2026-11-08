#!/usr/bin/env python3
"""Interactive Graphical User Interface (GUI) for TurtleBot 4 Voice Navigation (Lab 07).
Built with PyQt5 for rock-solid stability, modern UI aesthetics, real-time map rendering,
and seamless ROS 2 Humble integration.
"""
import json
import math
from pathlib import Path
import queue
import sys
import threading
import time

from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, pyqtSignal, QObject
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QBrush, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QLineEdit, QTextEdit, QProgressBar,
    QCheckBox, QGridLayout, QFrame, QSizePolicy
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
    'A': {'name': 'Khu A: Phòng học', 'fill': QColor(30, 58, 95, 200), 'border': QColor(56, 189, 248), 'bounds': (-5.8, 0.8, -0.15, 4.4), 'target': (-2.2, 2.0)},
    'B': {'name': 'Khu B: Phòng TN', 'fill': QColor(20, 83, 45, 200), 'border': QColor(74, 222, 128), 'bounds': (0.15, 0.8, 5.8, 4.4), 'target': (2.2, 2.0)},
    'C': {'name': 'Khu C: Văn phòng', 'fill': QColor(124, 45, 18, 200), 'border': QColor(251, 146, 60), 'bounds': (0.15, -4.4, 5.8, -0.8), 'target': (2.2, -2.0)},
    'D': {'name': 'Khu D: Thư viện', 'fill': QColor(76, 29, 149, 200), 'border': QColor(192, 132, 252), 'bounds': (-5.8, -4.4, -0.15, -0.8), 'target': (-2.2, -2.0)},
    'E': {'name': 'Khu E: Hành lang & Sảnh', 'fill': QColor(51, 65, 85, 180), 'border': QColor(148, 163, 184), 'bounds': (-5.8, -0.8, 5.8, 0.8), 'target': (0.0, 0.0)},
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


class MapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(460, 460)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.target_x = None
        self.target_y = None

    def set_robot_pose(self, x, y, yaw):
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        self.update()

    def set_target(self, x, y):
        self.target_x = x
        self.target_y = y
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()
        p.fillRect(0, 0, w, h, QColor("#020617"))

        pad = 25
        side = min(w, h) - 2 * pad
        cx = w / 2.0
        cy = h / 2.0
        scale = side / 12.0  # -6m to +6m span

        def to_screen(x, y):
            return cx + x * scale, cy - y * scale

        # Draw outer boundary
        ox0, oy0 = to_screen(-5.8, 5.8)
        ox1, oy1 = to_screen(5.8, -5.8)
        p.setPen(QPen(QColor("#475569"), 3))
        p.setBrush(QBrush(QColor("#0b1120")))
        p.drawRect(QRectF(ox0, oy0, ox1 - ox0, oy1 - oy0))

        # Draw 5 Semantic Zones
        for zid, z in ZONE_CFG.items():
            min_x, min_y, max_x, max_y = z['bounds']
            sx0, sy0 = to_screen(min_x, max_y)
            sx1, sy1 = to_screen(max_x, min_y)
            p.setPen(QPen(z['border'], 2))
            p.setBrush(QBrush(z['fill']))
            p.drawRect(QRectF(sx0, sy0, sx1 - sx0, sy1 - sy0))

            # Label
            tx, ty = z['target']
            stx, sty = to_screen(tx, ty)
            p.setPen(QPen(QColor("#ffffff")))
            p.setFont(QFont("Helvetica", 9, QFont.Bold))
            p.drawText(int(stx - 65), int(sty - 8), 130, 20, Qt.AlignCenter, z['name'])
            p.setFont(QFont("Consolas", 8))
            p.setPen(QPen(z['border']))
            p.drawText(int(stx - 50), int(sty + 10), 100, 15, Qt.AlignCenter, f"({tx:+.1f}, {ty:+.1f})")

            # Target waypoint dot
            p.setPen(QPen(Qt.white, 1))
            p.setBrush(QBrush(QColor("#facc15")))
            p.drawEllipse(QPointF(stx, sty), 4, 4)

        # Draw Internal Corridor Walls and Doorways
        walls = [
            (-5.8, 0.8, -3.8, 0.8), (-2.6, 0.8, 2.6, 0.8), (3.8, 0.8, 5.8, 0.8),
            (-5.8, -0.8, -3.8, -0.8), (-2.6, -0.8, 2.6, -0.8), (3.8, -0.8, 5.8, -0.8),
            (0.0, 0.8, 0.0, 4.4), (0.0, -0.8, 0.0, -4.4),
        ]
        p.setPen(QPen(QColor("#94a3b8"), 3))
        for wx0, wy0, wx1, wy1 in walls:
            px0, py0 = to_screen(wx0, wy0)
            px1, py1 = to_screen(wx1, wy1)
            p.drawLine(QPointF(px0, py0), QPointF(px1, py1))

        # Draw Target Goal Marker if set
        if self.target_x is not None and self.target_y is not None:
            gx, gy = to_screen(self.target_x, self.target_y)
            p.setPen(QPen(QColor("#ef4444"), 2))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(gx, gy), 12, 12)
            p.drawLine(QPointF(gx - 16, gy), QPointF(gx + 16, gy))
            p.drawLine(QPointF(gx, gy - 16), QPointF(gx, gy + 16))

        # Draw TurtleBot 4 Robot Marker
        rx, ry = to_screen(self.robot_x, self.robot_y)
        r_rad = max(9.0, 0.22 * scale)
        p.setPen(QPen(Qt.white, 2))
        p.setBrush(QBrush(QColor("#38bdf8")))
        p.drawEllipse(QPointF(rx, ry), r_rad, r_rad)

        # Orientation Arrow
        head_len = r_rad + 12
        hx = rx + head_len * math.cos(self.robot_yaw)
        hy = ry - head_len * math.sin(self.robot_yaw)
        p.setPen(QPen(QColor("#f43f5e"), 3))
        p.drawLine(QPointF(rx, ry), QPointF(hx, hy))

        p.setFont(QFont("Helvetica", 8, QFont.Bold))
        p.setPen(QPen(QColor("#38bdf8")))
        p.drawText(int(rx - 45), int(ry - r_rad - 16), 90, 15, Qt.AlignCenter, "TURTLEBOT 4")


class VoiceNavigationMainWindow(QMainWindow):
    sig_update_vu = pyqtSignal(float)
    sig_recognized = pyqtSignal(str)

    def __init__(self, bridge: RosBridge):
        super().__init__()
        self.bridge = bridge
        self.setWindowTitle("TurtleBot 4 - Hệ Thống Điều Hướng Bằng Giọng Nói & Phân Vùng Bản Đồ (Lab 07)")
        self.resize(1180, 820)
        self.setMinimumSize(1020, 720)
        self.setStyleSheet("background-color: #0f172a; color: #f8fafc;")

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
        _, _, v = get_audio()
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
        root_layout.setContentsMargins(14, 10, 14, 10)
        root_layout.setSpacing(8)

        # 1. Header Banner
        header = QFrame()
        header.setStyleSheet("background-color: #1e293b; border-radius: 6px; padding: 6px;")
        h_layout = QVBoxLayout(header)
        h_layout.setContentsMargins(12, 6, 12, 6)
        h_layout.setSpacing(2)

        lbl_title = QLabel("TURTLEBOT 4 - ĐIỀU HƯỚNG BẰNG GIỌNG NÓI & PHÂN VÙNG BẢN ĐỒ")
        lbl_title.setStyleSheet("font-size: 17px; font-weight: bold; color: #38bdf8;")
        h_layout.addWidget(lbl_title)

        lbl_sub = QLabel("Bài thực hành 7: Phân đoạn Map 2D (5 Khu vực A, B, C, D, E) | Nhận diện giọng nói tiếng Việt | Nav2 Autonomous Navigation")
        lbl_sub.setStyleSheet("font-size: 11px; color: #94a3b8;")
        h_layout.addWidget(lbl_sub)
        root_layout.addWidget(header)

        # 2. Main Content Splitter (Left: Controls & Logs; Right: Map & Robot Pose)
        content_layout = QHBoxLayout()
        content_layout.setSpacing(12)
        root_layout.addLayout(content_layout, stretch=1)

        # --- LEFT PANEL: Voice & Controls ---
        left_widget = QWidget()
        left_widget.setFixedWidth(460)
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(8)
        content_layout.addWidget(left_widget)

        # Section: Voice Controls
        voice_card = QFrame()
        voice_card.setStyleSheet("background-color: #1e293b; border-radius: 6px; padding: 10px;")
        v_card_layout = QVBoxLayout(voice_card)
        v_card_layout.setSpacing(8)

        lbl_vsec = QLabel("🎙️ ĐIỀU KHIỂN GIỌNG NÓI (VOICE INTERACTION)")
        lbl_vsec.setStyleSheet("font-size: 13px; font-weight: bold; color: #38bdf8;")
        v_card_layout.addWidget(lbl_vsec)

        self.btn_ptt = QPushButton("🎙️  BẤM ĐỂ NÓI (PUSH TO TALK)")
        self.btn_ptt.setStyleSheet("""
            QPushButton {
                background-color: #0284c7; color: white; font-size: 14px; font-weight: bold;
                border-radius: 6px; padding: 12px;
            }
            QPushButton:hover { background-color: #0369a1; }
            QPushButton:pressed { background-color: #075985; }
        """)
        self.btn_ptt.clicked.connect(self.on_ptt_clicked)
        v_card_layout.addWidget(self.btn_ptt)

        # Audio VU Level Meter
        vu_row = QHBoxLayout()
        vu_lbl = QLabel("Âm lượng Micro:")
        vu_lbl.setStyleSheet("font-size: 11px; color: #cbd5e1;")
        vu_row.addWidget(vu_lbl)

        self.vu_bar = QProgressBar()
        self.vu_bar.setRange(0, 100)
        self.vu_bar.setValue(0)
        self.vu_bar.setFixedHeight(14)
        self.vu_bar.setTextVisible(False)
        self.vu_bar.setStyleSheet("""
            QProgressBar { background-color: #0f172a; border-radius: 4px; border: 1px solid #334155; }
            QProgressBar::chunk { background-color: #22c55e; border-radius: 3px; }
        """)
        vu_row.addWidget(self.vu_bar, stretch=1)
        v_card_layout.addLayout(vu_row)

        self.chk_continuous = QCheckBox("🟢 Bật Micro tự động nghe liên tục (Continuous Listening)")
        self.chk_continuous.setStyleSheet("font-size: 11px; color: #e2e8f0; padding: 2px;")
        self.chk_continuous.toggled.connect(self.on_continuous_toggled)
        v_card_layout.addWidget(self.chk_continuous)

        self.lbl_mic_status = QLabel("Trạng thái: Sẵn sàng nghe câu lệnh giọng nói...")
        self.lbl_mic_status.setStyleSheet("background-color: #0f172a; color: #4ade80; padding: 8px; border-radius: 4px; font-style: italic; font-size: 11px;")
        v_card_layout.addWidget(self.lbl_mic_status)
        left_layout.addWidget(voice_card)

        # Section: Quick Action Buttons (Lab 07 Scenarios)
        quick_card = QFrame()
        quick_card.setStyleSheet("background-color: #1e293b; border-radius: 6px; padding: 10px;")
        q_layout = QVBoxLayout(quick_card)
        q_layout.setSpacing(6)

        lbl_qsec = QLabel("⚡ CÂU LỆNH MẪU & PHÍM TẮT NHANH (BÀI 7)")
        lbl_qsec.setStyleSheet("font-size: 12px; font-weight: bold; color: #fbbf24;")
        q_layout.addWidget(lbl_qsec)

        grid = QGridLayout()
        grid.setSpacing(6)

        actions = [
            ("1. Đến Phòng học (Khu A)", "Đi đến khu vực A", "#0284c7"),
            ("2. Đến Phòng TN (Khu B)", "Đi đến phòng thí nghiệm", "#16a34a"),
            ("3. Đi từ A đến B (A -> B)", "Di chuyển từ khu vực A đến khu vực B", "#ea580c"),
            ("4. Đến Văn phòng (Khu C)", "Đi đến văn phòng", "#d97706"),
            ("5. Đến Thư viện (Khu D)", "Đi đến thư viện", "#9333ea"),
            ("6. A qua E đến B (Multi-hop)", "Đi từ phòng học qua hành lang rồi đến phòng thí nghiệm", "#0d9488"),
            ("7. Phòng tổng thống (Lỗi khu vực)", "Đi đến phòng tổng thống", "#dc2626"),
            ("8. Robot đang ở đâu?", "Robot đang ở đâu?", "#475569"),
        ]

        for idx, (txt, cmd, col) in enumerate(actions):
            r_idx, c_idx = divmod(idx, 2)
            btn = QPushButton(txt)
            btn.setStyleSheet(f"""
                QPushButton {{ background-color: {col}; color: white; font-size: 11px; font-weight: bold; border-radius: 4px; padding: 6px; }}
                QPushButton:hover {{ opacity: 0.9; }}
            """)
            btn.clicked.connect(lambda _, c=cmd: self.send_cmd(c))
            grid.addWidget(btn, r_idx, c_idx)
        q_layout.addLayout(grid)

        # Emergency Stop Button
        btn_stop = QPushButton("🛑 DỪNG KHẨN CẤP (EMERGENCY STOP)")
        btn_stop.setStyleSheet("""
            QPushButton { background-color: #b91c1c; color: white; font-size: 12px; font-weight: bold; border-radius: 4px; padding: 8px; }
            QPushButton:hover { background-color: #991b1b; }
        """)
        btn_stop.clicked.connect(lambda: self.send_cmd("Dừng lại"))
        q_layout.addWidget(btn_stop)
        left_layout.addWidget(quick_card)

        # Custom Command Input
        input_row = QHBoxLayout()
        self.entry_cmd = QLineEdit()
        self.entry_cmd.setPlaceholderText("Hoặc nhập câu lệnh tiếng Việt/tiếng Anh...")
        self.entry_cmd.setStyleSheet("background-color: #1e293b; color: white; border: 1px solid #334155; border-radius: 4px; padding: 6px; font-size: 12px;")
        self.entry_cmd.returnPressed.connect(self.on_custom_send)
        input_row.addWidget(self.entry_cmd, stretch=1)

        btn_send = QPushButton("Gửi")
        btn_send.setStyleSheet("background-color: #38bdf8; color: #0f172a; font-weight: bold; border-radius: 4px; padding: 6px 14px;")
        btn_send.clicked.connect(self.on_custom_send)
        input_row.addWidget(btn_send)
        left_layout.addLayout(input_row)

        # Log Text Box
        self.txt_log = QTextEdit()
        self.txt_log.setReadOnly(True)
        self.txt_log.setFixedHeight(120)
        self.txt_log.setStyleSheet("background-color: #020617; color: #cbd5e1; font-family: Consolas; font-size: 10px; border-radius: 4px; border: 1px solid #334155;")
        left_layout.addWidget(self.txt_log)

        # --- RIGHT PANEL: Map & Telemetry ---
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)
        content_layout.addWidget(right_widget, stretch=1)

        lbl_msec = QLabel("🗺️ BẢN ĐỒ 2D PHÂN VÙNG VÀ ĐỊNH VỊ ROBOT (LIVE)")
        lbl_msec.setStyleSheet("font-size: 13px; font-weight: bold; color: #38bdf8;")
        right_layout.addWidget(lbl_msec)

        self.map_widget = MapWidget()
        right_layout.addWidget(self.map_widget, stretch=1)

        # Telemetry Bar
        self.lbl_telemetry = QLabel("Định vị AMCL: Chờ kết nối... | Tọa độ: (0.00, 0.00) | Khu vực: Sảnh (Khu E)")
        self.lbl_telemetry.setStyleSheet("background-color: #1e293b; color: #38bdf8; padding: 8px 12px; border-radius: 6px; font-size: 12px; font-weight: bold;")
        right_layout.addWidget(self.lbl_telemetry)

    def _wire_signals(self):
        self.bridge.sig_status.connect(self.on_ros_status)
        self.bridge.sig_speech.connect(self.on_ros_speech)
        self.bridge.sig_pose.connect(self.on_ros_pose)
        self.sig_update_vu.connect(self.on_vu_update)
        self.sig_recognized.connect(self.on_speech_recognized)

    def on_custom_send(self):
        txt = self.entry_cmd.text().strip()
        if txt:
            self.send_cmd(txt)
            self.entry_cmd.clear()

    def send_cmd(self, text: str):
        self.log(f"[LỆNH GỬI]: '{text}'")
        self.bridge.send_command(text)

    def on_ptt_clicked(self):
        if self.is_recording:
            return
        threading.Thread(target=self._record_worker, daemon=True).start()

    def on_continuous_toggled(self, checked):
        self.is_continuous = checked
        if checked:
            self.log("[MICRO]: Bật lắng nghe liên tục.")
            threading.Thread(target=self._continuous_mic_worker, daemon=True).start()
        else:
            self.log("[MICRO]: Tắt lắng nghe liên tục.")

    def _record_worker(self):
        s_device, n_py, _, _ = get_audio()
        if s_device is None or n_py is None:
            self.log("[!] Microphone chưa sẵn sàng. Bạn có thể gõ lệnh hoặc bấm nút.")
            return

        self.is_recording = True
        self.btn_ptt.setText("🔴  ĐANG LẮNG NGHE... HÃY NÓI!")
        self.btn_ptt.setStyleSheet("background-color: #dc2626; color: white; font-size: 14px; font-weight: bold; border-radius: 6px; padding: 12px;")
        self.lbl_mic_status.setText("Trạng thái: Đang ghi âm... HÃY NÓI CÂU LỆNH!")
        self.lbl_mic_status.setStyleSheet("background-color: #0f172a; color: #f87171; padding: 8px; border-radius: 4px; font-style: italic;")

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
            self.lbl_mic_status.setStyleSheet("background-color: #0f172a; color: #fbbf24; padding: 8px; border-radius: 4px; font-style: italic;")
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
                    self.lbl_mic_status.setText(f"Đã nhận diện: '{recognized_txt}'")
                    self.lbl_mic_status.setStyleSheet("background-color: #0f172a; color: #4ade80; padding: 8px; border-radius: 4px; font-weight: bold;")
                    self.sig_recognized.emit(recognized_txt)
                else:
                    self.lbl_mic_status.setText("Không nghe rõ giọng nói. Hãy nói to rõ hoặc bấm lại!")
                    self.lbl_mic_status.setStyleSheet("background-color: #0f172a; color: #f87171; padding: 8px; border-radius: 4px; font-style: italic;")
            else:
                self.lbl_mic_status.setText("Không thu nhận được âm thanh từ micro.")
                self.lbl_mic_status.setStyleSheet("background-color: #0f172a; color: #f87171; padding: 8px; border-radius: 4px; font-style: italic;")

        except Exception as e:
            self.log(f"[Lỗi Micro]: {e}")
            self.lbl_mic_status.setText(f"Lỗi Micro: {e}")
        finally:
            self.is_recording = False
            self.btn_ptt.setText("🎙️  BẤM ĐỂ NÓI (PUSH TO TALK)")
            self.btn_ptt.setStyleSheet("""
                QPushButton { background-color: #0284c7; color: white; font-size: 14px; font-weight: bold; border-radius: 6px; padding: 12px; }
                QPushButton:hover { background-color: #0369a1; }
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

    def on_vu_update(self, val):
        self.vu_bar.setValue(int(val))

    def on_speech_recognized(self, text):
        self.log(f"[MIC NHẬN DIỆN]: '{text}'")
        self.lbl_mic_status.setText(f"Đã nhận diện: '{text}'")
        self.lbl_mic_status.setStyleSheet("background-color: #0f172a; color: #4ade80; padding: 8px; border-radius: 4px; font-style: italic;")
        self.bridge.send_command(text)

    def on_ros_speech(self, text):
        self.log(f"[HỆ THỐNG TIẾP NHẬN]: {text}")

    def on_ros_status(self, raw_json):
        try:
            d = json.loads(raw_json)
            st = d.get('state', 'UNKNOWN')
            msg = d.get('message', '')
            zone = d.get('current_zone', 'Chưa rõ')
            col = "#38bdf8"
            if st == 'NAVIGATING':
                col = "#fbbf24"
            elif st == 'SUCCEEDED':
                col = "#4ade80"
            elif st == 'FAILED':
                col = "#f87171"
            self.log(f"[{st}]: {msg}")
            self.lbl_mic_status.setText(f"Robot: {msg}")
            self.lbl_mic_status.setStyleSheet(f"background-color: #0f172a; color: {col}; padding: 8px; border-radius: 4px; font-style: italic;")
        except Exception:
            self.log(f"[NAV]: {raw_json}")

    def on_ros_pose(self, x, y, yaw):
        self.map_widget.set_robot_pose(x, y, yaw)
        deg = math.degrees(yaw)
        cur_z = self._calc_zone(x, y)
        self.lbl_telemetry.setText(f"Định vị AMCL: ACTIVE | Tọa độ: ({x:.2f}, {y:.2f}) | Góc quay: {deg:.0f}° | Khu vực: {cur_z}")

    def _calc_zone(self, x, y):
        for zid, z in ZONE_CFG.items():
            min_x, min_y, max_x, max_y = z['bounds']
            if min_x <= x <= max_x and min_y <= y <= max_y:
                return z['name']
        return "Sảnh / Hành lang (Khu E)"

    def log(self, text):
        t_str = time.strftime("[%H:%M:%S] ")
        self.txt_log.append(f"<span style='color: #64748b;'>{t_str}</span> {text}")


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
