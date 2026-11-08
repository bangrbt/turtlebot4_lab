#!/usr/bin/env python3
"""Interactive Graphical User Interface (GUI) for TurtleBot 4 Voice Navigation (Lab 07).
Features:
- Live Push-to-Talk and Continuous Microphone Speech Recognition with Vosk/SpeechRecognition
- Real-time Audio VU Level Meter
- 2D Visual Map Canvas with 5 Semantic Zones, Obstacles, and Live Robot Pose
- Live Speech-to-Text log, NLU intent breakdown, and Nav2 navigation status
- 9 One-click Quick Scenario Buttons covering all lab requirements
"""
import math
import os
import queue
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose

# Optional audio libraries
try:
    import sounddevice as sd
    import numpy as np
    AUDIO_AVAILABLE = True
except Exception:
    AUDIO_AVAILABLE = False


ZONE_COLORS = {
    'A': {'fill': '#1e3a5f', 'border': '#38bdf8', 'name': 'Khu A: Phòng học'},
    'B': {'fill': '#14532d', 'border': '#4ade80', 'name': 'Khu B: Phòng thí nghiệm'},
    'C': {'fill': '#7c2d12', 'border': '#fb923c', 'name': 'Khu C: Văn phòng'},
    'D': {'fill': '#4c1d95', 'border': '#c084fc', 'name': 'Khu D: Thư viện'},
    'E': {'fill': '#334155', 'border': '#94a3b8', 'name': 'Khu E: Hành lang & Sảnh'},
}

ZONE_BOUNDS = {
    'A': (-5.0, 1.3, -1.3, 5.0, -3.2, 2.6),
    'B': (1.3, 1.3, 5.0, 5.0, 3.2, 2.6),
    'C': (1.3, -5.0, 5.0, -1.3, 3.2, -2.6),
    'D': (-5.0, -5.0, -1.3, -1.3, -3.2, -2.6),
    'E': (-5.0, -1.3, 5.0, 1.3, 0.0, 0.0),
}


class VoiceGuiNode(Node):
    def __init__(self, gui_app):
        super().__init__('voice_navigation_gui_node')
        self.gui = gui_app
        self.declare_parameter('use_sim_time', True)

        self.cmd_pub = self.create_publisher(String, '/voice/command_text', 10)
        self.recognized_pub = self.create_publisher(String, '/voice/recognized_text', 10)

        self.status_sub = self.create_subscription(
            String,
            '/voice/navigation_status',
            self.on_nav_status,
            10
        )
        self.speech_sub = self.create_subscription(
            String,
            '/voice/recognized_text',
            self.on_speech_text,
            10
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.on_amcl_pose,
            qos_profile_sensor_data
        )

    def on_nav_status(self, msg: String):
        self.gui.msg_queue.put(('nav_status', msg.data))

    def on_speech_text(self, msg: String):
        self.gui.msg_queue.put(('speech_text', msg.data))

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.gui.msg_queue.put(('robot_pose', (x, y, yaw)))


class VoiceNavigationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("TurtleBot 4 - Hệ Thống Điều Hướng Bằng Giọng Nói & Bản Đồ Phân Vùng (Lab 07)")
        self.root.geometry("1120x800")
        self.root.minsize(980, 720)
        self.root.configure(bg="#0f172a")

        self.msg_queue = queue.Queue()
        self.node = None
        self.is_recording = False
        self.is_continuous = False
        self.audio_level = 0.0
        self.robot_pose = (0.0, 0.0, 0.0)
        self.target_pose = None
        self.vosk_model = None

        self._init_vosk_engine()
        self._build_ui()
        self._start_poll()

    def _init_vosk_engine(self):
        """Pre-load local Vosk offline model if available."""
        try:
            import vosk
            vosk.SetLogLevel(-1)
            paths = [
                Path.cwd() / '.cache_voice/vosk-model-small-vn',
                Path.home() / '.cache/turtlebot4_voice/vosk-model-small-vn',
                Path.cwd() / '.cache_voice/vosk-model-small-en-us',
            ]
            for p in paths:
                if p.is_dir():
                    self.vosk_model = vosk.Model(str(p))
                    break
        except Exception:
            self.vosk_model = None

    def _build_ui(self):
        # 1. Top Header Banner
        header = tk.Frame(self.root, bg="#1e293b", pady=10, padx=16)
        header.pack(fill=tk.X, side=tk.TOP)

        title = tk.Label(
            header,
            text="TURTLEBOT 4 - ĐIỀU HƯỚNG BẰNG GIỌNG NÓI & PHÂN VÙNG BẢN ĐỒ",
            font=("Helvetica", 16, "bold"),
            fg="#38bdf8",
            bg="#1e293b"
        )
        title.pack(anchor=tk.W)

        subtitle = tk.Label(
            header,
            text="Bài thực hành 7: Phân đoạn Map 2D (5 Khu vực A, B, C, D, E) | Nhận diện giọng nói tiếng Việt | Nav2 Autonomous Navigation",
            font=("Helvetica", 10),
            fg="#94a3b8",
            bg="#1e293b"
        )
        subtitle.pack(anchor=tk.W)

        # 2. Main Content (Left: Voice & Logs; Right: Map & Robot Pose)
        main_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg="#0f172a", sashwidth=4)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=12, pady=10)

        # --- LEFT PANEL: Voice Controls & Logs ---
        left_frame = tk.Frame(main_pane, bg="#1e293b", padx=12, pady=12)
        main_pane.add(left_frame, minsize=420)

        # Section: Voice Controls
        voice_lbl = tk.Label(left_frame, text="🎙️ ĐIỀU KHIỂN GIỌNG NÓI (VOICE INTERACTION)", font=("Helvetica", 12, "bold"), fg="#38bdf8", bg="#1e293b")
        voice_lbl.pack(anchor=tk.W, pady=(0, 6))

        # Push to Talk Big Button
        self.btn_ptt = tk.Button(
            left_frame,
            text="🎙️  BẤM ĐỂ NÓI (PUSH TO TALK)",
            font=("Helvetica", 13, "bold"),
            bg="#0284c7",
            fg="white",
            activebackground="#0369a1",
            activeforeground="white",
            relief=tk.RAISED,
            bd=3,
            pady=10,
            cursor="hand2",
            command=self.on_ptt_click
        )
        self.btn_ptt.pack(fill=tk.X, pady=6)

        # Real-time Audio VU Level Meter
        vu_box = tk.Frame(left_frame, bg="#1e293b")
        vu_box.pack(fill=tk.X, pady=4)
        tk.Label(vu_box, text="Âm lượng Micro:", font=("Helvetica", 9), fg="#cbd5e1", bg="#1e293b").pack(side=tk.LEFT)
        self.vu_canvas = tk.Canvas(vu_box, height=16, bg="#0f172a", highlightthickness=1, highlightbackground="#334155")
        self.vu_canvas.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(8, 0))

        # Toggle Continuous Mic
        self.continuous_var = tk.BooleanVar(value=False)
        self.chk_continuous = tk.Checkbutton(
            left_frame,
            text="🟢 Bật Micro tự động nghe liên tục (Continuous Listening)",
            font=("Helvetica", 10),
            fg="#f1f5f9",
            bg="#1e293b",
            selectcolor="#0f172a",
            activebackground="#1e293b",
            activeforeground="#38bdf8",
            variable=self.continuous_var,
            command=self.on_toggle_continuous
        )
        self.chk_continuous.pack(anchor=tk.W, pady=6)

        # Recognition Status Banner
        self.lbl_mic_status = tk.Label(
            left_frame,
            text="Trạng thái: Sẵn sàng nghe câu lệnh giọng nói...",
            font=("Helvetica", 10, "italic"),
            fg="#4ade80",
            bg="#0f172a",
            padx=8,
            pady=6,
            relief=tk.SUNKEN,
            bd=1
        )
        self.lbl_mic_status.pack(fill=tk.X, pady=6)

        # Quick Scenario Buttons Title
        quick_lbl = tk.Label(left_frame, text="⚡ CÂU LỆNH MẪU & PHÍM TẮT NHANH (BÀI 7)", font=("Helvetica", 11, "bold"), fg="#fbbf24", bg="#1e293b")
        quick_lbl.pack(anchor=tk.W, pady=(12, 4))

        # Grid of Quick Buttons
        btn_grid = tk.Frame(left_frame, bg="#1e293b")
        btn_grid.pack(fill=tk.X, pady=2)

        buttons = [
            ("1. Đến Phòng học (Khu A)", "Đi đến khu vực A", "#0284c7"),
            ("2. Đến Phòng TN (Khu B)", "Đi đến phòng thí nghiệm", "#16a34a"),
            ("3. Đi từ A đến B (A -> B)", "Di chuyển từ khu vực A đến khu vực B", "#ea580c"),
            ("4. Đến Văn phòng (Khu C)", "Đi đến văn phòng", "#d97706"),
            ("5. Đến Thư viện (Khu D)", "Đi đến thư viện", "#9333ea"),
            ("6. A qua E đến B (Multi-hop)", "Đi từ phòng học qua hành lang rồi đến phòng thí nghiệm", "#0d9488"),
            ("7. Phòng tổng thống (Lỗi khu vực)", "Đi đến phòng tổng thống", "#dc2626"),
            ("8. Robot đang ở đâu?", "Robot đang ở đâu?", "#475569"),
            ("🛑 DỪNG KHẨN CẤP", "Dừng lại", "#b91c1c")
        ]

        for i, (btn_text, cmd_val, col) in enumerate(buttons):
            row, col_idx = divmod(i, 2)
            if i == len(buttons) - 1:  # Emergency stop full width
                b = tk.Button(
                    btn_grid,
                    text=btn_text,
                    font=("Helvetica", 9, "bold"),
                    bg=col,
                    fg="white",
                    activebackground="#991b1b",
                    activeforeground="white",
                    relief=tk.RAISED,
                    bd=2,
                    pady=4,
                    command=lambda c=cmd_val: self.send_text_command(c)
                )
                b.grid(row=row, column=0, columnspan=2, sticky="ew", padx=2, pady=3)
            else:
                b = tk.Button(
                    btn_grid,
                    text=btn_text,
                    font=("Helvetica", 9),
                    bg=col,
                    fg="white",
                    activebackground="#334155",
                    activeforeground="white",
                    relief=tk.RAISED,
                    bd=2,
                    pady=3,
                    command=lambda c=cmd_val: self.send_text_command(c)
                )
                b.grid(row=row, column=col_idx, sticky="ew", padx=2, pady=2)
        btn_grid.columnconfigure(0, weight=1)
        btn_grid.columnconfigure(1, weight=1)

        # Custom Voice/Text Input field
        input_box = tk.Frame(left_frame, bg="#1e293b")
        input_box.pack(fill=tk.X, pady=(10, 4))
        self.entry_cmd = tk.Entry(input_box, font=("Helvetica", 10), bg="#0f172a", fg="white", insertbackground="white")
        self.entry_cmd.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
        self.entry_cmd.bind("<Return>", lambda e: self.on_custom_send())

        btn_send = tk.Button(input_box, text="Gửi lệnh", font=("Helvetica", 9, "bold"), bg="#38bdf8", fg="#0f172a", command=self.on_custom_send)
        btn_send.pack(side=tk.RIGHT)

        # Log Text Box
        log_lbl = tk.Label(left_frame, text="📜 NHẬT KÝ ĐIỀU HƯỚNG & NHẬN DẠNG:", font=("Helvetica", 10, "bold"), fg="#94a3b8", bg="#1e293b")
        log_lbl.pack(anchor=tk.W, pady=(8, 2))

        self.txt_log = tk.Text(left_frame, height=9, bg="#0f172a", fg="#e2e8f0", font=("Consolas", 9), relief=tk.SUNKEN, bd=1)
        self.txt_log.pack(fill=tk.BOTH, expand=True)

        # --- RIGHT PANEL: Live Map & Robot Tracking ---
        right_frame = tk.Frame(main_pane, bg="#1e293b", padx=12, pady=12)
        main_pane.add(right_frame, minsize=520)

        map_lbl = tk.Label(right_frame, text="🗺️ BẢN ĐỒ 2D PHÂN VÙNG VÀ ĐỊNH VỊ ROBOT (LIVE)", font=("Helvetica", 12, "bold"), fg="#38bdf8", bg="#1e293b")
        map_lbl.pack(anchor=tk.W, pady=(0, 6))

        # Map Canvas (Square aspect ratio for -6m to +6m coordinate space)
        self.map_canvas = tk.Canvas(right_frame, bg="#0b1120", highlightthickness=2, highlightbackground="#334155")
        self.map_canvas.pack(fill=tk.BOTH, expand=True, pady=4)
        self.map_canvas.bind("<Configure>", lambda e: self.draw_map())

        # Live Robot Telemetry Bar
        self.telemetry_bar = tk.Label(
            right_frame,
            text="Định vị AMCL: Chờ khởi tạo... | Vị trí: (0.00, 0.00) | Khu vực: Chờ xác định",
            font=("Helvetica", 10, "bold"),
            fg="#38bdf8",
            bg="#0f172a",
            padx=10,
            pady=8,
            relief=tk.SUNKEN,
            bd=1
        )
        self.telemetry_bar.pack(fill=tk.X, pady=(6, 0))

    def on_custom_send(self):
        cmd = self.entry_cmd.get().strip()
        if cmd:
            self.send_text_command(cmd)
            self.entry_cmd.delete(0, tk.END)

    def send_text_command(self, text):
        self.log(f"[LỆNH GỬI]: '{text}'")
        if self.node:
            self.node.cmd_pub.publish(String(data=text))

    def on_ptt_click(self):
        if self.is_recording:
            return
        threading.Thread(target=self._record_and_recognize_worker, daemon=True).start()

    def on_toggle_continuous(self):
        self.is_continuous = self.continuous_var.get()
        if self.is_continuous:
            self.log("[MICRO]: Đã bật chế độ lắng nghe liên tục.")
            threading.Thread(target=self._continuous_mic_worker, daemon=True).start()
        else:
            self.log("[MICRO]: Đã tắt chế độ nghe liên tục.")

    def _record_and_recognize_worker(self):
        if not AUDIO_AVAILABLE:
            self.msg_queue.put(('log', '[!] Thư viện sounddevice chưa sẵn sàng. Đang chạy ở chế độ văn bản.'))
            return

        self.is_recording = True
        self.msg_queue.put(('ptt_state', True))
        sample_rate = 16000
        duration = 3.5

        try:
            self.msg_queue.put(('mic_status', ("Đang lắng nghe... HÃY NÓI CÂU LỆNH NGAY BÂY GIỜ!", "#f87171")))
            audio_buffer = []

            def audio_callback(indata, frames, time_info, status):
                volume_norm = float(np.linalg.norm(indata) * 10)
                self.audio_level = min(1.0, volume_norm / 15.0)
                audio_buffer.append(indata.copy())

            with sd.InputStream(samplerate=sample_rate, channels=1, dtype='int16', callback=audio_callback):
                for _ in range(int(duration * 20)):
                    time.sleep(0.05)
                    self.msg_queue.put(('vu', self.audio_level))

            self.msg_queue.put(('mic_status', ("Đang xử lý nhận diện giọng nói...", "#fbbf24")))
            self.audio_level = 0.0
            self.msg_queue.put(('vu', 0.0))

            if audio_buffer and self.vosk_model:
                audio_data = np.concatenate(audio_buffer, axis=0).tobytes()
                import vosk
                rec = vosk.KaldiRecognizer(self.vosk_model, sample_rate)
                rec.AcceptWaveform(audio_data)
                res = json.loads(rec.FinalResult())
                text = res.get('text', '').strip()
                if text:
                    self.msg_queue.put(('recognized_speech', text))
                else:
                    self.msg_queue.put(('mic_status', ("Không nhận rõ giọng nói. Vui lòng nói to hoặc bấm lại!", "#f87171")))
            else:
                self.msg_queue.put(('mic_status', ("Đã ghi âm xong. Chờ NLU xử lý...", "#38bdf8")))

        except Exception as e:
            self.msg_queue.put(('log', f"[Lỗi ghi âm]: {e}"))
            self.msg_queue.put(('mic_status', (f"Lỗi micro: {e}", "#f87171")))
        finally:
            self.is_recording = False
            self.msg_queue.put(('ptt_state', False))

    def _continuous_mic_worker(self):
        if not AUDIO_AVAILABLE or not self.vosk_model:
            return
        sample_rate = 16000
        import vosk
        rec = vosk.KaldiRecognizer(self.vosk_model, sample_rate)

        def cb(indata, frames, time_info, status):
            if not self.is_continuous or self.is_recording:
                return
            volume_norm = float(np.linalg.norm(indata) * 10)
            level = min(1.0, volume_norm / 15.0)
            self.msg_queue.put(('vu', level))
            if rec.AcceptWaveform(indata.tobytes()):
                res = json.loads(rec.Result())
                txt = res.get('text', '').strip()
                if txt:
                    self.msg_queue.put(('recognized_speech', txt))

        try:
            with sd.InputStream(samplerate=sample_rate, channels=1, dtype='int16', callback=cb):
                while self.is_continuous:
                    time.sleep(0.1)
        except Exception as err:
            self.msg_queue.put(('log', f"[Continuous Mic Error]: {err}"))
        finally:
            self.msg_queue.put(('vu', 0.0))

    def _start_poll(self):
        """Processes messages from ROS and background audio threads into Tkinter main thread."""
        try:
            while not self.msg_queue.empty():
                msg_type, data = self.msg_queue.get_nowait()
                if msg_type == 'log':
                    self.log(data)
                elif msg_type == 'ptt_state':
                    if data:
                        self.btn_ptt.configure(text="🔴  ĐANG LẮNG NGHE... (HÃY NÓI!)", bg="#dc2626")
                    else:
                        self.btn_ptt.configure(text="🎙️  BẤM ĐỂ NÓI (PUSH TO TALK)", bg="#0284c7")
                elif msg_type == 'mic_status':
                    text, color = data
                    self.lbl_mic_status.configure(text=f"Trạng thái: {text}", fg=color)
                elif msg_type == 'vu':
                    self.draw_vu_meter(data)
                elif msg_type == 'recognized_speech':
                    self.lbl_mic_status.configure(text=f"Đã nhận diện: '{data}'", fg="#4ade80")
                    self.log(f"[MIC NHẬN DIỆN]: '{data}'")
                    if self.node:
                        self.node.cmd_pub.publish(String(data=data))
                elif msg_type == 'nav_status':
                    self.handle_nav_status(data)
                elif msg_type == 'speech_text':
                    self.log(f"[HỆ THỐNG TIẾP NHẬN]: {data}")
                elif msg_type == 'robot_pose':
                    self.robot_pose = data
                    self.update_telemetry()
                    self.draw_map()
        except Exception:
            pass

        self.root.after(40, self._start_poll)

    def handle_nav_status(self, raw_json):
        try:
            d = json.loads(raw_json)
            state = d.get('state', 'UNKNOWN')
            msg = d.get('message', '')
            cur_zone = d.get('current_zone', 'Chưa rõ')
            cur_step = d.get('current_step', '')
            total_steps = d.get('total_steps', 1)
            step_idx = d.get('step_index', 0)

            state_color = "#38bdf8"
            if state == 'NAVIGATING':
                state_color = "#fbbf24"
            elif state == 'SUCCEEDED':
                state_color = "#4ade80"
            elif state == 'FAILED':
                state_color = "#f87171"

            self.log(f"[{state}]: {msg}")
            self.lbl_mic_status.configure(text=f"Robot: {msg}", fg=state_color)
            self.update_telemetry(cur_zone=cur_zone, status_msg=msg)
        except Exception:
            self.log(f"[NAV STATUS]: {raw_json}")

    def update_telemetry(self, cur_zone=None, status_msg=None):
        x, y, yaw = self.robot_pose
        zone_str = cur_zone or self.determine_zone(x, y)
        deg = math.degrees(yaw)
        self.telemetry_bar.configure(
            text=f"Định vị AMCL: ACTIVE | Tọa độ: ({x:.2f}, {y:.2f}) | Góc quay: {deg:.0f}° | Khu vực: {zone_str}"
        )

    def determine_zone(self, x, y):
        for zid, (min_x, min_y, max_x, max_y, _, _) in ZONE_BOUNDS.items():
            if min_x <= x <= max_x and min_y <= y <= max_y:
                return ZONE_COLORS[zid]['name']
        return "Sảnh / Hành lang (Khu E)"

    def draw_vu_meter(self, level):
        self.vu_canvas.delete("all")
        w = self.vu_canvas.winfo_width()
        h = self.vu_canvas.winfo_height()
        bar_w = int(w * max(0.0, min(1.0, level)))
        col = "#22c55e" if level < 0.6 else ("#eab308" if level < 0.85 else "#ef4444")
        if bar_w > 0:
            self.vu_canvas.create_rectangle(0, 0, bar_w, h, fill=col, outline="")

    def draw_map(self):
        c = self.map_canvas
        c.delete("all")
        w = c.winfo_width()
        h = c.winfo_height()
        if w < 100 or h < 100:
            return

        # Coordinate transform: ROS world is (-6.0, -6.0) to (+6.0, +6.0)
        # Margin of 25px
        pad = 25
        side = min(w, h) - 2 * pad
        cx = w / 2.0
        cy = h / 2.0
        scale = side / 12.0  # pixels per meter

        def to_canvas(x, y):
            px = cx + x * scale
            py = cy - y * scale  # Invert Y for canvas
            return px, py

        # Draw outer bounding perimeter
        x0, y0 = to_canvas(-5.8, 5.8)
        x1, y1 = to_canvas(5.8, -5.8)
        c.create_rectangle(x0, y0, x1, y1, fill="#020617", outline="#475569", width=3)

        # Draw 5 Semantic Zones
        for zid, (min_x, min_y, max_x, max_y, target_x, target_y) in ZONE_BOUNDS.items():
            cfg = ZONE_COLORS[zid]
            px0, py0 = to_canvas(min_x, max_y)
            px1, py1 = to_canvas(max_x, min_y)
            c.create_rectangle(px0, py0, px1, py1, fill=cfg['fill'], outline=cfg['border'], width=2)

            # Zone label at target coordinate
            tx, ty = to_canvas(target_x, target_y)
            c.create_text(tx, ty - 12, text=cfg['name'], fill="#f8fafc", font=("Helvetica", 10, "bold"))
            c.create_text(tx, ty + 10, text=f"({target_x:+.1f}, {target_y:+.1f})", fill=cfg['border'], font=("Consolas", 8))

            # Target waypoint flag
            c.create_oval(tx - 4, ty - 4, tx + 4, ty + 4, fill="#facc15", outline="white")

        # Draw Center Dividers (Campus Walls)
        for wx0, wy0, wx1, wy1 in [
            (-5.0, 1.2, -1.2, 1.2), (1.2, 1.2, 5.0, 1.2),
            (-5.0, -1.2, -1.2, -1.2), (1.2, -1.2, 5.0, -1.2),
            (-1.2, 1.2, -1.2, 5.0), (1.2, 1.2, 1.2, 5.0),
            (-1.2, -1.2, -1.2, -5.0), (1.2, -1.2, 1.2, -5.0),
        ]:
            c0, c1 = to_canvas(wx0, wy0)
            c2, c3 = to_canvas(wx1, wy1)
            c.create_line(c0, c1, c2, c3, fill="#64748b", width=3)

        # Draw Robot Position (TurtleBot 4 Circular marker + Heading Arrow)
        rx, ry, ryaw = self.robot_pose
        rcx, rcy = to_canvas(rx, ry)
        r_radius = max(8, int(0.25 * scale))

        # Heading vector
        hx = rcx + (r_radius + 10) * math.cos(ryaw)
        hy = rcy - (r_radius + 10) * math.sin(ryaw)

        c.create_oval(rcx - r_radius, rcy - r_radius, rcx + r_radius, rcy + r_radius, fill="#38bdf8", outline="white", width=2)
        c.create_line(rcx, rcy, hx, hy, fill="#f43f5e", width=3, arrow=tk.LAST)
        c.create_text(rcx, rcy - r_radius - 8, text="TURTLEBOT 4", fill="#38bdf8", font=("Helvetica", 8, "bold"))

    def log(self, text):
        t_str = time.strftime("[%H:%M:%S] ")
        self.txt_log.insert(tk.END, t_str + text + "\n")
        self.txt_log.see(tk.END)


def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    gui = VoiceNavigationGUI(root)
    node = VoiceGuiNode(gui)
    gui.node = node

    # ROS 2 executor thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    def on_closing():
        gui.is_continuous = False
        gui.is_recording = False
        root.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == '__main__':
    main()
