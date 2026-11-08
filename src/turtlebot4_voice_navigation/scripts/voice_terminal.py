#!/usr/bin/env python3
"""Interactive Terminal Interface for TurtleBot 4 Voice Navigation.
Allows sending spoken or typed commands, quick numerical shortcuts, and monitoring real-time Nav2 feedback.
"""
from pathlib import Path
import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


BANNER = """
========================================================================
   TURTLEBOT 4 - ĐIỀU HƯỚNG BẰNG GIỌNG NÓI & PHÂN VÙNG BẢN ĐỒ (LAB 07)
   5 KHU VỰC CHỨC NĂNG: A (Phòng học) | B (Phòng TN) | C (Văn phòng)
                        D (Thư viện)   | E (Hành lang & Sảnh)
========================================================================
  [Phím tắt nhanh]:
    1. Đi đến khu vực A (Phòng học)
    2. Đi đến phòng thí nghiệm (Khu vực B)
    3. Di chuyển từ khu vực A đến khu vực B
    4. Đi đến văn phòng (Khu vực C)
    5. Đi đến thư viện (Khu vực D)
    6. Đi từ phòng học qua hành lang rồi đến phòng thí nghiệm (Multi-waypoint)
    7. Đi đến phòng tổng thống (Kiểm tra xử lý khu vực KHÔNG tồn tại)
    8. Robot đang ở đâu? (Truy vấn vị trí hiện tại)
    9. Dừng lại (Emergency Stop)
   
  [Hoặc nhập trực tiếp bất kỳ câu lệnh bằng tiếng Việt hoặc tiếng Anh]:
    Ví dụ: 'Đi đến thư viện', 'Go to classroom', 'Di chuyển từ B sang C'
    'list' : Hiển thị tọa độ chi tiết 5 khu vực
    'q'    : Thoát
========================================================================
"""

SHORTCUTS = {
    '1': "Đi đến khu vực A",
    '2': "Đi đến phòng thí nghiệm",
    '3': "Di chuyển từ khu vực A đến khu vực B",
    '4': "Đi đến văn phòng",
    '5': "Đi đến thư viện",
    '6': "Đi từ phòng học qua hành lang rồi đến phòng thí nghiệm",
    '7': "Đi đến phòng tổng thống",
    '8': "Robot đang ở đâu?",
    '9': "Dừng lại"
}


class VoiceTerminalNode(Node):
    def __init__(self):
        super().__init__('voice_terminal_client')
        self.cmd_pub = self.create_publisher(String, '/voice/command_text', 10)
        self.status_sub = self.create_subscription(
            String,
            '/voice/navigation_status',
            self.on_status,
            10
        )
        self.audio_status_sub = self.create_subscription(
            String,
            '/voice/audio_status',
            self.on_audio_status,
            10
        )

    def on_status(self, msg: String):
        try:
            d = json.loads(msg.data)
            state = d.get('state', 'UNKNOWN')
            feedback = d.get('feedback', '')
            cur_zone = d.get('current_zone', '')
            route = d.get('active_route', [])

            color = "\033[92m" if state in ("COMPLETED", "NAVIGATING") else "\033[93m" if state in ("WAYPOINT_REACHED", "QUERY_RESULT") else "\033[91m"
            reset = "\033[0m"

            print(f"\n{color}[ROBOT STATUS - {state}]{reset} {feedback}")
            if cur_zone:
                print(f"   Vị trí hiện tại: \033[96m{cur_zone}\033[0m")
            if route:
                print(f"   Lộ trình mục tiêu: \033[94m{' -> '.join(route)}\033[0m")
            print("Nhập lệnh hoặc chọn số (1-9) > ", end="", flush=True)
        except Exception:
            pass

    def on_audio_status(self, msg: String):
        try:
            d = json.loads(msg.data)
            state = d.get('state', '')
            message = d.get('message', '')
            if state == 'RECOGNIZED':
                print(f"\n\033[95m[MIC NHẬN DIỆN]:\033[0m '{message}'")
                print("Nhập lệnh hoặc chọn số (1-9) > ", end="", flush=True)
        except Exception:
            pass

    def send_command(self, text: str):
        print(f"\033[94m[LỆNH GỬI ĐI]:\033[0m '{text}'")
        self.cmd_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = VoiceTerminalNode()
    print(BANNER)

    import threading
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            try:
                cmd = input("Nhập lệnh hoặc chọn số (1-9) > ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not cmd:
                continue
            if cmd.lower() in ('q', 'exit', 'quit'):
                break
            if cmd.lower() == 'list':
                print("""
Danh sách 5 khu vực trên bản đồ Smart Campus:
  - [A] Phòng học (Classroom)        -> Tọa độ: (-3.2, 2.6)
  - [B] Phòng thí nghiệm (Laboratory) -> Tọa độ: ( 3.2, 2.6)
  - [C] Văn phòng (Office)            -> Tọa độ: ( 3.2, -2.6)
  - [D] Thư viện (Library)            -> Tọa độ: (-3.2, -2.6)
  - [E] Hành lang & Sảnh (Corridor)   -> Tọa độ: ( 0.0, 0.0) [Vị trí xuất phát TB4]
""")
                continue

            # Check if user entered a shortcut number
            if cmd in SHORTCUTS:
                command_to_send = SHORTCUTS[cmd]
            else:
                command_to_send = cmd

            node.send_command(command_to_send)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
