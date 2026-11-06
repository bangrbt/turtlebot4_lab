#!/usr/bin/env python3
"""Interactive Terminal Interface for TurtleBot 4 Voice Navigation.
Allows sending spoken or typed commands and monitoring real-time navigation feedback.
"""
import sys
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


BANNER = """
========================================================================
   TURTLEBOT 4 - ĐIỀU HƯỚNG BẰNG GIỌNG NÓI & PHÂN VÙNG BẢN ĐỒ (LAB 07)
========================================================================
 Các câu lệnh mẫu có thể nhập hoặc nói:
   1. "Đi đến khu vực A."
   2. "Đi đến phòng thí nghiệm."
   3. "Di chuyển từ khu vực A đến khu vực B."
   4. "Đi từ phòng A qua hành lang D rồi đến phòng B."
   5. "Robot đang ở đâu?"
   6. "Dừng lại"
   7. "Đi đến nhà ăn" (Thử nghiệm phòng không tồn tại)
   8. 'list' (Xem danh sách các phòng trên bản đồ)
   9. 'q' hoặc 'exit' để thoát.
========================================================================
"""

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

    def on_status(self, msg: String):
        try:
            d = json.loads(msg.data)
            state = d.get('state', 'UNKNOWN')
            feedback = d.get('feedback', '')
            cur_zone = d.get('current_zone', '')
            route = d.get('active_route', [])

            color = "\033[92m" if state in ("COMPLETED", "NAVIGATING") else "\033[93m" if state in ("WAYPOINT_REACHED", "QUERY_RESULT") else "\033[91m"
            reset = "\033[0m"

            print(f"\n{color}[FEEDBACK - {state}]{reset} {feedback}")
            if cur_zone:
                print(f"   Vị trí hiện tại: \033[96m{cur_zone}\033[0m")
            if route:
                print(f"   Lộ trình: \033[94m{' -> '.join(route)}\033[0m")
            print("Nhập lệnh > ", end="", flush=True)
        except Exception:
            pass

    def send_command(self, text: str):
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
                cmd = input("Nhập lệnh > ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not cmd:
                continue
            if cmd.lower() in ('q', 'exit', 'quit'):
                break
            if cmd.lower() == 'list':
                print("""
Danh sách các khu vực trên bản đồ Smart Campus:
  - Khu vực A: Phòng học (Classroom)        -> (x=-3.0, y=2.0)
  - Khu vực B: Phòng thí nghiệm (Laboratory) -> (x=2.8, y=2.0)
  - Khu vực C: Văn phòng (Office)            -> (x=2.8, y=-2.0)
  - Khu vực D: Hành lang & Sảnh (Corridor)   -> (x=-3.0, y=-1.8)
""")
                continue

            node.send_command(cmd)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
