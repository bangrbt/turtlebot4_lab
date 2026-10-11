#!/usr/bin/env python3
"""Desktop hold/release keyboard teleop for the existing Create 3 /cmd_vel path."""
import argparse
import json
import signal
import time
import tkinter as tk
from tkinter import ttk
from pathlib import Path
from Xlib import display, XK
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock

KEYS = {
    'w': (1, 0), 'i': (1, 0), 'Up': (1, 0),
    's': (-1, 0), 'comma': (-1, 0), 'Down': (-1, 0),
    'a': (0, 1), 'j': (0, 1), 'Left': (0, 1),
    'd': (0, -1), 'l': (0, -1), 'Right': (0, -1),
    'u': (1, 1), 'o': (1, -1), 'm': (-1, -1), 'period': (-1, 1),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=0.25)
    parser.add_argument('--turn', type=float, default=0.6)
    parser.add_argument('--log', type=Path)
    args = parser.parse_args()
    if not 0 < args.speed <= 0.35 or not 0 < args.turn <= 1:
        parser.error('speed must be in (0, 0.35] m/s and turn in (0, 1] rad/s')
    rclpy.init()
    node = rclpy.create_node('turtlebot4_lab_hold_teleop')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        node.destroy_node()
        rclpy.shutdown()
        parser.exit(1, f'LOI GUI: {exc}. Can DISPLAY cua desktop.\n')
    root.title('TurtleBot 4 - HOLD TELEOP')
    root.geometry('600x350')
    pressed = set()
    blocked = set()
    xdisplay = display.Display()
    keycodes = {key: xdisplay.keysym_to_keycode(XK.string_to_keysym(key)) for key in KEYS}
    stopcodes = {key: xdisplay.keysym_to_keycode(XK.string_to_keysym(key)) for key in ('space', 'k', 'Escape')}
    ctrlcodes = [xdisplay.keysym_to_keycode(XK.string_to_keysym(key)) for key in ('Control_L', 'Control_R')]
    ccode = xdisplay.keysym_to_keycode(XK.string_to_keysym('c'))
    clock = {'stamp': None, 'advanced_at': 0.0}
    graph = {'checked_at': 0.0, 'ready': False, 'reason': 'Dang cho ROS graph /clock...'}
    state = {'closed': False, 'last_command': None, 'focused': False}
    speed = tk.DoubleVar(value=args.speed)
    turn = tk.DoubleVar(value=args.turn)
    message = tk.StringVar(value=graph['reason'])
    logfile = args.log.expanduser() if args.log else None
    if logfile:
        logfile.parent.mkdir(parents=True, exist_ok=True)

    def log(event, **details):
        record = dict(event=event, wall_time_unix=time.time(), **details)
        line = json.dumps(record, ensure_ascii=False)
        if logfile:
            with logfile.open('a') as stream:
                stream.write(line + '\n')
        if event != 'command':
            print(line, flush=True)

    def publish(x, z):
        command = Twist()
        command.linear.x = x
        command.angular.z = z
        publisher.publish(command)
        if state['last_command'] != (x, z):
            log('command', linear_x=x, angular_z=z)
            state['last_command'] = (x, z)

    def stop(reason):
        blocked.update(pressed)
        pressed.clear()
        publish(0.0, 0.0)
        message.set(reason)
        log('stop', reason=reason)

    def on_clock(msg):
        stamp = (msg.clock.sec, msg.clock.nanosec)
        if clock['stamp'] != stamp:
            clock['advanced_at'] = time.monotonic()
        clock['stamp'] = stamp

    subscription = node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)

    def has_focus():
        window = xdisplay.get_input_focus().focus
        own_id = root.winfo_id()
        wrapper_id = xdisplay.create_resource_object('window', own_id).query_tree().parent.id
        for _ in range(12):
            if not hasattr(window, 'id'):
                return False
            if window.id in (own_id, wrapper_id):
                return True
            if window.id == xdisplay.screen().root.id:
                return False
            window = window.query_tree().parent
        return False

    def read_keys():
        focused = has_focus()
        if not focused:
            if state['focused'] or pressed:
                stop('Da doi cua so — robot dung. Click lai cua so nay de dieu khien.')
            state['focused'] = False
            return
        state['focused'] = True
        bits = xdisplay.query_keymap()
        down = lambda code: code > 0 and bool(bits[code // 8] & (1 << (code % 8)))
        physical = {key for key, code in keycodes.items() if down(code)}
        blocked.intersection_update(physical)
        if any(down(code) for code in stopcodes.values()):
            blocked.update(physical)
            if pressed:
                stop('Da dung — tha het phim truoc khi di tiep.')
            return
        if down(ccode) and any(down(code) for code in ctrlcodes):
            close()
            return
        active = physical - blocked
        for key in active - pressed:
            log('key_press', key=key, input='X11 key state, independent of input method')
        for key in pressed - active:
            log('key_release', key=key)
        pressed.clear()
        pressed.update(active)

    def close():
        if not state['closed']:
            stop('Dong teleop')
            state['closed'] = True
            root.quit()

    def tick():
        if state['closed']:
            return
        rclpy.spin_once(node, timeout_sec=0)
        read_keys()
        if state["closed"]:
            return
        now = time.monotonic()
        if now - graph['checked_at'] > 1:
            names = node.get_node_names()
            subscribers = node.get_subscriptions_info_by_topic('/cmd_vel')
            graph['ready'] = ('clock_bridge' in names and
                any(item.node_name == 'motion_control' for item in subscribers) and
                'controller_server' not in names and 'bt_navigator' not in names)
            graph['reason'] = 'Cho clock_bridge/motion_control; dung Nav2 truoc teleop.'
            graph['checked_at'] = now
        if not graph['ready'] or now - clock['advanced_at'] > 1:
            if pressed:
                stop('Simulation chua san sang/paused hoac Nav2 dang chay.')
            publish(0.0, 0.0)
            message.set(graph['reason'] if not graph['ready'] else 'Clock dung/paused — kiem tra Play.')
        else:
            try:
                linear = min(0.35, max(0.05, float(speed.get())))
                angular = min(1.0, max(0.1, float(turn.get())))
            except (ValueError, tk.TclError):
                stop('Toc do khong hop le.')
                linear = angular = 0.0
            x = max(-1, min(1, sum(KEYS[key][0] for key in pressed))) * linear
            z = max(-1, min(1, sum(KEYS[key][1] for key in pressed))) * angular
            publish(x, z)
            if pressed:
                message.set(f'Dang GIU {", ".join(sorted(pressed))}: {x:.2f} m/s, {z:.2f} rad/s')
            else:
                message.set('San sang — click cua so nay, GIU phim de chay. Khong can Enter.')
        root.after(50, tick)

    panel = ttk.Frame(root, padding=16)
    panel.pack(fill='both', expand=True)
    ttk.Label(panel, text='GIỮ PHÍM ĐỂ CHẠY — THẢ PHÍM ĐỂ DỪNG', font=('', 14, 'bold')).pack(pady=6)
    ttk.Label(panel, text='W / ↑ / i: tiến     S / ↓ / ,: lùi\nA / ← / j: quay trái     D / → / l: quay phải\nGiữ W + A/D để đi cong. Space / k / Esc: dừng.', justify='center').pack(pady=8)
    row = ttk.Frame(panel)
    row.pack(pady=10)
    ttk.Label(row, text='Tiến (m/s):').pack(side='left')
    ttk.Spinbox(row, from_=0.05, to=0.35, increment=0.05, textvariable=speed, width=6).pack(side='left', padx=8)
    ttk.Label(row, text='Quay (rad/s):').pack(side='left')
    ttk.Spinbox(row, from_=0.1, to=1.0, increment=0.1, textvariable=turn, width=6).pack(side='left', padx=8)
    ttk.Button(panel, text='DỪNG ROBOT', command=lambda: stop('Da nhan DUNG')).pack(pady=8)
    ttk.Label(panel, textvariable=message, wraplength=550, justify='center').pack(pady=8)
    ttk.Label(panel, text='Chuyển sang Gazebo/RViz tự dừng. Quay chậm để map không lệch.').pack()
    root.protocol('WM_DELETE_WINDOW', close)
    signal.signal(signal.SIGINT, lambda *_: close())
    signal.signal(signal.SIGTERM, lambda *_: close())
    root.after(100, root.focus_force)
    root.after(100, tick)
    log('started', speed=args.speed, turn=args.turn, rate_hz=20, input='X11 key state, not text/IME', release_poll_ms=50)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(10):
            publish(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        root.destroy()
        xdisplay.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
