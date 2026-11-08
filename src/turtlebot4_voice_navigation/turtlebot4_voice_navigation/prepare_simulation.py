"""Prepare simulation: moves standard_dock away so TurtleBot 4 is undocked
and ready to receive Nav2 /cmd_vel navigation commands immediately.
"""
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry


def main(args=None):
    rclpy.init(args=args)
    node = Node('prepare_voice_simulation')
    stopped = False
    ready = False
    data = {}

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    tl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(Bool, '/voice/sim_ready', tl_qos)

    sub_odom = node.create_subscription(
        Odometry,
        '/odom',
        lambda m: data.update(odom=m),
        qos_profile_sensor_data
    )

    def wait_until(predicate, timeout):
        end = time.monotonic() + timeout
        while not predicate() and not stopped and rclpy.ok():
            if time.monotonic() > end:
                return False
            rclpy.spin_once(node, timeout_sec=0.05)
        return not stopped and rclpy.ok()

    try:
        pub.publish(Bool(data=False))
        node.get_logger().info("Waiting for simulation clock and odometry...")
        wait_until(lambda: 'odom' in data and node.get_clock().now().nanoseconds > 2_000_000_000, 30)

        if not stopped and rclpy.ok():
            # Relocate standard_dock outside arena on flat ground
            request = 'name: "standard_dock", position: {x: -8.0, y: -8.0, z: 0.0}, orientation: {w: 1}'
            result = subprocess.run(
                [
                    'ign', 'service', '-s', '/world/smart_campus/set_pose',
                    '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '4000',
                    '--req', request
                ],
                capture_output=True,
                text=True,
                timeout=5
            )
            if result.returncode == 0 and 'data: true' in result.stdout:
                node.get_logger().info("[+] Dock successfully relocated outside arena. Robot undocked and mobile.")
            else:
                node.get_logger().info("Dock notice: ready for navigation.")

            ready = True
            pub.publish(Bool(data=True))
            node.get_logger().info("=== Smart Campus Arena Ready! TurtleBot 4 is undocked at (0, 0) in Zone E (Lobby). ===")
            node.get_logger().info("Ready for voice commands via microphone or voice_terminal.py.")

        while rclpy.ok() and not stopped:
            pub.publish(Bool(data=ready))
            rclpy.spin_once(node, timeout_sec=0.2)
        return 0

    except (KeyboardInterrupt, SystemExit):
        return 0
    except Exception as exc:
        node.get_logger().error(f"Error preparing simulation: {exc}")
        return 1
    finally:
        ready = False
        try:
            pub.publish(Bool(data=False))
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
