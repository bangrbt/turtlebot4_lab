"""Prepare simulation: moves standard_dock away so TurtleBot 4 is undocked
and ready to receive /cmd_vel and diffdrive commands immediately.
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
    node = Node('emotion_prepare_simulation')
    stopped = False
    ready = False
    data = {}

    def stop(signum, frame):
        nonlocal stopped
        stopped = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    tl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(Bool, '/emotion/sim_ready', tl_qos)

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
        node.get_logger().info("Đang chờ đồng hồ mô phỏng Gazebo và tọa độ Odometry...")
        wait_until(lambda: 'odom' in data and node.get_clock().now().nanoseconds > 2_000_000_000, 30)

        if not stopped and rclpy.ok():
            # Relocate standard_dock outside arena on flat ground
            request = 'name: "standard_dock", position: {x: -6.0, y: -6.0, z: 0.0}, orientation: {w: 1}'
            result = subprocess.run(
                [
                    'ign', 'service', '-s', '/world/emotion_arena/set_pose',
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
                node.get_logger().info("[+] Đã di chuyển dock sạc ra ngoài sàn đấu! TurtleBot 4 đã nhả sạc (undocked) thành công.")
            else:
                node.get_logger().info("Dock notice: ready for navigation.")

            ready = True
            pub.publish(Bool(data=True))
            node.get_logger().info("=== Sân đấu Arena đã sẵn sàng! TurtleBot 4 sẵn sàng di chuyển theo biểu cảm khuôn mặt! ===")

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
