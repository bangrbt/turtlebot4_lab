"""Automatically publish initial pose to AMCL on startup and verify transform."""
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__('auto_initial_pose')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.initial_x = float(self.get_parameter('initial_x').value)
        self.initial_y = float(self.get_parameter('initial_y').value)
        self.initial_yaw = float(self.get_parameter('initial_yaw').value)

        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.attempts = 0
        self.max_attempts = 30
        self.localized = False

        self.timer = self.create_timer(1.0, self.on_timer)
        self.get_logger().info(f'Auto initial pose initialized at ({self.initial_x}, {self.initial_y}). Waiting for AMCL/Map...')

    def on_timer(self):
        if self.localized:
            return

        # Check if map -> base_link transform is available
        try:
            if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2)):
                self.get_logger().info('=== [SUCCESS] Robot localization confirmed! Transform map -> base_link is active. Nav2 is READY! ===')
                self.localized = True
                self.timer.cancel()
                return
        except Exception:
            pass

        self.attempts += 1
        if self.attempts > self.max_attempts:
            self.get_logger().warn('Auto initial pose exceeded max attempts. Please check if AMCL is active.')
            self.timer.cancel()
            return

        # Publish initial pose
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.initial_x
        msg.pose.pose.position.y = self.initial_y
        msg.pose.pose.position.z = 0.0

        # Orientation quaternion from yaw
        # For yaw = 0.0: z = 0.0, w = 1.0
        import math
        msg.pose.pose.orientation.z = math.sin(self.initial_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.initial_yaw / 2.0)

        # Standard initial pose covariance
        cov = [0.0] * 36
        cov[0] = 0.25   # x
        cov[7] = 0.25   # y
        cov[35] = 0.068 # yaw
        msg.pose.covariance = cov

        self.pub.publish(msg)
        self.get_logger().info(f'Published initial pose ({self.initial_x}, {self.initial_y}) to /initialpose (attempt {self.attempts}/{self.max_attempts})...')


def main(args=None):
    rclpy.init(args=args)
    node = AutoInitialPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
