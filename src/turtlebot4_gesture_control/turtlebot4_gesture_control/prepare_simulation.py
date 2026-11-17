"""Prepare only the simulated arena; never move the robot before confirmation."""
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from std_msgs.msg import Bool
from irobot_create_msgs.msg import DockStatus
from nav_msgs.msg import Odometry


def main(args=None):
    rclpy.init(args=args);node=Node('gesture_prepare_simulation');stopped=False;ready=False;data={}
    def stop(signum,frame):
        nonlocal stopped
        stopped=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    pub=node.create_publisher(Bool,'/gesture/arena_ready',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    timer=node.create_timer(.3,lambda:pub.publish(Bool(data=ready)),clock=Clock(clock_type=ClockType.STEADY_TIME))
    inputs=[node.create_subscription(Odometry,'/odom',lambda m:data.update(odom=m),qos_profile_sensor_data),
            node.create_subscription(DockStatus,'/dock_status',lambda m:data.update(dock=m),qos_profile_sensor_data)]
    def wait_until(predicate,timeout):
        end=time.monotonic()+timeout
        while not predicate():
            if stopped or time.monotonic()>end:raise RuntimeError('Preparation interrupted/timed out')
            rclpy.spin_once(node,timeout_sec=.05)
    try:
        pub.publish(Bool(data=False))
        wait_until(lambda:'odom' in data and 'dock' in data and node.get_clock().now().nanoseconds>3_000_000_000,60)
        # Move only the dock, not the robot. No undock trajectory and no safety_override.
        request='name: "standard_dock", position: {x: -4.5, y: -4.5, z: 0}, orientation: {w: 1}'
        result=subprocess.run(['ign','service','-s','/world/gesture_arena/set_pose',
            '--reqtype','ignition.msgs.Pose','--reptype','ignition.msgs.Boolean','--timeout','5000','--req',request],
            capture_output=True,text=True,timeout=8)
        if result.returncode or 'data: true' not in result.stdout:raise RuntimeError('Cannot move dock outside demo area')
        wait_until(lambda:not data['dock'].is_docked,10)
        ready=True
        node.get_logger().info('Arena ready: dock moved away; robot stationary. Select a gesture, then confirm with Thumb Up.')
        while rclpy.ok() and not stopped:rclpy.spin_once(node,timeout_sec=.1)
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc));return 1
    finally:
        ready=False;pub.publish(Bool(data=False))
        for _ in range(3):rclpy.spin_once(node,timeout_sec=.03)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
