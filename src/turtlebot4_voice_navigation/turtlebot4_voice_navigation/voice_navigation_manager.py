"""Voice Navigation Manager: processes parsed voice intents and manages
autonomous Nav2 NavigateToPose / NavigateThroughPoses action execution.
"""
from pathlib import Path
from typing import List, Optional, Tuple, Dict, Any
import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

from .semantic_map import SemanticMap, Zone
from .nlu_parser import CommandParser, VoiceIntent, IntentType


class VoiceNavigationManager(Node):
    """Coordinates voice command interpretation, spatial location tracking,
    and Nav2 action navigation.
    """

    def __init__(self):
        super().__init__('voice_navigation_manager')

        self.declare_parameter('zones_file', '')
        self.declare_parameter('navigate_to_pose_action', '/navigate_to_pose')
        self.declare_parameter('navigate_through_poses_action', '/navigate_through_poses')
        self.declare_parameter('recognized_topic', '/voice/recognized_text')
        self.declare_parameter('status_topic', '/voice/navigation_status')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('waypoint_dwell_seconds', 2.0)
        self.declare_parameter('action_timeout_seconds', 300.0)

        zones_path = self.get_parameter('zones_file').value
        if not zones_path:
            # Fallback to local package share or source
            pkg_root = Path(__file__).resolve().parent.parent
            zones_path = str(pkg_root / 'config/zones.yaml')

        self.get_logger().info(f"Loading semantic map from: {zones_path}")
        self.semantic_map = SemanticMap(zones_path)
        self.parser = CommandParser(self.semantic_map)

        # Nav2 Action Clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, self.get_parameter('navigate_to_pose_action').value)
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, self.get_parameter('navigate_through_poses_action').value)

        # Publishers & Subscriptions
        tl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, self.get_parameter('status_topic').value, tl_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)

        self.text_sub = self.create_subscription(
            String,
            self.get_parameter('recognized_topic').value,
            self.on_recognized_text,
            10
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.on_amcl_pose,
            qos_profile_sensor_data
        )

        # TF Buffer for continuous robot localization
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Internal State
        self.current_pose: Optional[Tuple[float, float, float]] = None  # (x, y, yaw)
        self.current_goal_handle = None
        self.is_navigating = False
        self.active_intent: Optional[VoiceIntent] = None
        self.active_waypoints: List[Zone] = []
        self.active_waypoint_idx = 0
        self.state = "IDLE"

        # Periodic timer for status publishing and TF check
        self.timer = self.create_timer(0.5, self.on_timer)

        self.publish_status("IDLE", "Hệ thống điều hướng bằng giọng nói đã sẵn sàng.")
        self.get_logger().info("Voice Navigation Manager ready.")

    def on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.current_pose = (p.x, p.y, yaw)

    def get_robot_pose(self) -> Optional[Tuple[float, float, float]]:
        """Queries latest robot pose from TF (map -> base_link), falling back to AMCL."""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.current_pose = (tx, ty, yaw)
            return self.current_pose
        except (LookupException, ConnectivityException, ExtrapolationException):
            return self.current_pose

    def get_current_zone(self) -> Optional[Zone]:
        pose = self.get_robot_pose()
        if pose:
            return self.semantic_map.get_zone_at_point(pose[0], pose[1])
        return None

    def publish_status(self, state: str, feedback_text: str, extra: Optional[Dict[str, Any]] = None):
        self.state = state
        cur_zone = self.get_current_zone()
        pose = self.get_robot_pose()

        payload = {
            "state": state,
            "feedback": feedback_text,
            "current_zone": cur_zone.name if cur_zone else "Chưa xác định",
            "current_zone_id": cur_zone.id if cur_zone else None,
            "robot_pose": {"x": round(pose[0], 2), "y": round(pose[1], 2), "yaw": round(pose[2], 2)} if pose else None,
            "active_route": [z.name for z in self.active_waypoints] if self.active_waypoints else [],
            "waypoint_index": self.active_waypoint_idx,
            "timestamp": time.time()
        }
        if extra:
            payload.update(extra)

        self.status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.get_logger().info(f"[STATUS: {state}] {feedback_text}")

    def on_timer(self):
        """Periodic status heartbeat."""
        self.get_robot_pose()

    def on_recognized_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"Parsing recognized command: '{text}'")
        intent = self.parser.parse(text)

        # 1. Handle Invalid Zone
        if intent.intent_type == IntentType.INVALID_ZONE:
            self.publish_status(
                "REJECTED",
                intent.error_message,
                {"error_type": "INVALID_ZONE", "unrecognized": intent.unrecognized_names}
            )
            return

        # 2. Handle Stop / Cancel
        if intent.intent_type == IntentType.STOP:
            self.cancel_active_navigation("Đã nhận lệnh dừng khẩn cấp. Hủy toàn bộ mục tiêu di chuyển.")
            return

        # 3. Handle Location Query
        if intent.intent_type == IntentType.QUERY_LOCATION:
            cur_zone = self.get_current_zone()
            pose = self.get_robot_pose()
            if cur_zone:
                msg_text = f"Robot hiện đang ở {cur_zone.name} ({cur_zone.description})."
            elif pose:
                msg_text = f"Robot đang ở tọa độ (x={pose[0]:.2f}, y={pose[1]:.2f}) nhưng ngoài ranh giới phòng xác định."
            else:
                msg_text = "Chưa có dữ liệu định vị robot trên bản đồ."
            self.publish_status("QUERY_RESULT", msg_text)
            return

        # 4. Handle Unknown Intent
        if not intent.is_valid or intent.intent_type == IntentType.UNKNOWN:
            self.publish_status("REJECTED", intent.error_message, {"error_type": "UNKNOWN_INTENT"})
            return

        # 5. Handle Navigation Intents
        self.execute_navigation_intent(intent)

    def execute_navigation_intent(self, intent: VoiceIntent):
        """Builds waypoint sequence and dispatches Nav2 action goal."""
        self.active_intent = intent
        cur_zone = self.get_current_zone()

        waypoints: List[Zone] = []

        if intent.intent_type == IntentType.NAVIGATE_TO:
            waypoints = [intent.target_zone]

        elif intent.intent_type == IntentType.NAVIGATE_FROM_TO:
            orig = intent.origin_zone
            dest = intent.target_zone
            # If robot is already at origin, navigate directly to destination
            if cur_zone and cur_zone.id == orig.id:
                self.get_logger().info(f"Robot already at {orig.name}, navigating directly to {dest.name}.")
                waypoints = [dest]
            else:
                self.get_logger().info(f"Robot not at {orig.name} (currently at {cur_zone.name if cur_zone else 'outside'}). Routing via {orig.name} then {dest.name}.")
                waypoints = [orig, dest]

        elif intent.intent_type == IntentType.NAVIGATE_WAYPOINTS:
            # Multi-hop route
            waypoints = intent.waypoint_zones.copy()
            # If robot is already at the first waypoint, can skip it
            if cur_zone and len(waypoints) > 1 and cur_zone.id == waypoints[0].id:
                waypoints = waypoints[1:]

        self.active_waypoints = waypoints
        self.active_waypoint_idx = 0

        route_str = " -> ".join([z.name for z in waypoints])
        feedback = f"Bắt đầu điều hướng theo lộ trình: {route_str}."
        self.publish_status("NAVIGATING", feedback)

        # Dispatch first waypoint
        self.send_next_waypoint()

    def make_pose_stamped(self, target: Zone) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = target.target_pose.x
        pose.pose.position.y = target.target_pose.y
        pose.pose.position.z = 0.0

        yaw = target.target_pose.yaw
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def send_next_waypoint(self):
        """Sends the next waypoint in the queue to Nav2."""
        if self.active_waypoint_idx >= len(self.active_waypoints):
            self.is_navigating = False
            self.publish_status("COMPLETED", "Đã hoàn thành toàn bộ lộ trình điều hướng thành công!")
            return

        target_zone = self.active_waypoints[self.active_waypoint_idx]
        total = len(self.active_waypoints)
        step = self.active_waypoint_idx + 1

        self.get_logger().info(f"Waiting for Nav2 action server: {self.get_parameter('navigate_to_pose_action').value}")
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=3.0):
            self.publish_status("FAILED", "Nav2 NavigateToPose action server chưa sẵn sàng.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose_stamped(target_zone)

        self.is_navigating = True
        desc = f"Đang di chuyển đến {target_zone.name} (Chặng {step}/{total})."
        self.publish_status("NAVIGATING", desc)

        send_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.on_nav_feedback
        )
        send_future.add_done_callback(self.on_goal_response)

    def on_nav_feedback(self, feedback_msg):
        # Can monitor distance_remaining if available in Nav2 feedback
        pass

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            retries = getattr(self, '_goal_retry_count', 0)
            if retries < 3:
                self._goal_retry_count = retries + 1
                self.get_logger().warn(f"Nav2 rejected goal (costmap/TF warming up). Retrying in 1.5s (attempt {self._goal_retry_count}/3)...")
                self._retry_timer = self.create_timer(1.5, self._retry_send_waypoint)
                return
            self._goal_retry_count = 0
            self.publish_status("FAILED", "Mục tiêu điều hướng bị Nav2 từ chối. Hãy đảm bảo Nav2 và bản đồ đã hoàn tất khởi động.")
            self.is_navigating = False
            return

        self._goal_retry_count = 0
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav_result)

    def _retry_send_waypoint(self):
        if hasattr(self, '_retry_timer') and self._retry_timer:
            self._retry_timer.cancel()
            self._retry_timer = None
        self.send_next_waypoint()

    def on_nav_result(self, future):
        result = future.result()
        status = result.status

        # Status 4 = STATUS_SUCCEEDED in action_msgs/GoalStatus
        if status == 4:
            reached_zone = self.active_waypoints[self.active_waypoint_idx]
            self.active_waypoint_idx += 1
            if self.active_waypoint_idx < len(self.active_waypoints):
                dwell = float(self.get_parameter('waypoint_dwell_seconds').value)
                self.publish_status(
                    "WAYPOINT_REACHED",
                    f"Đã đến {reached_zone.name}. Tạm dừng {dwell:.1f}s trước khi chuyển sang chặng tiếp theo."
                )
                time.sleep(dwell)
                self.send_next_waypoint()
            else:
                self.is_navigating = False
                self.publish_status("COMPLETED", f"Đã hoàn thành điều hướng và đến đích: {reached_zone.name}.")
        else:
            self.is_navigating = False
            self.publish_status("FAILED", f"Điều hướng không thành công (Nav2 Goal Status: {status}).")

    def cancel_active_navigation(self, reason: str = ""):
        """Cancels current Nav2 action goal and stops wheels."""
        if self.current_goal_handle is not None and self.is_navigating:
            self.get_logger().info("Cancelling active Nav2 goal...")
            self.current_goal_handle.cancel_goal_async()

        # Send zero velocity immediately
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.is_navigating = False
        self.active_waypoints.clear()
        self.active_waypoint_idx = 0
        self.publish_status("CANCELED", reason or "Đã hủy mục tiêu điều hướng.")


def main(args=None):
    rclpy.init(args=args)
    manager = VoiceNavigationManager()
    try:
        rclpy.spin(manager)
    except KeyboardInterrupt:
        pass
    finally:
        manager.cancel_active_navigation("Node shutting down")
        manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
