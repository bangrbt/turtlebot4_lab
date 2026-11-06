"""Zone Visualizer Node: publishes RViz MarkerArray containing 3D floating zone labels,
colored room boundaries, and active route lines on the ROS 2 /map frame.
"""
from pathlib import Path
from typing import List, Dict, Any
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import String, ColorRGBA

from .semantic_map import SemanticMap, Zone


class ZoneVisualizerNode(Node):
    """Publishes semantic zone boundaries, 3D text labels, and active waypoint routes to RViz2."""

    def __init__(self):
        super().__init__('zone_visualizer_node')

        self.declare_parameter('zones_file', '')
        self.declare_parameter('markers_topic', '/semantic_zones_markers')
        self.declare_parameter('status_topic', '/voice/navigation_status')
        self.declare_parameter('publish_rate', 1.0)

        zones_path = self.get_parameter('zones_file').value
        if not zones_path:
            pkg_root = Path(__file__).resolve().parent.parent
            zones_path = str(pkg_root / 'config/zones.yaml')

        self.semantic_map = SemanticMap(zones_path)

        # Latching QoS for RViz markers
        tl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(MarkerArray, self.get_parameter('markers_topic').value, tl_qos)

        # Listen to navigation status to draw dynamic route lines
        self.status_sub = self.create_subscription(
            String,
            self.get_parameter('status_topic').value,
            self.on_nav_status,
            10
        )

        self.active_route_names: List[str] = []
        self.active_state = "IDLE"

        # Static markers (zones, polygons, labels)
        self.static_markers = self.build_static_zone_markers()

        # Timer to keep RViz updated
        rate = float(self.get_parameter('publish_rate').value)
        self.timer = self.create_timer(1.0 / max(0.1, rate), self.publish_all_markers)

        self.get_logger().info(f"Zone Visualizer ready. Publishing to {self.get_parameter('markers_topic').value}")

    def on_nav_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.active_state = data.get('state', 'IDLE')
            self.active_route_names = data.get('active_route', [])
        except Exception:
            pass

    def build_static_zone_markers(self) -> MarkerArray:
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0

        for zid, z in self.semantic_map.get_all_zones().items():
            r, g, b = [c / 255.0 for c in z.color_rgb]

            # 1. 3D Floating Text Label
            txt_marker = Marker()
            txt_marker.header.frame_id = 'map'
            txt_marker.header.stamp = stamp
            txt_marker.ns = 'zone_labels'
            txt_marker.id = marker_id
            marker_id += 1
            txt_marker.type = Marker.TEXT_VIEW_FACING
            txt_marker.action = Marker.ADD

            # Place label at centroid / target
            txt_marker.pose.position.x = z.target_pose.x
            txt_marker.pose.position.y = z.target_pose.y
            txt_marker.pose.position.z = z.marker_z + 0.4
            txt_marker.scale.z = 0.38  # Text height in meters
            txt_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
            txt_marker.text = f"[{z.id}] {z.name}"
            arr.markers.append(txt_marker)

            # 2. Polygon Boundary Outline (LINE_STRIP)
            if len(z.polygon) >= 3:
                line_marker = Marker()
                line_marker.header.frame_id = 'map'
                line_marker.header.stamp = stamp
                line_marker.ns = 'zone_boundaries'
                line_marker.id = marker_id
                marker_id += 1
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                line_marker.scale.x = 0.08  # Line width
                line_marker.color = ColorRGBA(r=r, g=g, b=b, a=0.9)

                for pt in z.polygon:
                    line_marker.points.append(Point(x=pt[0], y=pt[1], z=0.05))
                # Close loop
                line_marker.points.append(Point(x=z.polygon[0][0], y=z.polygon[0][1], z=0.05))
                arr.markers.append(line_marker)

            # 3. Target Pose Cylinder Indicator
            cyl_marker = Marker()
            cyl_marker.header.frame_id = 'map'
            cyl_marker.header.stamp = stamp
            cyl_marker.ns = 'zone_targets'
            cyl_marker.id = marker_id
            marker_id += 1
            cyl_marker.type = Marker.CYLINDER
            cyl_marker.action = Marker.ADD
            cyl_marker.pose.position.x = z.target_pose.x
            cyl_marker.pose.position.y = z.target_pose.y
            cyl_marker.pose.position.z = 0.15
            cyl_marker.scale.x = 0.4
            cyl_marker.scale.y = 0.4
            cyl_marker.scale.z = 0.3
            cyl_marker.color = ColorRGBA(r=r, g=g, b=b, a=0.7)
            arr.markers.append(cyl_marker)

        return arr

    def build_dynamic_route_markers(self) -> List[Marker]:
        stamp = self.get_clock().now().to_msg()
        markers = []

        if self.active_state == "NAVIGATING" and len(self.active_route_names) >= 1:
            route_marker = Marker()
            route_marker.header.frame_id = 'map'
            route_marker.header.stamp = stamp
            route_marker.ns = 'active_route'
            route_marker.id = 999
            route_marker.type = Marker.LINE_STRIP
            route_marker.action = Marker.ADD
            route_marker.scale.x = 0.12  # Thick path line
            route_marker.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.95)  # Bright gold

            for name in self.active_route_names:
                zone = self.semantic_map.lookup_zone(name)
                if zone:
                    route_marker.points.append(Point(x=zone.target_pose.x, y=zone.target_pose.y, z=0.2))

            if len(route_marker.points) >= 2:
                markers.append(route_marker)
        else:
            # Delete route marker if inactive
            del_marker = Marker()
            del_marker.header.frame_id = 'map'
            del_marker.header.stamp = stamp
            del_marker.ns = 'active_route'
            del_marker.id = 999
            del_marker.action = Marker.DELETE
            markers.append(del_marker)

        return markers

    def publish_all_markers(self):
        arr = MarkerArray()
        arr.markers.extend(self.static_markers.markers)
        arr.markers.extend(self.build_dynamic_route_markers())
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ZoneVisualizerNode()
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
