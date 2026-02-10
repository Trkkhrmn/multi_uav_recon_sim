#!/usr/bin/env python3
# 4 scout zonunun sinirlarini rvizde gosteriyor
# coverage_partitions.yaml dan okuyor

import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point


class ZoneBoundariesNode(Node):
    def __init__(self):
        super().__init__('zone_boundaries_node')

        self.declare_parameter('config_path', '')
        self.declare_parameter('marker_topic', '/scout/zone_boundaries')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('publish_interval_s', 1.0)

        config_path = self.get_parameter('config_path').value
        if not config_path:
            pkg_share = get_package_share_directory('multi_scout')
            config_path = os.path.join(pkg_share, 'config', 'coverage_partitions.yaml')

        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        drones = data.get('drones', [])

        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.zones = []
        for d in drones:
            self.zones.append({
                'min_x': d['scan_zone_world_min_x'],
                'max_x': d['scan_zone_world_max_x'],
                'min_y': d['scan_zone_world_min_y'],
                'max_y': d['scan_zone_world_max_y'],
            })

        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 1)
        self.timer = self.create_timer(
            self.get_parameter('publish_interval_s').value,
            self._publish
        )
        self.get_logger().info(
            f"{len(self.zones)} zone boundaries publishing, topic: {self.marker_topic}, frame: {self.frame_id}"
        )

    def _publish(self):
        msg = MarkerArray()
        colors = [(1, 0, 0, 0.8), (0, 1, 0, 0.8), (0, 0, 1, 0.8), (1, 1, 0, 0.8)]
        for i, z in enumerate(self.zones):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'scout_zones'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 2.0
            m.color.r = colors[i % 4][0]
            m.color.g = colors[i % 4][1]
            m.color.b = colors[i % 4][2]
            m.color.a = colors[i % 4][3]
            # dikdortgen kose noktalari (z=0 yer seviyesi)
            min_x, max_x = z['min_x'], z['max_x']
            min_y, max_y = z['min_y'], z['max_y']
            for (px, py) in [
                (min_x, min_y), (max_x, min_y), (max_x, max_y), (min_x, max_y), (min_x, min_y)
            ]:
                p = Point()
                p.x = float(px)
                p.y = float(py)
                p.z = 0.0
                m.points.append(p)
            msg.markers.append(m)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZoneBoundariesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
