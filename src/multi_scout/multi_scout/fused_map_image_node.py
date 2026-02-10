#!/usr/bin/env python3
# birlestirilmis harita gorseli - zone sinirlari + hedefler
# /scout/fused_map_image topicine image yayinliyor

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


# world koordinatini resim pikseline cevir (yukari kuzey olsun diye y flip)
def _world_to_pixel(x_world, y_world, bounds, width, height):
    min_x, max_x, min_y, max_y = bounds
    if max_x - min_x < 1e-6 or max_y - min_y < 1e-6:
        return None
    px = int((x_world - min_x) / (max_x - min_x) * (width - 1))
    py = int((y_world - min_y) / (max_y - min_y) * (height - 1))
    # ekranda y asagi gidiyor, kuzey yukari olsun diye ceviriyorum
    py = height - 1 - py
    return (px, py)


class FusedMapImageNode(Node):
    def __init__(self):
        super().__init__('fused_map_image_node')

        self.declare_parameter('fused_targets_topic', '/scout/fused_targets')
        self.declare_parameter('output_topic', '/scout/fused_map_image')
        self.declare_parameter('image_width', 800)
        self.declare_parameter('image_height', 800)
        self.declare_parameter('publish_interval_s', 1.0)
        self.declare_parameter('config_path', '')

        self.fused_topic = self.get_parameter('fused_targets_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.img_w = int(self.get_parameter('image_width').value)
        self.img_h = int(self.get_parameter('image_height').value)
        pub_interval = self.get_parameter('publish_interval_s').value
        config_path = self.get_parameter('config_path').value

        if not config_path:
            config_path = os.path.join(
                get_package_share_directory('multi_scout'), 'config', 'coverage_partitions.yaml'
            )
        with open(config_path, 'r') as f:
            data = yaml.safe_load(f)
        self.zones = []
        for d in data.get('drones', []):
            self.zones.append({
                'min_x': d['scan_zone_world_min_x'],
                'max_x': d['scan_zone_world_max_x'],
                'min_y': d['scan_zone_world_min_y'],
                'max_y': d['scan_zone_world_max_y'],
            })
        self.bounds = (
            -70.44, 77.17,
            -64.58, 64.68
        )
        self.bridge = CvBridge()
        self.fused_targets = []  # son alinan hedefler (x, y, tip)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.sub = self.create_subscription(
            PoseArray, self.fused_topic, self._cb_fused, qos
        )
        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.timer = self.create_timer(pub_interval, self._publish_image)

        self.get_logger().info(
            f"Map image: {self.fused_topic} -> {self.output_topic}, {self.img_w}x{self.img_h}"
        )

    def _cb_fused(self, msg):
        self.fused_targets = []
        for p in msg.poses:
            tt = int(round(p.orientation.x)) if hasattr(p, 'orientation') else 0
            if tt < 0 or tt > 2:
                tt = 0
            self.fused_targets.append((p.position.x, p.position.y, tt))

    def _publish_image(self):
        min_x, max_x, min_y, max_y = self.bounds
        img = np.ones((self.img_h, self.img_w, 3), dtype=np.uint8) * 50   # dark grey so grid is readable

        # grid cizgileri her 20 m
        step = 20.0
        for xw in range(int(min_x), int(max_x) + 1, int(step)):
            p = _world_to_pixel(xw, min_y, self.bounds, self.img_w, self.img_h)
            p2 = _world_to_pixel(xw, max_y, self.bounds, self.img_w, self.img_h)
            if p and p2:
                cv2.line(img, p, p2, (70, 70, 70), 1)
        for yw in range(int(min_y), int(max_y) + 1, int(step)):
            p = _world_to_pixel(min_x, yw, self.bounds, self.img_w, self.img_h)
            p2 = _world_to_pixel(max_x, yw, self.bounds, self.img_w, self.img_h)
            if p and p2:
                cv2.line(img, p, p2, (70, 70, 70), 1)

        # Zone rectangles (thick lines)
        colors = [(255, 120, 120), (120, 255, 120), (120, 120, 255), (255, 255, 120)]
        for i, z in enumerate(self.zones):
            pts = []
            for (xw, yw) in [
                (z['min_x'], z['min_y']), (z['max_x'], z['min_y']),
                (z['max_x'], z['max_y']), (z['min_x'], z['max_y']), (z['min_x'], z['min_y'])
            ]:
                p = _world_to_pixel(xw, yw, self.bounds, self.img_w, self.img_h)
                if p:
                    pts.append(p)
            if len(pts) >= 2:
                for j in range(len(pts) - 1):
                    cv2.line(img, pts[j], pts[j + 1], colors[i % 4], 3)

        # hedefleri tip rengine gore ciz
        type_colors = {
            0: (255, 150, 0),    # Mavi (BGR)
            1: (255, 0, 255),    # Magenta (BGR)
            2: (255, 220, 0),    # Cyan (BGR)
        }
        type_labels = {0: 'B', 1: 'M', 2: 'C'}
        for tup in self.fused_targets:
            xw, yw = tup[0], tup[1]
            tt = tup[2] if len(tup) > 2 else 0
            p = _world_to_pixel(xw, yw, self.bounds, self.img_w, self.img_h)
            if p and 0 <= p[0] < self.img_w and 0 <= p[1] < self.img_h:
                color = type_colors.get(tt, (0, 0, 255))
                cv2.circle(img, p, 10, color, -1)
                cv2.circle(img, p, 12, (255, 255, 255), 2)
                cv2.putText(img, type_labels.get(tt, '?'), (p[0] - 5, p[1] + 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        msg_img = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg_img.header.stamp = self.get_clock().now().to_msg()
        msg_img.header.frame_id = 'world'
        self.pub.publish(msg_img)


def main(args=None):
    rclpy.init(args=args)
    node = FusedMapImageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
