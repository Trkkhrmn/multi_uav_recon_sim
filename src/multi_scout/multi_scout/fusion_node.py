#!/usr/bin/env python3
# fusion node - 4 dronedan gelen tespitleri birlestiriyor
# ayni hedefe ait olanlari merge_radius icinde birlestirip centroid aliyor

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PointStamped, PoseArray, Pose, Quaternion
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray, Marker


class FusionNode(Node):
    def __init__(self):
        super().__init__('fusion_node')

        self.declare_parameter('merge_radius_m', 4.0)   # Same object: detections within this radius (m) get merged. Smaller = more separate targets.
        self.declare_parameter('enemy_zone_min_x', -70.44)
        self.declare_parameter('enemy_zone_max_x', 77.17)
        self.declare_parameter('enemy_zone_min_y', -64.58)
        self.declare_parameter('enemy_zone_max_y', 64.68)
        self.declare_parameter('detection_topics', [
            '/scout/detections_1', '/scout/detections_2',
            '/scout/detections_3', '/scout/detections_4'
        ])
        self.declare_parameter('output_topic', '/scout/fused_targets')
        self.declare_parameter('center_topic', '/scout/detected_targets_for_center')  # Final target list sent to center/MRTA
        self.declare_parameter('publish_interval_s', 2.0)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('markers_topic', '/scout/fused_targets_markers')
        self.declare_parameter('min_observations', 1)  # 1 = targets seen only once get published too
        self.declare_parameter('target_completed_topic', '/mrta/target_completed')  # MRTA says enough packages dropped here

        self.merge_radius = self.get_parameter('merge_radius_m').value
        self.min_observations = int(self.get_parameter('min_observations').value)
        self.enemy_min_x = self.get_parameter('enemy_zone_min_x').value
        self.enemy_max_x = self.get_parameter('enemy_zone_max_x').value
        self.enemy_min_y = self.get_parameter('enemy_zone_min_y').value
        self.enemy_max_y = self.get_parameter('enemy_zone_max_y').value
        detection_topics = self.get_parameter('detection_topics').value
        output_topic = self.get_parameter('output_topic').value
        center_topic = self.get_parameter('center_topic').value
        pub_interval = self.get_parameter('publish_interval_s').value
        publish_markers = self.get_parameter('publish_markers').value
        markers_topic = self.get_parameter('markers_topic').value
        target_completed_topic = self.get_parameter('target_completed_topic').value

        # birlestirilmis hedefler listesi - her biri x,y, observations, type
        self.fused_targets = []

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        for topic in detection_topics:
            self.create_subscription(
                PointStamped,
                topic,
                self._make_callback(topic),
                qos
            )

        self.pub_fused = self.create_publisher(PoseArray, output_topic, 10)
        self.pub_center = self.create_publisher(PoseArray, center_topic, 10)
        self.create_subscription(
            PointStamped,
            target_completed_topic,
            self._cb_target_completed,
            10
        )
        if publish_markers:
            self.pub_markers = self.create_publisher(MarkerArray, markers_topic, 10)
        else:
            self.pub_markers = None
        self.timer = self.create_timer(pub_interval, self._publish_fused)

        self.get_logger().info(
            f"Fusion started: {len(detection_topics)} topics, merge radius {self.merge_radius}m, "
            f"min obs {self.min_observations}, output: {output_topic}"
        )

    def _make_callback(self, topic_name):
        def callback(msg):
            x, y = msg.point.x, msg.point.y
            if not self._inside_enemy_zone(x, y):
                return
            # point.z da tip var (0 mavi 1 magenta 2 cyan)
            target_type = int(round(msg.point.z))
            if target_type < 0 or target_type > 2:
                target_type = 0
            self._add_detection(x, y, target_type)
        return callback

    def _inside_enemy_zone(self, x, y):
        return (
            self.enemy_min_x <= x <= self.enemy_max_x and
            self.enemy_min_y <= y <= self.enemy_max_y
        )

    def _add_detection(self, x, y, target_type=0):
        for t in self.fused_targets:
            if math.hypot(x - t['x'], y - t['y']) <= self.merge_radius:
                t['observations'].append((x, y))
                n = len(t['observations'])
                t['x'] = sum(p[0] for p in t['observations']) / n
                t['y'] = sum(p[1] for p in t['observations']) / n
                # tipi cogunluga gore al - en cok gelen tip
                t.setdefault('type_votes', []).append(target_type)
                from collections import Counter
                most_common = Counter(t['type_votes']).most_common(1)[0][0]
                t['type'] = most_common
                self.get_logger().info(
                    f"Target updated (obs {n}, type={t['type']}): ({t['x']:.2f}, {t['y']:.2f})"
                )
                return
        self.fused_targets.append({
            'x': x, 'y': y,
            'observations': [(x, y)],
            'type': target_type,
            'type_votes': [target_type],
        })
        type_labels = {0: 'BLUE', 1: 'MAGENTA', 2: 'CYAN'}
        self.get_logger().info(
            f"New target: ({x:.2f}, {y:.2f}) type={type_labels.get(target_type, '?')}, "
            f"total {len(self.fused_targets)} targets"
        )

    # mrta bu hedefi tamamladiginda listeden cikar
    def _cb_target_completed(self, msg):
        x, y = msg.point.x, msg.point.y
        before = len(self.fused_targets)
        self.fused_targets = [
            t for t in self.fused_targets
            if math.hypot(t['x'] - x, t['y'] - y) > self.merge_radius
        ]
        if len(self.fused_targets) < before:
            self.get_logger().info(f"Target done, removed from list: ({x:.2f}, {y:.2f}), {len(self.fused_targets)} left")

    def _publish_fused(self):
        stamp = self.get_clock().now().to_msg()
        # min_observations kadar goren hedefleri yayinla (gurultu azalsin diye)
        filtered = [t for t in self.fused_targets if len(t['observations']) >= self.min_observations]
        msg = PoseArray()
        msg.header = Header(stamp=stamp, frame_id='world')
        for t in filtered:
            p = Pose()
            p.position.x = float(t['x'])
            p.position.y = float(t['y'])
            p.position.z = 0.0
            # tip orientation.x ile gidiyor (pose'da baska yerde tasimadim)
            tt = int(t.get('type', 0))
            p.orientation = Quaternion(x=float(tt), y=0.0, z=0.0, w=1.0)
            msg.poses.append(p)
        self.pub_fused.publish(msg)
        self.pub_center.publish(msg)  # ayni listeyi merkeze de gonderiyorum

        if self.pub_markers:
            ma = MarkerArray()
            for i, t in enumerate(filtered):
                m = Marker()
                m.header.stamp = stamp
                m.header.frame_id = 'world'
                m.ns = 'fused_targets'
                m.id = i
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = t['x']
                m.pose.position.y = t['y']
                m.pose.position.z = 0.0
                m.scale.x = m.scale.y = m.scale.z = 3.0
                tt = int(t.get('type', 0))
                if tt == 0:
                    m.color.r, m.color.g, m.color.b = 0.2, 0.2, 1.0   # mavi
                elif tt == 1:
                    m.color.r, m.color.g, m.color.b = 0.85, 0.0, 0.85  # magenta
                else:
                    m.color.r, m.color.g, m.color.b = 0.0, 0.85, 0.85  # cyan
                m.color.a = 0.9
                ma.markers.append(m)
            self.pub_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
