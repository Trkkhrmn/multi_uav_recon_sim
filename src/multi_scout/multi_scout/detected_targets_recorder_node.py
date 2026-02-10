#!/usr/bin/env python3
# fused_targets topicindeki hedefleri dosyaya yaziyor
# timer ile veya kapatirken kaydedebilir

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseArray


class DetectedTargetsRecorderNode(Node):
    def __init__(self):
        super().__init__('detected_targets_recorder_node')

        self.declare_parameter('fused_targets_topic', '/scout/fused_targets')
        self.declare_parameter('output_file', '')  # e.g. data/detected_scenario_1.yaml
        self.declare_parameter('save_interval_s', 30.0)  # 0 = only on shutdown
        self.declare_parameter('save_on_shutdown', True)

        self.fused_topic = self.get_parameter('fused_targets_topic').value
        raw_output = self.get_parameter('output_file').value
        if isinstance(raw_output, str) and raw_output.strip():
            self.output_file = os.path.abspath(os.path.expanduser(raw_output.strip()))
        else:
            # parametre verilmemisse buraya yaz
            self.output_file = os.path.join(
                os.path.expanduser('~'), 'multi_uav_recon_ws', 'output', 'detected_final.yaml'
            )
        save_interval = float(self.get_parameter('save_interval_s').value or 0)
        self.save_on_shutdown = bool(self.get_parameter('save_on_shutdown').value)

        self.detected = []  # [(x, y), ...]
        self._last_header_stamp = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.sub = self.create_subscription(
            PoseArray, self.fused_topic, self._cb_fused, qos
        )

        if save_interval > 0:
            self.timer = self.create_timer(save_interval, self._save_now)
            self.get_logger().info(
                f"Saving every {save_interval}s"
                + (", and on shutdown" if self.save_on_shutdown else "")
                + f" -> {self.output_file}"
            )
        else:
            self.timer = None
            self.get_logger().info(f"Saving only on exit (Ctrl+C) -> {self.output_file}")

    def _cb_fused(self, msg):
        self.detected = [(p.position.x, p.position.y) for p in msg.poses]
        self._last_header_stamp = msg.header.stamp

    # yaml formatinda dosyaya yaziyorum
    def _write_to_file(self, path):
        if not path:
            return
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        data = {
            'detected_targets': [{'x': x, 'y': y} for (x, y) in self.detected],
            'count': len(self.detected),
            'source_topic': self.fused_topic,
        }
        if self._last_header_stamp:
            data['last_update'] = {
                'sec': int(self._last_header_stamp.sec),
                'nanosec': int(self._last_header_stamp.nanosec),
            }
        with open(path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
        self.get_logger().info(f"Wrote coords to {path} ({len(self.detected)} targets)")

    def _save_now(self):
        if self.output_file:
            self._write_to_file(self.output_file)

    def destroy_node(self):
        if self.save_on_shutdown and self.output_file:
            self._write_to_file(self.output_file)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectedTargetsRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
