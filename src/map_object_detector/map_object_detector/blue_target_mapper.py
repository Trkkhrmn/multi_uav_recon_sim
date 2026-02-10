#!/usr/bin/env python3
# mavi, magenta, cyan kupeleri tespit ediyor (kameradan)
# camera_info ile gercek kamera matrisi kullaniyorum, pixel -> yer koordinati
# asagi bakan kamera icin optical to body donusumu var
# hedef tipleri: 0=mavi 1=magenta 2=cyan (point.z ile gonderiyorum)

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from image_geometry import PinholeCameraModel
    HAS_IMAGE_GEOMETRY = True
except ImportError:
    HAS_IMAGE_GEOMETRY = False


# Downward camera: ROS optical (Z fwd, X right, Y down) -> PX4 body NED (X north, Y east, Z down)
R_OPTICAL_TO_BODY_NED = np.array([
    [0, -1, 0],
    [1,  0, 0],
    [0,  0, 1],
], dtype=np.float64)

# HSV araliklari - gazebodaki kupun renklerine gore ayarladim (biraz deneme yanilma)
COLOR_RANGES = {
    'blue': {
        'lower': np.array([100, 40, 40]),
        'upper': np.array([140, 255, 255]),
        'type_id': 0,
        'bgr_draw': (255, 150, 0),   # light blue for debug draw
        'label': 'BLUE',
    },
    'magenta': {
        'lower': np.array([140, 80, 80]),
        'upper': np.array([170, 255, 255]),
        'type_id': 1,
        'bgr_draw': (255, 0, 255),   # magenta debug BGR
        'label': 'MAGENTA',
    },
    'cyan': {
        'lower': np.array([80, 100, 100]),
        'upper': np.array([100, 255, 255]),
        'type_id': 2,
        'bgr_draw': (255, 220, 0),   # cyan debug BGR
        'label': 'CYAN',
    },
}


class BlueTargetMapper(Node):
    def __init__(self):
        super().__init__('blue_target_mapper')
        self.bridge = CvBridge()

        self.declare_parameter('image_topic', '/leader_1/camera/image_raw')
        self.declare_parameter('camera_info_topic', '')
        self.declare_parameter('drone_id', 'px4_1')
        self.declare_parameter('target_altitude', 80.0)
        self.declare_parameter('detection_output_topic', '')
        self.declare_parameter('min_area_px', 40)
        self.declare_parameter('max_area_px', 15000)
        self.declare_parameter('aspect_ratio_min', 0.35)
        self.declare_parameter('aspect_ratio_max', 2.5)
        self.declare_parameter('spawn_x_world', 185.41)
        self.declare_parameter('spawn_y_world', 56.52)
        self.declare_parameter('fallback_hfov_rad', 1.047)

        self.image_topic = self.get_parameter('image_topic').value
        ci_topic = self.get_parameter('camera_info_topic').value
        if not ci_topic and 'image_raw' in self.image_topic:
            ci_topic = self.image_topic.replace('image_raw', 'camera_info')
        self.camera_info_topic = ci_topic
        self.drone_id = self.get_parameter('drone_id').value
        self.target_altitude = self.get_parameter('target_altitude').value
        self.detection_output_topic = self.get_parameter('detection_output_topic').value
        self.min_area_px = int(self.get_parameter('min_area_px').value)
        self.max_area_px = int(self.get_parameter('max_area_px').value)
        self.aspect_ratio_min = float(self.get_parameter('aspect_ratio_min').value)
        self.aspect_ratio_max = float(self.get_parameter('aspect_ratio_max').value)
        self.spawn_x_world = float(self.get_parameter('spawn_x_world').value)
        self.spawn_y_world = float(self.get_parameter('spawn_y_world').value)
        self.fallback_hfov_rad = float(self.get_parameter('fallback_hfov_rad').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_img = self.create_subscription(Image, self.image_topic, self.image_callback, qos)
        self.sub_pos = self.create_subscription(
            VehicleLocalPosition,
            f'/{self.drone_id}/fmu/out/vehicle_local_position_v1',
            self.pos_callback,
            qos,
        )
        self.sub_att = self.create_subscription(
            VehicleAttitude,
            f'/{self.drone_id}/fmu/out/vehicle_attitude',
            self.att_callback,
            qos,
        )
        self.sub_caminfo = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos,
        )

        self.pub_debug = self.create_publisher(Image, 'blue_target_debug', 10)
        self.pub_detection = (
            self.create_publisher(PointStamped, self.detection_output_topic, 10)
            if self.detection_output_topic else None
        )

        self.drone_pos = np.array([0.0, 0.0, 0.0])
        self.drone_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.found_targets = []   # [(world_x, world_y, type_id), ...]
        self.is_scanning_active = False
        self.altitude_tolerance = 5.0

        self.camera_model = None
        self.K_inv_fallback = None
        self._build_fallback_k()

        self.get_logger().info(
            f"Basladi. Hedef yukseklik {self.target_altitude}m, 3 renk (mavi/magenta/cyan)"
        )

    def _build_fallback_k(self):
        w, h = 640, 480
        fx = (w / 2.0) / math.tan(self.fallback_hfov_rad / 2.0)
        K = np.array([[fx, 0, w / 2.0], [0, fx, h / 2.0], [0, 0, 1]], dtype=np.float64)
        self.K_inv_fallback = np.linalg.inv(K)

    def camera_info_callback(self, msg):
        if self.camera_model is not None:
            return
        if HAS_IMAGE_GEOMETRY:
            self.camera_model = PinholeCameraModel()
            self.camera_model.fromCameraInfo(msg)
            self.get_logger().info("Got camera_info, using PinholeCameraModel.")
        else:
            K = np.array(msg.k).reshape(3, 3)
            self.K_inv_fallback = np.linalg.inv(K)
            self.get_logger().info("Got camera_info, using K matrix (no image_geometry).")

    def pos_callback(self, msg):
        self.drone_pos = np.array([msg.x, msg.y, msg.z])
        current_alt = abs(msg.z)
        if not self.is_scanning_active and current_alt >= (self.target_altitude - self.altitude_tolerance):
            self.is_scanning_active = True
            self.get_logger().info(f"Altitude ok ({current_alt:.1f}m), scan enabled.")
        elif self.is_scanning_active and current_alt < 10.0:
            self.is_scanning_active = False
            self.get_logger().info("Landing, scan disabled.")

    def att_callback(self, msg):
        self.drone_quat = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])

    def _quat_to_rotation_matrix(self, q):
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)

    def pixel_to_ground_ned(self, u, v):
        if abs(self.drone_pos[2]) < 0.5:
            return None
        if self.camera_model is not None and HAS_IMAGE_GEOMETRY:
            try:
                ray_opt = self.camera_model.projectPixelTo3dRay((u, v))
                ray_opt = np.array(ray_opt, dtype=np.float64)
            except Exception:
                ray_opt = self.K_inv_fallback @ np.array([u, v, 1.0], dtype=np.float64)
        else:
            ray_opt = self.K_inv_fallback @ np.array([u, v, 1.0], dtype=np.float64)
        ray_opt = ray_opt / np.linalg.norm(ray_opt)
        ray_body = R_OPTICAL_TO_BODY_NED @ ray_opt
        R_body_to_ned = self._quat_to_rotation_matrix(self.drone_quat)
        ray_ned = R_body_to_ned @ ray_body
        if ray_ned[2] <= 1e-6:
            return None
        t = -self.drone_pos[2] / ray_ned[2]
        local_north = self.drone_pos[0] + t * ray_ned[0]
        local_east = self.drone_pos[1] + t * ray_ned[1]
        return (local_north, local_east)

    # bu renk icin maske olustur
    def _create_color_mask(self, hsv, color_name, color_info):
        if 'lower1' in color_info:
            # Red: two ranges merged (0-10 and 170-180)
            mask1 = cv2.inRange(hsv, color_info['lower1'], color_info['upper1'])
            mask2 = cv2.inRange(hsv, color_info['lower2'], color_info['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv, color_info['lower'], color_info['upper'])
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        return mask

    def image_callback(self, msg):
        if not self.is_scanning_active:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        debug_img = cv_image.copy()

        # Detect each colour separately
        for color_name, color_info in COLOR_RANGES.items():
            mask = self._create_color_mask(hsv, color_name, color_info)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            type_id = color_info['type_id']
            draw_color = color_info['bgr_draw']
            label = color_info['label']

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (self.min_area_px < area < self.max_area_px):
                    continue
                x, y, w, h = cv2.boundingRect(cnt)
                if h < 1:
                    continue
                ar = w / float(h)
                if not (self.aspect_ratio_min <= ar <= self.aspect_ratio_max):
                    continue

                M = cv2.moments(cnt)
                if M['m00'] < 1e-6:
                    center_u, center_v = x + w / 2.0, y + h / 2.0
                else:
                    center_u = M['m10'] / M['m00']
                    center_v = M['m01'] / M['m00']

                coords = self.pixel_to_ground_ned(center_u, center_v)
                if not coords:
                    continue
                local_north, local_east = coords
                world_x = self.spawn_x_world + local_east
                world_y = self.spawn_y_world + local_north

                cv2.rectangle(debug_img, (x, y), (x + w, y + h), draw_color, 2)
                cv2.putText(
                    debug_img, f"{label} {world_x:.1f},{world_y:.1f}", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, draw_color, 2,
                )

                is_unique = True
                for (sx, sy, _st) in self.found_targets:
                    if math.hypot(world_x - sx, world_y - sy) < 8.0:
                        is_unique = False
                        break
                if not is_unique:
                    continue

                self.get_logger().info(
                    f"[{label}] Target found: X={world_x:.2f}, Y={world_y:.2f} type={type_id} "
                    f"(alt {abs(self.drone_pos[2]):.0f}m)"
                )
                self.found_targets.append((world_x, world_y, type_id))
                if self.pub_detection:
                    out = PointStamped()
                    out.header.stamp = msg.header.stamp
                    out.header.frame_id = 'world'
                    out.point.x = world_x
                    out.point.y = world_y
                    out.point.z = float(type_id)  # tip bilgisi
                    self.pub_detection.publish(out)

        out_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
        out_msg.header = msg.header
        self.pub_debug.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BlueTargetMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
