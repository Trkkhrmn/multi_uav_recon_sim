#!/usr/bin/env python3
"""
Multi Robot Task Allocation (MRTA) node.

Keşif dronları (4x leader_iris) /scout/fused_targets ile hedef koordinatlarını
yayınladıktan sonra, 6 normal iris drone bu hedeflere atanır.

- /scout/fused_targets (PoseArray, world frame x,y) dinlenir.
- Her worker için px4_{id}/fmu/out/vehicle_local_position_v1 ile konum alınır.
- Atama: açgözlü (greedy) minimum mesafe — her drone en yakın hedefe atanır.
- Akış: Offboard + Arm → transit irtifada hedefe git → bırakma irtifasına alçal →
  yükü fiziksel bırak (Gazebo spawn) → tekrar yüksel → yeni hedef veya üsse dönüş.
"""

import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, Pose, PoseArray
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)


def world_to_local_ned(world_x, world_y, spawn_x_world, spawn_y_world):
    """Gazebo world (X=Doğu, Y=Kuzey) → PX4 yerel NED (x=North, y=East)."""
    local_north = world_y - spawn_y_world
    local_east = world_x - spawn_x_world
    return local_north, local_east


def local_to_world(local_north, local_east, spawn_x_world, spawn_y_world):
    """PX4 yerel NED → Gazebo world."""
    world_x = spawn_x_world + local_east
    world_y = spawn_y_world + local_north
    return world_x, world_y


class MRTANode(Node):
    def __init__(self):
        super().__init__('mrta_node')

        self.declare_parameter('fused_targets_topic', '/scout/fused_targets')
        self.declare_parameter('transit_altitude_m', 50.0)
        self.declare_parameter('drop_altitude_m', 10.0)  # Bu irtifaya alçalıp yük bırakılır (tam iniş yok)
        self.declare_parameter('drop_hover_after_s', 1.0)  # Bıraktıktan sonra bu süre hover, sonra yüksel
        self.declare_parameter('acceptance_radius_xy_m', 3.0)
        self.declare_parameter('acceptance_radius_alt_m', 2.0)
        self.declare_parameter('start_delay_s', 5.0)
        self.declare_parameter('task_done_delay_s', 5.0)  # Görev bittikten sonra yeni atama/üsse dönüş
        self.declare_parameter('spawn_entity_service', '/spawn_entity')
        self.declare_parameter('payload_sdf_path', '')  # Boşsa inline basit kutu kullanılır
        self.declare_parameter('publish_markers', True)  # RViz'de MRTA görselleştirme
        self.declare_parameter('markers_topic', '/mrta/markers')
        self.declare_parameter('status_topic', '/mrta/status')
        self.declare_parameter('marker_frame_id', 'world')
        self.declare_parameter('worker_vehicle_ids', [5, 6, 7, 8, 9, 10])
        self.declare_parameter('worker_spawn_x_world', [183.41, 185.41, 187.41, 189.41, 191.41, 193.41])
        self.declare_parameter('worker_spawn_y_world', [54.52, 54.52, 54.52, 54.52, 54.52, 54.52])

        self.fused_topic = self.get_parameter('fused_targets_topic').value
        self.transit_altitude_m = float(self.get_parameter('transit_altitude_m').value)
        self.transit_altitude_local = -abs(self.transit_altitude_m)
        self.drop_altitude_m = float(self.get_parameter('drop_altitude_m').value)
        self.drop_altitude_local = -abs(self.drop_altitude_m)
        self.drop_hover_after_s = float(self.get_parameter('drop_hover_after_s').value)
        self.spawn_entity_service = self.get_parameter('spawn_entity_service').value
        self.payload_sdf_path = self.get_parameter('payload_sdf_path').value
        self.publish_markers = self.get_parameter('publish_markers').value
        self.markers_topic = self.get_parameter('markers_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.marker_frame_id = self.get_parameter('marker_frame_id').value
        self.acceptance_xy = float(self.get_parameter('acceptance_radius_xy_m').value)
        self.acceptance_alt = float(self.get_parameter('acceptance_radius_alt_m').value)
        self.start_delay_s = float(self.get_parameter('start_delay_s').value)
        self.task_done_delay_s = float(self.get_parameter('task_done_delay_s').value)

        ids = self.get_parameter('worker_vehicle_ids').value
        xs = self.get_parameter('worker_spawn_x_world').value
        ys = self.get_parameter('worker_spawn_y_world').value
        if not isinstance(ids, list):
            ids = []
        if not isinstance(xs, list):
            xs = []
        if not isinstance(ys, list):
            ys = []
        n = min(len(ids), len(xs), len(ys))
        self.workers = []
        for i in range(n):
            vid = int(ids[i])
            sx = float(xs[i])
            sy = float(ys[i])
            self.workers.append({
                'vehicle_id': vid,
                'spawn_x_world': sx,
                'spawn_y_world': sy,
            })

        if not self.workers:
            self.get_logger().warn('worker_drones bos, config dosyasini kontrol et.')

        self.targets = []  # [(world_x, world_y), ...]
        self.positions = {}   # vehicle_id -> VehicleLocalPosition (son bilinen)
        self.assignment = {}  # vehicle_id -> (world_x, world_y) atanmış hedef
        self.state = {}  # IDLE | ARMING | CLIMB | GO | DESCEND | DROP | AFTER_DROP | CLIMB_AFTER_DROP | RETURN_TO_BASE | RTH_LANDED
        self.arming_counter = {}
        self.land_time = {}
        self.drop_triggered = {}   # vehicle_id -> bool (yük bir kez bırakıldı mı)
        self.after_drop_counter = {}  # vehicle_id -> int (AFTER_DROP hover sayacı)
        self.visited_targets = set()
        self._dropped_at = set()  # (vehicle_id, (world_x, world_y)) — bu çiftte yalnızca bir kez yük bırakılır
        self._assignment_radius_m = 5.0
        self._assignment_interval_s = 2.0
        self._last_assignment_time = None
        self.start_time = None
        self._spawn_client = None
        self._payload_sdf_cache = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub_fused = self.create_subscription(
            PoseArray, self.fused_topic, self._cb_fused, 10
        )

        self._worker_pubs = {}  # vehicle_id -> { offboard, setpoint, command }
        self.subs_position = {}

        for w in self.workers:
            vid = w['vehicle_id']
            self.positions[vid] = None
            self.assignment[vid] = None
            self.state[vid] = 'IDLE'
            self.arming_counter[vid] = 0
            self.land_time[vid] = None
            self.drop_triggered[vid] = False
            self.after_drop_counter[vid] = 0
            prefix = f'px4_{vid}/'
            self._worker_pubs[vid] = {
                'offboard': self.create_publisher(
                    OffboardControlMode, f'{prefix}fmu/in/offboard_control_mode', qos),
                'setpoint': self.create_publisher(
                    TrajectorySetpoint, f'{prefix}fmu/in/trajectory_setpoint', qos),
                'command': self.create_publisher(
                    VehicleCommand, f'{prefix}fmu/in/vehicle_command', qos),
            }
            self.subs_position[vid] = self.create_subscription(
                VehicleLocalPosition,
                f'{prefix}fmu/out/vehicle_local_position_v1',
                self._make_position_cb(vid),
                qos,
            )

        self.timer = self.create_timer(0.1, self._timer_callback)
        self._spawn_client = self.create_client(SpawnEntity, self.spawn_entity_service)
        if self.publish_markers:
            self._pub_markers = self.create_publisher(MarkerArray, self.markers_topic, 10)
            self._marker_timer = self.create_timer(0.5, self._publish_mrta_visualization)  # 2 Hz: markers + status
        else:
            self._pub_markers = None
            self._marker_timer = None
        self._pub_status = self.create_publisher(String, self.status_topic, 10)
        self.get_logger().info(
            f'MRTA basladi: {len(self.workers)} drone, hedefler: {self.fused_topic}, '
            f'transit {self.transit_altitude_m}m, birakma {self.drop_altitude_m}m'
        )

    def _make_position_cb(self, vehicle_id):
        def cb(msg):
            self.positions[vehicle_id] = msg
        return cb

    def _cb_fused(self, msg):
        self.targets = [(p.position.x, p.position.y) for p in msg.poses]
        if self.targets and self.start_time is None:
            self.start_time = self.get_clock().now()
        if self.targets:
            self.get_logger().info(f'Hedef listesi guncellendi: {len(self.targets)} hedef')

    def _timestamp_us(self):
        t = self.get_clock().now()
        return t.nanoseconds // 1000

    def _get_payload_sdf(self):
        """Bırakılacak paket SDF (inline basit kutu veya dosyadan)."""
        if self._payload_sdf_cache is not None:
            return self._payload_sdf_cache
        path = (self.payload_sdf_path or '').strip()
        if path and os.path.isfile(os.path.expanduser(path)):
            with open(os.path.expanduser(path), 'r') as f:
                self._payload_sdf_cache = f.read()
            return self._payload_sdf_cache
        # Inline: dinamik küçük kutu (mesh yok, primitive box)
        self._payload_sdf_cache = '''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="payload_box">
    <static>false</static>
    <link name="link">
      <inertial><mass>1.0</mass>
        <inertia><ixx>0.01</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.01</iyy><iyz>0</iyz><izz>0.01</izz></inertia>
      </inertial>
      <collision name="collision">
        <geometry><box><size>0.25 0.25 0.2</size></box></geometry>
        <surface><contact><ode><max_vel>0.1</max_vel><min_depth>0.001</min_depth></ode></contact></surface>
      </collision>
      <visual name="visual">
        <geometry><box><size>0.25 0.25 0.2</size></box></geometry>
        <material><script><name>Gazebo/Orange</name><uri>__default__</uri></script></material>
      </visual>
    </link>
  </model>
</sdf>'''
        return self._payload_sdf_cache

    def _drop_payload_at(self, world_x, world_y, world_z):
        """Gazebo'da (world_x, world_y, world_z) konumuna dinamik paket spawn eder; yerçekimiyle düşer."""
        if self._spawn_client is None or not self._spawn_client.service_is_ready():
            self.get_logger().warn('Spawn servisi hazir degil, yuk birakilamadi.')
            return
        req = SpawnEntity.Request()
        req.name = f'payload_{int(self.get_clock().now().nanoseconds)}'
        req.xml = self._get_payload_sdf().replace('name="payload_box"', f'name="{req.name}"')
        req.initial_pose = Pose()
        req.initial_pose.position.x = float(world_x)
        req.initial_pose.position.y = float(world_y)
        req.initial_pose.position.z = float(world_z) - 0.3
        future = self._spawn_client.call_async(req)
        future.add_done_callback(lambda f: self._on_spawn_done(f, world_x, world_y, world_z))

    def _on_spawn_done(self, future, x, y, z):
        try:
            r = future.result()
            self.get_logger().info(f'Yuk birakildi: ({x:.1f}, {y:.1f}, {z:.1f}), {r.status_message}')
        except Exception as e:
            self.get_logger().error(f'Yuk spawn hatasi: {e}')

    def _publish_mrta_visualization(self):
        """RViz marker'ları ve /mrta/status özetini yayınla."""
        now = self.get_clock().now()
        stamp = now.to_msg()
        if self._pub_markers:
            self._publish_mrta_markers(stamp)
        self._publish_mrta_status()

    def _publish_mrta_markers(self, stamp):
        """Hedefler, atama çizgileri ve drone durum etiketleri (RViz)."""
        ma = MarkerArray()
        frame = self.marker_frame_id
        mid = 0

        def add_marker(ns, scale_xyz, color_rgba, pose_or_points, mtype=Marker.SPHERE, line_pts=None):
            nonlocal mid
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame
            m.ns = ns
            m.id = mid
            mid += 1
            m.action = Marker.ADD
            m.type = mtype
            m.scale.x = scale_xyz[0]
            m.scale.y = scale_xyz[1]
            m.scale.z = scale_xyz[2]
            m.color.r = color_rgba[0]
            m.color.g = color_rgba[1]
            m.color.b = color_rgba[2]
            m.color.a = color_rgba[3]
            if line_pts is not None:
                m.points = line_pts
            else:
                m.pose.position.x = pose_or_points[0]
                m.pose.position.y = pose_or_points[1]
                m.pose.position.z = pose_or_points[2]
            ma.markers.append(m)

        # 1) Hedefler: yeşil = atanmamış/henüz gidilecek, gri = ziyaret edilmiş
        for i, (tx, ty) in enumerate(self.targets):
            visited = any(math.hypot(tx - vx, ty - vy) <= self._assignment_radius_m
                         for (vx, vy) in self.visited_targets)
            z = self.drop_altitude_m if not visited else 0.0
            if visited:
                add_marker('targets_visited', (2.5, 2.5, 2.5), (0.5, 0.5, 0.5, 0.7), (tx, ty, z))
            else:
                add_marker('targets_pending', (3.0, 3.0, 1.0), (0.2, 0.8, 0.2, 0.9), (tx, ty, self.drop_altitude_m))

        # 2) Atama çizgisi: drone konumu -> atandığı hedef (sarı çizgi)
        for w in self.workers:
            vid = w['vehicle_id']
            pos = self.positions.get(vid)
            target_xy = self.assignment.get(vid)
            if pos is None or target_xy is None:
                continue
            wx = w['spawn_x_world'] + pos.y
            wy = w['spawn_y_world'] + pos.x
            wz = -float(pos.z)
            tx, ty = target_xy
            tz = self.drop_altitude_m
            p0 = Point()
            p0.x = float(wx)
            p0.y = float(wy)
            p0.z = float(wz)
            p1 = Point()
            p1.x = float(tx)
            p1.y = float(ty)
            p1.z = float(tz)
            add_marker('assignment_line', (0.15, 0.0, 0.0), (1.0, 0.9, 0.0, 0.85), None, Marker.LINE_LIST, [p0, p1])

        # 3) Drone durum etiketleri: konumun biraz üstünde "vid: STATE"
        for w in self.workers:
            vid = w['vehicle_id']
            pos = self.positions.get(vid)
            if pos is None:
                continue
            wx = w['spawn_x_world'] + pos.y
            wy = w['spawn_y_world'] + pos.x
            wz = -float(pos.z) + 5.0
            st = self.state.get(vid, 'IDLE')
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame
            m.ns = 'drone_state'
            m.id = mid
            mid += 1
            m.action = Marker.ADD
            m.type = Marker.TEXT_VIEW_FACING
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = wz
            m.scale.z = 4.0
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.text = f'W{vid}: {st}'
            ma.markers.append(m)

        self._pub_markers.publish(ma)

    def _publish_mrta_status(self):
        """Özet metin: hangi drone hangi hedefe hangi durumda (topic + log için)."""
        parts = []
        for w in self.workers:
            vid = w['vehicle_id']
            st = self.state.get(vid, 'IDLE')
            target_xy = self.assignment.get(vid)
            if target_xy:
                parts.append(f'W{vid}→({target_xy[0]:.0f},{target_xy[1]:.0f}) {st}')
            else:
                parts.append(f'W{vid} {st}')
        msg = String()
        msg.data = ' | '.join(parts) if parts else 'MRTA (beklemede)'
        self._pub_status.publish(msg)

    def _run_assignment(self):
        """
        Konum bazlı atama: Henüz bir dron atanmamış her hedefe, en yakın IDLE dronu ata.
        Fusion sırası veya hedef sayısı değişse bile her hedef mutlaka bir drona gider.
        """
        if not self.targets or not self.workers:
            return
        if not all(self.positions.get(w['vehicle_id']) is not None for w in self.workers):
            return
        # Atanmış konumlar + ziyaret edilmiş (görevi bitmiş) hedefler
        assigned_positions = [
            self.assignment[vid] for vid in self.assignment
            if self.assignment[vid] is not None
        ]
        def target_covered(tx, ty):
            for (ax, ay) in assigned_positions:
                if math.hypot(tx - ax, ty - ay) <= self._assignment_radius_m:
                    return True
            for (vx, vy) in self.visited_targets:
                if math.hypot(tx - vx, ty - vy) <= self._assignment_radius_m:
                    return True
            return False
        unassigned_targets = [
            (tx, ty) for (tx, ty) in self.targets
            if not target_covered(tx, ty)
        ]
        idle_workers = [w for w in self.workers if self.state.get(w['vehicle_id']) in ('IDLE', 'RTH_LANDED')]
        if not unassigned_targets or not idle_workers:
            return
        # Her atanmamış hedef için en yakın IDLE dronu ata (açgözlü)
        for tx, ty in unassigned_targets:
            if not idle_workers:
                break
            best_dist = float('inf')
            best_w = None
            for w in idle_workers:
                vid = w['vehicle_id']
                pos = self.positions[vid]
                wx, wy = local_to_world(pos.x, pos.y, w['spawn_x_world'], w['spawn_y_world'])
                d = math.hypot(tx - wx, ty - wy)
                if d < best_dist:
                    best_dist = d
                    best_w = w
            if best_w is None:
                break
            vid = best_w['vehicle_id']
            self.assignment[vid] = (tx, ty)
            self.state[vid] = 'ARMING'
            self.arming_counter[vid] = 0
            assigned_positions.append((tx, ty))
            idle_workers.remove(best_w)
            self.get_logger().info(f'Atama: drone {vid} -> hedef ({tx:.1f}, {ty:.1f})')

    def _publish_offboard(self, vehicle_id):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._timestamp_us()
        self._worker_pubs[vehicle_id]['offboard'].publish(msg)

    def _publish_setpoint(self, vehicle_id, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = np.array([float(x), float(y), float(z)], dtype=np.float32)
        msg.velocity = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        msg.yaw = float(yaw)
        msg.timestamp = self._timestamp_us()
        self._worker_pubs[vehicle_id]['setpoint'].publish(msg)

    def _publish_command(self, vehicle_id, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = vehicle_id + 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._timestamp_us()
        self._worker_pubs[vehicle_id]['command'].publish(msg)

    def _timer_callback(self):
        now = self.get_clock().now()
        if self.start_time is not None and self.start_delay_s > 0:
            elapsed = (now - self.start_time).nanoseconds / 1e9
            if elapsed < self.start_delay_s:
                return

        # Periyodik konum bazlı atama: atanmamış hedeflere IDLE dron atanır
        now_sec = now.nanoseconds / 1e9
        if self._last_assignment_time is None:
            self._last_assignment_time = now_sec
        if self.targets and (now_sec - self._last_assignment_time) >= self._assignment_interval_s:
            self._run_assignment()
            self._last_assignment_time = now_sec

        for w in self.workers:
            vid = w['vehicle_id']
            pos = self.positions.get(vid)
            if pos is None:
                continue

            target_xy = self.assignment.get(vid)
            st = self.state.get(vid, 'IDLE')

            self._publish_offboard(vid)

            if st == 'IDLE':
                self._publish_setpoint(vid, 0.0, 0.0, self.transit_altitude_local)
                continue

            if st == 'ARMING':
                self._publish_setpoint(vid, 0.0, 0.0, self.transit_altitude_local)
                self.arming_counter[vid] = self.arming_counter.get(vid, 0) + 1
                if self.arming_counter[vid] >= 30:  # ~3 s offboard setpoint sonra arm
                    self._publish_command(vid, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self._publish_command(vid, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                    self.state[vid] = 'CLIMB'
                continue

            if st == 'CLIMB':
                self._publish_setpoint(vid, 0.0, 0.0, self.transit_altitude_local)
                dz = pos.z - self.transit_altitude_local
                if abs(dz) < self.acceptance_alt:
                    self.state[vid] = 'GO'
                continue

            if st == 'GO' and target_xy:
                tx_w, ty_w = target_xy
                ln, le = world_to_local_ned(
                    tx_w, ty_w, w['spawn_x_world'], w['spawn_y_world']
                )
                self._publish_setpoint(
                    vid, ln, le, self.transit_altitude_local
                )
                dx = pos.x - ln
                dy = pos.y - le
                dz = pos.z - self.transit_altitude_local
                if (math.sqrt(dx*dx + dy*dy) < self.acceptance_xy and
                        abs(dz) < self.acceptance_alt):
                    self.get_logger().info(f'Drone {vid} hedefe vardi, birakma irtifasina iniyor ({self.drop_altitude_m}m)')
                    self.state[vid] = 'DESCEND'
                continue

            if st == 'DESCEND' and target_xy:
                tx_w, ty_w = target_xy
                ln, le = world_to_local_ned(
                    tx_w, ty_w, w['spawn_x_world'], w['spawn_y_world']
                )
                self._publish_setpoint(vid, ln, le, self.drop_altitude_local)
                dx = pos.x - ln
                dy = pos.y - le
                dz = pos.z - self.drop_altitude_local
                if (math.sqrt(dx*dx + dy*dy) < self.acceptance_xy and
                        abs(dz) < self.acceptance_alt):
                    self.get_logger().info(f'Drone {vid} birakma irtifasinda, yuk birakiliyor')
                    self.state[vid] = 'DROP'
                continue

            if st == 'DROP' and target_xy:
                tx_w, ty_w = target_xy
                ln, le = world_to_local_ned(
                    tx_w, ty_w, w['spawn_x_world'], w['spawn_y_world']
                )
                self._publish_setpoint(vid, ln, le, self.drop_altitude_local)
                # Aynı (drone, hedef bölgesi) için yalnızca bir kez yük bırak (çift bırakmayı engelle)
                r = self._assignment_radius_m
                cell = (round(tx_w / r) * r, round(ty_w / r) * r)
                drop_key = (vid, cell)
                if drop_key not in self._dropped_at:
                    world_x = w['spawn_x_world'] + pos.y
                    world_y = w['spawn_y_world'] + pos.x
                    world_z = -float(pos.z)
                    self._drop_payload_at(world_x, world_y, world_z)
                    self._dropped_at.add(drop_key)
                self.drop_triggered[vid] = True
                self.after_drop_counter[vid] = 0
                self.state[vid] = 'AFTER_DROP'
                continue

            if st == 'AFTER_DROP' and target_xy:
                tx_w, ty_w = target_xy
                ln, le = world_to_local_ned(
                    tx_w, ty_w, w['spawn_x_world'], w['spawn_y_world']
                )
                self._publish_setpoint(vid, ln, le, self.drop_altitude_local)
                self.after_drop_counter[vid] = self.after_drop_counter.get(vid, 0) + 1
                if self.after_drop_counter[vid] >= max(1, int(self.drop_hover_after_s * 10)):
                    self.state[vid] = 'CLIMB_AFTER_DROP'
                continue

            if st == 'CLIMB_AFTER_DROP' and target_xy:
                tx_w, ty_w = target_xy
                ln, le = world_to_local_ned(
                    tx_w, ty_w, w['spawn_x_world'], w['spawn_y_world']
                )
                self._publish_setpoint(vid, ln, le, self.transit_altitude_local)
                dx = pos.x - ln
                dy = pos.y - le
                dz = pos.z - self.transit_altitude_local
                if (math.sqrt(dx*dx + dy*dy) < self.acceptance_xy and
                        abs(dz) < self.acceptance_alt):
                    # Görev bitti: hedefi ziyaret edildi say, yeni atama veya üsse dönüş
                    self.visited_targets.add((target_xy[0], target_xy[1]))
                    self.assignment[vid] = None
                    self.drop_triggered[vid] = False
                    self.after_drop_counter[vid] = 0
                    def visited(tx, ty):
                        for (vx, vy) in self.visited_targets:
                            if math.hypot(tx - vx, ty - vy) <= self._assignment_radius_m:
                                return True
                        return False
                    unvisited = [(tx, ty) for (tx, ty) in self.targets if not visited(tx, ty)]
                    if unvisited:
                        wx, wy = target_xy[0], target_xy[1]
                        closest = min(unvisited, key=lambda t: math.hypot(t[0] - wx, t[1] - wy))
                        self.assignment[vid] = closest
                        self.state[vid] = 'ARMING'
                        self.arming_counter[vid] = 0
                        self.get_logger().info(
                            f'Drone {vid} yuku birakti, yeni hedef: ({closest[0]:.1f}, {closest[1]:.1f})'
                        )
                    else:
                        self.state[vid] = 'RETURN_TO_BASE'
                        self.get_logger().info(f'Drone {vid} yuku birakti, hedef kalmadi, usse donus')
                continue

            if st == 'RETURN_TO_BASE':
                # Üs (0,0) yerel NED'de; setpoint ile git, yerdeyse önce arm/offboard
                self._publish_setpoint(vid, 0.0, 0.0, self.transit_altitude_local)
                self.arming_counter[vid] = self.arming_counter.get(vid, 0) + 1
                if self.arming_counter[vid] >= 30:
                    self._publish_command(vid, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self._publish_command(vid, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                dx, dy = pos.x - 0.0, pos.y - 0.0
                dz = pos.z - self.transit_altitude_local
                if math.sqrt(dx*dx + dy*dy) < self.acceptance_xy and abs(dz) < self.acceptance_alt:
                    self._publish_command(vid, VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.state[vid] = 'RTH_LANDED'
                    self.assignment[vid] = (w['spawn_x_world'], w['spawn_y_world'])  # üs konumu (RTH_LANDED'da hedef üs sayılsın)
                    self.get_logger().info(f'Drone {vid} usse indi')
                continue

            if st == 'RTH_LANDED':
                self._publish_setpoint(vid, 0.0, 0.0, 0.0)
                continue


def main(args=None):
    rclpy.init(args=args)
    node = MRTANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
