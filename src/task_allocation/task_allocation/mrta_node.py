#!/usr/bin/env python3
# MRTA node - scoutlar hedef verince worker drone'lar ataniyor
# fused_targets topic'inden hedefleri aliyoruz, 6 drone'a gorev dagitiyoruz
# biraz internetten baktim nasil yapiliyor diye

import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, Pose, PoseArray, PointStamped
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)


# dunya koordundan px4 local NED'e cevirme (hocanin slaytindaki gibi)
def world_to_local_ned(world_x, world_y, spawn_x_world, spawn_y_world):
    local_north = world_y - spawn_y_world
    local_east = world_x - spawn_x_world
    return local_north, local_east


def local_to_world(local_north, local_east, spawn_x_world, spawn_y_world):
    world_x = spawn_x_world + local_east
    world_y = spawn_y_world + local_north
    return world_x, world_y


# hedef pozisyonu key yapmak icin yuvarluyorum ki set'te tutabileyim
def _target_key(x, y):
    return (round(float(x), 1), round(float(y), 1))


class MRTANode(Node):
    def __init__(self):
        super().__init__('mrta_node')

        self.declare_parameter('fused_targets_topic', '/scout/fused_targets')
        self.declare_parameter('transit_altitude_m', 50.0)
        self.declare_parameter('drop_altitude_m', 10.0)  # Descend to this alt then drop (no full landing)
        self.declare_parameter('drop_hover_after_s', 1.0)  # Hover this long after drop, then climb
        self.declare_parameter('acceptance_radius_xy_m', 3.0)
        self.declare_parameter('acceptance_radius_alt_m', 2.0)
        self.declare_parameter('start_delay_s', 5.0)
        self.declare_parameter('task_done_delay_s', 5.0)
        self.declare_parameter('resupply_seconds_per_package', 5.0)  # Seconds per package at resupply
        self.declare_parameter('spawn_entity_service', '/spawn_entity')
        self.declare_parameter('payload_sdf_path', '')  # If empty we use inline simple box
        self.declare_parameter('publish_markers', True)  # MRTA viz in RViz
        self.declare_parameter('markers_topic', '/mrta/markers')
        self.declare_parameter('status_topic', '/mrta/status')
        self.declare_parameter('marker_frame_id', 'world')
        self.declare_parameter('worker_vehicle_ids', [5, 6, 7, 8, 9, 10])
        self.declare_parameter('worker_spawn_x_world', [183.41, 185.41, 187.41, 189.41, 191.41, 193.41])
        self.declare_parameter('worker_spawn_y_world', [54.52, 54.52, 54.52, 54.52, 54.52, 54.52])
        self.declare_parameter('worker_max_packages', [3, 3, 2, 2, 1, 1])
        self.declare_parameter('worker_speed_scale', [0.5, 0.5, 0.7, 0.7, 1.0, 1.0])
        self.declare_parameter('target_completed_topic', '/mrta/target_completed')

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
        self.resupply_seconds_per_package = float(self.get_parameter('resupply_seconds_per_package').value)
        self.target_completed_topic = self.get_parameter('target_completed_topic').value

        # hedef tipine gore kac paket lazim: mavi=2, magenta=1, cyan=3 (bunu yaml'dan da alabilirdim aslinda)
        self.required_packages_by_type = {0: 2, 1: 1, 2: 3}

        ids = self.get_parameter('worker_vehicle_ids').value
        xs = self.get_parameter('worker_spawn_x_world').value
        ys = self.get_parameter('worker_spawn_y_world').value
        max_pkgs = self.get_parameter('worker_max_packages').value
        speed_scales = self.get_parameter('worker_speed_scale').value
        if not isinstance(ids, list): ids = []
        if not isinstance(xs, list): xs = []
        if not isinstance(ys, list): ys = []
        if not isinstance(max_pkgs, list): max_pkgs = [1] * len(ids)
        if not isinstance(speed_scales, list): speed_scales = [1.0] * len(ids)
        n = min(len(ids), len(xs), len(ys), len(max_pkgs), len(speed_scales))
        self.workers = []
        for i in range(n):
            vid = int(ids[i])
            sx, sy = float(xs[i]), float(ys[i])
            mp = int(max_pkgs[i]) if i < len(max_pkgs) else 1
            sp = float(speed_scales[i]) if i < len(speed_scales) else 1.0
            self.workers.append({
                'vehicle_id': vid,
                'spawn_x_world': sx,
                'spawn_y_world': sy,
                'max_packages': max(1, mp),
                'speed_scale': max(0.1, min(1.0, sp)),
            })

        if not self.workers:
            self.get_logger().warn('worker_drones bos, config dosyasini kontrol et.')

        # hedefler listesi - her biri (x, y, tip)
        self.targets = []
        self.positions = {}
        self.assignment = {}  # vehicle_id -> (world_x, world_y, target_type) assigned target
        self.state = {}  # IDLE | ARMING | CLIMB | GO | ... | RETURN_TO_BASE | RESUPPLY_WAIT | RTH_LANDED
        self.arming_counter = {}
        self.land_time = {}
        self.drop_triggered = {}
        self.after_drop_counter = {}
        self.packages_remaining = {}   # vehicle_id -> int
        self.resupply_start_time = {}  # vehicle_id -> rclpy.time.Time
        self.return_reason = {}       # vehicle_id -> 'resupply' | 'all_done'
        self.visited_targets = set()  # (x,y) visited (one package dropped)
        self._dropped_at = set()      # (vehicle_id, cell) for one drop per cell
        self.drops_at_target = {}     # (cell_x, cell_y) -> num packages dropped
        self.target_type_at_cell = {} # (cell_x, cell_y) -> target_type (0,1,2)
        self.completed_targets = set()  # (x,y) completed targets (enough packages)
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
            self.packages_remaining[vid] = w['max_packages']
            self.resupply_start_time[vid] = None
            self.return_reason[vid] = None
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
        self._pub_target_completed = self.create_publisher(PointStamped, self.target_completed_topic, 10)
        if self.publish_markers:
            self._pub_markers = self.create_publisher(MarkerArray, self.markers_topic, 10)
            self._marker_timer = self.create_timer(0.5, self._publish_mrta_visualization)  # 2 Hz: markers + status
        else:
            self._pub_markers = None
            self._marker_timer = None
        self._pub_status = self.create_publisher(String, self.status_topic, 10)
        self.get_logger().info(
            f'MRTA started: {len(self.workers)} drones, targets: {self.fused_topic}, '
            f'transit {self.transit_altitude_m}m, birakma {self.drop_altitude_m}m'
        )

    def _make_position_cb(self, vehicle_id):
        def cb(msg):
            self.positions[vehicle_id] = msg
        return cb

    def _cb_fused(self, msg):
        # orientation.x = target type (0=blue, 1=magenta, 2=cyan)
        self.targets = []
        for p in msg.poses:
            tt = int(round(p.orientation.x))
            if tt < 0: tt = 0
            if tt > 2: tt = 2
            self.targets.append((float(p.position.x), float(p.position.y), tt))
        if self.targets and self.start_time is None:
            self.start_time = self.get_clock().now()
        if self.targets:
            self.get_logger().info(f'Target list updated: {len(self.targets)} targets (typed)')

    def _timestamp_us(self):
        t = self.get_clock().now()
        return t.nanoseconds // 1000

    # yuk olarak birakilacak modelin sdf'i - dosyadan ya da hazir kutu
    def _get_payload_sdf(self):
        if self._payload_sdf_cache is not None:
            return self._payload_sdf_cache
        path = (self.payload_sdf_path or '').strip()
        if path and os.path.isfile(os.path.expanduser(path)):
            with open(os.path.expanduser(path), 'r') as f:
                self._payload_sdf_cache = f.read()
            return self._payload_sdf_cache
        # Inline: small dynamic box (no mesh, primitive)
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

    # gazeboda bu noktaya yuk spawn ediyor (yercekimi ile dusuyor)
    def _drop_payload_at(self, world_x, world_y, world_z):
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

    # rviz icin markerlari ve statusu yayinliyor
    def _publish_mrta_visualization(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        if self._pub_markers:
            self._publish_mrta_markers(stamp)
        self._publish_mrta_status()

    # rvizde hedefleri, cizgileri ve drone durumlarini gosteriyor
    def _publish_mrta_markers(self, stamp):
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

        # 1) Targets: type colour (blue/green/red), grey = completed
        for i, tup in enumerate(self.targets):
            tx, ty = tup[0], tup[1]
            tt = tup[2] if len(tup) > 2 else 0
            tkey = _target_key(tx, ty)
            completed = tkey in self.completed_targets or self.drops_at_target.get(tkey, 0) >= self.required_packages_by_type.get(tt, 3)
            z = self.drop_altitude_m if not completed else 0.0
            if completed:
                add_marker('targets_completed', (2.5, 2.5, 2.5), (0.5, 0.5, 0.5, 0.7), (tx, ty, z))
            else:
                if tt == 0:   color = (0.2, 0.2, 1.0, 0.9)   # mavi
                elif tt == 1: color = (0.85, 0.0, 0.85, 0.9)   # magenta
                else:        color = (0.0, 0.85, 0.85, 0.9)  # cyan
                add_marker('targets_pending', (3.0, 3.0, 1.0), color, (tx, ty, self.drop_altitude_m))

        # 2) Assignment line: drone pos -> assigned target (yellow line)
        for w in self.workers:
            vid = w['vehicle_id']
            pos = self.positions.get(vid)
            target_xy = self.assignment.get(vid)
            if pos is None or target_xy is None:
                continue
            wx = w['spawn_x_world'] + pos.y
            wy = w['spawn_y_world'] + pos.x
            wz = -float(pos.z)
            tx, ty = target_xy[0], target_xy[1]
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

        # 3) Drone state labels: just above position "vid: STATE"
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
            pkg = self.packages_remaining.get(vid, 0)
            m.text = f'W{vid}: {st} ({pkg}p)'
            ma.markers.append(m)

        self._pub_markers.publish(ma)

    # panel icin json status - hangi drone nerede hangi hedefe gidiyor
    def _publish_mrta_status(self):
        import json
        parts = []
        json_workers = []
        type_labels = {0: 'BLUE', 1: 'MAGENTA', 2: 'CYAN'}
        for w in self.workers:
            vid = w['vehicle_id']
            st = self.state.get(vid, 'IDLE')
            target_xy = self.assignment.get(vid)
            pkg_rem = self.packages_remaining.get(vid, 0)
            max_pkg = w['max_packages']
            pos = self.positions.get(vid)
            wx, wy, wz = 0.0, 0.0, 0.0
            if pos is not None:
                wx = w['spawn_x_world'] + pos.y
                wy = w['spawn_y_world'] + pos.x
                wz = -float(pos.z)
            entry = {
                'id': vid,
                'state': st,
                'packages': f'{pkg_rem}/{max_pkg}',
                'pkg_remaining': pkg_rem,
                'pkg_max': max_pkg,
                'speed_scale': w['speed_scale'],
                'pos_x': round(wx, 1),
                'pos_y': round(wy, 1),
                'pos_z': round(wz, 1),
                'target_x': None,
                'target_y': None,
                'target_type': None,
                'target_type_label': None,
            }
            if target_xy:
                tx, ty = target_xy[0], target_xy[1]
                tt = target_xy[2] if len(target_xy) > 2 else 0
                entry['target_x'] = round(tx, 1)
                entry['target_y'] = round(ty, 1)
                entry['target_type'] = tt
                entry['target_type_label'] = type_labels.get(tt, '?')
                parts.append(f'W{vid}→({tx:.0f},{ty:.0f}) {st}')
            else:
                parts.append(f'W{vid} {st}')
            json_workers.append(entry)
        # Targets summary
        completed_count = len(self.completed_targets)
        total_targets = len(self.targets)
        pending_targets = total_targets - completed_count
        # Target list (for map panel)
        json_targets = []
        for tup in self.targets:
            tx, ty = tup[0], tup[1]
            tt = tup[2] if len(tup) > 2 else 0
            tkey = _target_key(tx, ty)
            drops = self.drops_at_target.get(tkey, 0)
            req = self.required_packages_by_type.get(tt, 3)
            completed = tkey in self.completed_targets
            if tt == 2:
                priority = 'PRIORITY'
            elif tt == 0:
                priority = 'NORMAL'
            else:
                priority = 'DECOY'
            json_targets.append({
                'x': round(tx, 1), 'y': round(ty, 1),
                'type': tt, 'drops': drops, 'required': req,
                'completed': completed,
                'priority': priority,
            })
        # Base position (spawn average)
        base_x = sum(w['spawn_x_world'] for w in self.workers) / max(len(self.workers), 1)
        base_y = sum(w['spawn_y_world'] for w in self.workers) / max(len(self.workers), 1)
        json_data = {
            'workers': json_workers,
            'targets': json_targets,
            'targets_total': total_targets,
            'targets_completed': completed_count,
            'targets_pending': pending_targets,
            'base_x': round(base_x, 1),
            'base_y': round(base_y, 1),
        }
        msg = String()
        msg.data = json.dumps(json_data)
        self._pub_status.publish(msg)

    # drone'lari hedeflere atama - kapasiteye gore en uygun hedefe veriyorum, yoksa en yakin
    def _run_assignment(self):
        if not self.targets or not self.workers:
            return
        if not all(self.positions.get(w['vehicle_id']) is not None for w in self.workers):
            return

        # Assigned target keys
        assigned_keys = set()
        for vid, a in self.assignment.items():
            if a is not None:
                assigned_keys.add(_target_key(a[0], a[1]))

        # Incomplete and unassigned targets
        unassigned_targets = []
        for tup in self.targets:
            tx, ty, ttype = tup[0], tup[1], tup[2] if len(tup) > 2 else 0
            tkey = _target_key(tx, ty)
            if tkey in self.completed_targets:
                continue
            required = self.required_packages_by_type.get(ttype, 3)
            if self.drops_at_target.get(tkey, 0) >= required:
                continue
            if tkey in assigned_keys:
                continue
            unassigned_targets.append((tx, ty, ttype))

        # IDLE drones that have packages
        idle_workers = [
            w for w in self.workers
            if self.state.get(w['vehicle_id']) in ('IDLE', 'RTH_LANDED')
            and self.packages_remaining.get(w['vehicle_id'], 0) > 0
        ]
        if not unassigned_targets or not idle_workers:
            return

        # Capacity -> preferred target type
        capacity_to_preferred_type = {3: 2, 2: 0, 1: 1}

        for w in list(idle_workers):
            vid = w['vehicle_id']
            pos = self.positions[vid]
            wx, wy = local_to_world(pos.x, pos.y, w['spawn_x_world'], w['spawn_y_world'])
            cap = w['max_packages']
            preferred_type = capacity_to_preferred_type.get(cap, None)

            preferred_targets = [
                t for t in unassigned_targets
                if t[2] == preferred_type and _target_key(t[0], t[1]) not in assigned_keys
            ] if preferred_type is not None else []

            any_targets = [
                t for t in unassigned_targets
                if _target_key(t[0], t[1]) not in assigned_keys
            ]

            candidates = preferred_targets if preferred_targets else any_targets
            if not candidates:
                continue

            best = min(candidates, key=lambda t: math.hypot(t[0] - wx, t[1] - wy))
            tx, ty, ttype = best
            tkey = _target_key(tx, ty)

            self.assignment[vid] = (tx, ty, ttype)
            self.state[vid] = 'ARMING'
            self.arming_counter[vid] = 0
            assigned_keys.add(tkey)
            unassigned_targets.remove(best)
            idle_workers.remove(w)

            match_str = 'ESLESME' if preferred_type == ttype else 'YEDEK'
            self.get_logger().info(
                f'Assign [{match_str}]: drone W{vid} (cap={cap}) -> target ({tx:.1f}, {ty:.1f}) type={ttype}'
            )

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

        # belirli araliklarla atama yap - idle drone'lari hedeflere ver
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

            target_xy = self.assignment.get(vid)  # (world_x, world_y, target_type) or None
            st = self.state.get(vid, 'IDLE')
            w_dict = next((x for x in self.workers if x['vehicle_id'] == vid), None)
            max_pkg = w_dict['max_packages'] if w_dict else 1

            self._publish_offboard(vid)

            if st == 'IDLE':
                self._publish_setpoint(vid, 0.0, 0.0, self.transit_altitude_local)
                continue

            if st == 'RESUPPLY_WAIT':
                self._publish_setpoint(vid, 0.0, 0.0, self.transit_altitude_local)
                t0 = self.resupply_start_time.get(vid)
                if t0 is not None:
                    elapsed = (now - t0).nanoseconds / 1e9
                    need = self.resupply_seconds_per_package * max_pkg
                    if elapsed >= need:
                        self.packages_remaining[vid] = max_pkg
                        self.resupply_start_time[vid] = None
                        self.return_reason[vid] = None
                        self.state[vid] = 'IDLE'
                        self.get_logger().info(f'Drone {vid} resupplied: {max_pkg} packages')
                continue

            if st == 'ARMING':
                self._publish_setpoint(vid, 0.0, 0.0, self.transit_altitude_local)
                self.arming_counter[vid] = self.arming_counter.get(vid, 0) + 1
                if self.arming_counter[vid] >= 30:  # 3 saniye kadar bekleyip arm ver (px4 oyle istiyor)
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
                tx_w, ty_w = target_xy[0], target_xy[1]
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
                    self.get_logger().info(f'Drone {vid} at target, descending to drop alt ({self.drop_altitude_m}m)')
                    self.state[vid] = 'DESCEND'
                continue

            if st == 'DESCEND' and target_xy:
                tx_w, ty_w = target_xy[0], target_xy[1]
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
                tx_w, ty_w = target_xy[0], target_xy[1]
                ttype = target_xy[2] if len(target_xy) > 2 else 0
                ln, le = world_to_local_ned(
                    tx_w, ty_w, w['spawn_x_world'], w['spawn_y_world']
                )
                self._publish_setpoint(vid, ln, le, self.drop_altitude_local)
                tkey = _target_key(tx_w, ty_w)
                drop_key = (vid, tkey)
                if drop_key not in self._dropped_at:
                    drop_z = self.drop_altitude_m  # birakma yuksekligi
                    self._drop_payload_at(tx_w, ty_w, drop_z)
                    self._dropped_at.add(drop_key)
                    self.packages_remaining[vid] = max(0, self.packages_remaining.get(vid, max_pkg) - 1)
                    self.drops_at_target[tkey] = self.drops_at_target.get(tkey, 0) + 1
                    self.target_type_at_cell[tkey] = ttype
                    required = self.required_packages_by_type.get(ttype, 3)
                    self.get_logger().info(
                        f'Drone W{vid} dropped at target ({tx_w:.1f},{ty_w:.1f}): '
                        f'{self.drops_at_target[tkey]}/{required}'
                    )
                    if self.drops_at_target[tkey] >= required:
                        self.completed_targets.add(tkey)
                        msg = PointStamped()
                        msg.header.stamp = now.to_msg()
                        msg.header.frame_id = 'world'
                        msg.point.x = float(tx_w)
                        msg.point.y = float(ty_w)
                        msg.point.z = 0.0
                        self._pub_target_completed.publish(msg)
                        self.get_logger().info(f'Target COMPLETED: ({tx_w:.1f}, {ty_w:.1f}) type={ttype}')
                self.drop_triggered[vid] = True
                self.after_drop_counter[vid] = 0
                self.state[vid] = 'AFTER_DROP'
                continue

            if st == 'AFTER_DROP' and target_xy:
                tx_w, ty_w = target_xy[0], target_xy[1]
                ln, le = world_to_local_ned(
                    tx_w, ty_w, w['spawn_x_world'], w['spawn_y_world']
                )
                self._publish_setpoint(vid, ln, le, self.drop_altitude_local)
                self.after_drop_counter[vid] = self.after_drop_counter.get(vid, 0) + 1
                if self.after_drop_counter[vid] >= max(1, int(self.drop_hover_after_s * 10)):
                    self.state[vid] = 'CLIMB_AFTER_DROP'
                continue

            if st == 'CLIMB_AFTER_DROP' and target_xy:
                tx_w, ty_w = target_xy[0], target_xy[1]
                ln, le = world_to_local_ned(
                    tx_w, ty_w, w['spawn_x_world'], w['spawn_y_world']
                )
                self._publish_setpoint(vid, ln, le, self.transit_altitude_local)
                dx = pos.x - ln
                dy = pos.y - le
                dz = pos.z - self.transit_altitude_local
                if (math.sqrt(dx*dx + dy*dy) < self.acceptance_xy and
                        abs(dz) < self.acceptance_alt):
                    self.visited_targets.add((tx_w, ty_w))
                    self.assignment[vid] = None
                    self.drop_triggered[vid] = False
                    self.after_drop_counter[vid] = 0
                    assigned_keys_now = set()
                    for v, a in self.assignment.items():
                        if a is not None:
                            assigned_keys_now.add(_target_key(a[0], a[1]))
                    def still_needed(t):
                        tx, ty = t[0], t[1]
                        tt = t[2] if len(t) > 2 else 0
                        tkey = _target_key(tx, ty)
                        if tkey in self.completed_targets or tkey in assigned_keys_now:
                            return False
                        return self.drops_at_target.get(tkey, 0) < self.required_packages_by_type.get(tt, 3)
                    unassigned = [(t[0], t[1], t[2] if len(t) > 2 else 0) for t in self.targets if still_needed(t)]
                    pkg = self.packages_remaining.get(vid, 0)
                    if pkg == 0 and unassigned:
                        self.return_reason[vid] = 'resupply'
                        self.state[vid] = 'RETURN_TO_BASE'
                        self.get_logger().info(f'Drone {vid} out of packages, return to base (resupply)')
                    elif pkg > 0 and unassigned:
                        # Capacity-target match: prefer matching type, else nearest
                        capacity_to_preferred = {3: 2, 2: 0, 1: 1}
                        cap = w_dict['max_packages'] if w_dict else 1
                        pref = capacity_to_preferred.get(cap, None)
                        preferred = [t for t in unassigned if t[2] == pref] if pref is not None else []
                        pool = preferred if preferred else unassigned
                        closest = min(pool, key=lambda t: math.hypot(t[0] - tx_w, t[1] - ty_w))
                        self.assignment[vid] = closest
                        self.state[vid] = 'GO'
                        match = 'ESLESME' if pref == closest[2] else 'YEDEK'
                        self.get_logger().info(
                            f'Drone W{vid} [{match}] new target: ({closest[0]:.1f}, {closest[1]:.1f}) type={closest[2]}'
                        )
                    else:
                        self.return_reason[vid] = 'all_done'
                        self.state[vid] = 'RETURN_TO_BASE'
                        self.get_logger().info(f'Drone {vid} gorev bitti, usse donus')
                continue

            if st == 'RETURN_TO_BASE':
                self._publish_setpoint(vid, 0.0, 0.0, self.transit_altitude_local)
                self.arming_counter[vid] = self.arming_counter.get(vid, 0) + 1
                if self.arming_counter[vid] >= 30:
                    self._publish_command(vid, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    self._publish_command(vid, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                dx, dy = pos.x - 0.0, pos.y - 0.0
                dz = pos.z - self.transit_altitude_local
                if math.sqrt(dx*dx + dy*dy) < self.acceptance_xy and abs(dz) < self.acceptance_alt:
                    reason = self.return_reason.get(vid, 'all_done')
                    if reason == 'resupply':
                        self.state[vid] = 'RESUPPLY_WAIT'
                        self.resupply_start_time[vid] = now
                        self.get_logger().info(f'Drone {vid} at base, resupplying ({max_pkg}x {self.resupply_seconds_per_package}s)')
                    else:
                        self._publish_command(vid, VehicleCommand.VEHICLE_CMD_NAV_LAND)
                        self.state[vid] = 'RTH_LANDED'
                        self.assignment[vid] = None
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
