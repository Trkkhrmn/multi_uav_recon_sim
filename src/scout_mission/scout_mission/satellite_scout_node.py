#!/usr/bin/env python3
# leader drone - hedef uzerinde uyuyor, istege bagli tarama (daire/kare)
# 80m yukseklikte tarama yapiyor

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)


class SatelliteScoutNode(Node):
    def __init__(self):
        super().__init__('satellite_scout_node')

        # parametreler
        self.declare_parameter('vehicle_id', 1)
        self.vehicle_id = int(self.get_parameter('vehicle_id').value)

        # Target: Gazebo world coords (where to hover above)
        self.declare_parameter('target_x_world', -0.16)
        self.declare_parameter('target_y_world', 0.33)
        self.declare_parameter('satellite_altitude', -80.0)

        # Enemy zone box (Gazebo world): this box is scanned exactly, we don't leave it
        # Corners: (-70.44, 64.68), (77.17, 64.68), (-70.44, -64.58), (77.17, -64.58) — X=East, Y=North
        self.declare_parameter('scan_zone_world_min_x', -70.44)
        self.declare_parameter('scan_zone_world_max_x', 77.17)
        self.declare_parameter('scan_zone_world_min_y', -64.58)
        self.declare_parameter('scan_zone_world_max_y', 64.68)
        self.declare_parameter('use_scan_zone', True)   # True = use the box above, target/lawnmower override

        # Leader drone spawn (home)
        self.declare_parameter('spawn_x_world', 185.41)
        self.declare_parameter('spawn_y_world', 56.52)
        # Transit: climb to this alt first (avoid terrain), then go to zone centre
        self.declare_parameter('transit_altitude_m', 50.0)
        # Zone centre (world coords); if not set we use scan_zone centre
        self.declare_parameter('zone_center_world_x', None)
        self.declare_parameter('zone_center_world_y', None)
        # False = just wait at zone centre, no scan (no lawnmower)
        self.declare_parameter('scan_enable', False)

        # Fixed square: world (0,0) centre, 20 fwd/right, 40 back/left/fwd, 20 right, repeat. Step waypoints = smooth path
        self.declare_parameter('patrol_mode', 'fixed_square')
        self.declare_parameter('square_center_world_x', 0.0)   # Gazebo world X (East); centre point
        self.declare_parameter('square_center_world_y', 0.0)   # Gazebo world Y (North)
        self.declare_parameter('square_acceptance_radius', 2.5)
        self.declare_parameter('square_step_m', 5.0)            # Step between waypoints along the path
        self.declare_parameter('square_speed', 0.5)             # m/s; constant speed = smooth camera (no waypoint push)

        # Orbit scan (when patrol_mode == 'orbit')
        self.declare_parameter('orbit_scan_enable', True)
        self.declare_parameter('orbit_radius', 60.0)       # metres (radius); diameter = 2*radius
        self.declare_parameter('orbit_shape', 'lawnmower') # 'circle', 'square', 'lawnmower'
        self.declare_parameter('orbit_waypoints', 48)
        self.declare_parameter('orbit_acceptance_radius', 2.0)
        self.declare_parameter('orbit_direction', 1)      # 1=CCW, -1=CW (circle only)
        # Lawnmower: rectangle around centre, zigzag lanes (covers jet+tank area)
        self.declare_parameter('lawnmower_half_width_north', 80.0)   # North half-width (m)
        self.declare_parameter('lawnmower_half_height_east', 80.0)   # East half-height (m)
        self.declare_parameter('lawnmower_lane_spacing', 30.0)       # Lane spacing (m)
        self.declare_parameter('lawnmower_speed', 1.1)               # m/s (faster scan)
        self.declare_parameter('orbit_control_mode', 'velocity')
        self.declare_parameter('orbit_speed', 0.55)       # m/s tangent speed in velocity mode; 0.5–0.6 keeps overshoot low
        self.declare_parameter('orbit_altitude_kp', 0.05) # altitude P gain (velocity mode)

        self.target_x_world = self.get_parameter('target_x_world').value
        self.target_y_world = self.get_parameter('target_y_world').value
        self.satellite_altitude = self.get_parameter('satellite_altitude').value
        self.spawn_x_world = self.get_parameter('spawn_x_world').value
        self.spawn_y_world = self.get_parameter('spawn_y_world').value
        self.transit_altitude_m = self.get_parameter('transit_altitude_m').value
        zone_cx = self.get_parameter('zone_center_world_x').value
        zone_cy = self.get_parameter('zone_center_world_y').value
        self.scan_enable = self.get_parameter('scan_enable').value
        self.orbit_scan_enable = self.get_parameter('orbit_scan_enable').value
        self.orbit_radius = self.get_parameter('orbit_radius').value
        self.orbit_shape = self.get_parameter('orbit_shape').value
        self.orbit_waypoints = int(self.get_parameter('orbit_waypoints').value)
        self.orbit_acceptance_radius = self.get_parameter('orbit_acceptance_radius').value
        self.orbit_direction = int(self.get_parameter('orbit_direction').value)
        self.lawnmower_half_width_north = self.get_parameter('lawnmower_half_width_north').value
        self.lawnmower_half_height_east = self.get_parameter('lawnmower_half_height_east').value
        self.lawnmower_lane_spacing = self.get_parameter('lawnmower_lane_spacing').value
        self.lawnmower_speed = self.get_parameter('lawnmower_speed').value
        use_scan_zone = self.get_parameter('use_scan_zone').value
        sz_min_x = self.get_parameter('scan_zone_world_min_x').value
        sz_max_x = self.get_parameter('scan_zone_world_max_x').value
        sz_min_y = self.get_parameter('scan_zone_world_min_y').value
        sz_max_y = self.get_parameter('scan_zone_world_max_y').value
        self.orbit_control_mode = self.get_parameter('orbit_control_mode').value
        self.orbit_speed = self.get_parameter('orbit_speed').value
        self.orbit_altitude_kp = self.get_parameter('orbit_altitude_kp').value
        self.patrol_mode = self.get_parameter('patrol_mode').value
        self.square_center_world_x = self.get_parameter('square_center_world_x').value
        self.square_center_world_y = self.get_parameter('square_center_world_y').value
        self.square_acceptance_radius = self.get_parameter('square_acceptance_radius').value
        self.square_step_m = max(1.0, self.get_parameter('square_step_m').value)
        self.square_speed = max(0.2, self.get_parameter('square_speed').value)

        # Centre world (0,0) -> local NED
        self.square_center_local_x = self.square_center_world_y - self.spawn_y_world   # North
        self.square_center_local_y = self.square_center_world_x - self.spawn_x_world   # East
        self.fixed_square_waypoints = []  # filled by _build_fixed_square_waypoints()
        self.square_index = 0

        # World -> PX4 local NED (zone_center = zone centre, transit target)
        self.transit_altitude_local = -abs(self.transit_altitude_m)
        if self.patrol_mode == 'fixed_square':
            self.target_local_x = self.square_center_local_x
            self.target_local_y = self.square_center_local_y
            self.zone_center_local_x = self.square_center_local_x
            self.zone_center_local_y = self.square_center_local_y
            self._build_fixed_square_waypoints()
            self.get_logger().info(
                f"Fixed square patrol: centre ({self.square_center_world_x},{self.square_center_world_y}), "
                f"yerel ({self.square_center_local_x:.1f},{self.square_center_local_y:.1f}), "
                f"{len(self.fixed_square_waypoints)} wp, adim {self.square_step_m}m"
            )
        elif use_scan_zone and sz_min_x < sz_max_x and sz_min_y < sz_max_y:
            center_x_world = (zone_cx if zone_cx is not None else (sz_min_x + sz_max_x) / 2.0)
            center_y_world = (zone_cy if zone_cy is not None else (sz_min_y + sz_max_y) / 2.0)
            self.zone_center_local_x = center_y_world - self.spawn_y_world   # North
            self.zone_center_local_y = center_x_world - self.spawn_x_world   # East
            self.target_local_x = self.zone_center_local_x
            self.target_local_y = self.zone_center_local_y
            self.lawnmower_half_width_north = (sz_max_y - sz_min_y) / 2.0
            self.lawnmower_half_height_east = (sz_max_x - sz_min_x) / 2.0
            self.get_logger().info(
                f"Tarama zonu X=[{sz_min_x},{sz_max_x}], Y=[{sz_min_y},{sz_max_y}], "
                f"local centre N={self.zone_center_local_x:.1f} E={self.zone_center_local_y:.1f}"
            )
        else:
            self.target_local_x = self.target_y_world - self.spawn_y_world
            self.target_local_y = self.target_x_world - self.spawn_x_world
            self.zone_center_local_x = self.target_local_x
            self.zone_center_local_y = self.target_local_y
            self.get_logger().info(
                f"Hedef dunya X={self.target_x_world} Y={self.target_y_world} -> "
                f"yerel N={self.target_local_x:.2f} E={self.target_local_y:.2f} Z={self.satellite_altitude}"
            )

        # ros2 topic'ler
        self.topic_prefix = f'px4_{self.vehicle_id}/' if self.vehicle_id > 0 else ''
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, f'{self.topic_prefix}fmu/in/offboard_control_mode', qos)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, f'{self.topic_prefix}fmu/in/trajectory_setpoint', qos)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, f'{self.topic_prefix}fmu/in/vehicle_command', qos)

        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, f'{self.topic_prefix}fmu/out/vehicle_local_position_v1',
            self.vehicle_local_position_callback, qos)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, f'{self.topic_prefix}fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, qos)

        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.mission_state = 'INIT'
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.counter = 0
        self.orbit_waypoints_list = []
        self.orbit_index = 0
        self.lawnmower_path = []   # (north, east) list
        self.lawnmower_segment = 0

        self.timer = self.create_timer(0.1, self.timer_callback)

    # kare patrol icin waypoint listesi - merkezden baslayip donuyor
    def _build_fixed_square_waypoints(self):
        cx, cy = self.square_center_local_x, self.square_center_local_y
        step = self.square_step_m
        # Corner offsets from centre: fwd=+N, right=+E
        corners = [(0, 0), (40, 0), (40, 40), (-40, 40), (-40, -40), (40, -40), (40, 0), (0, 0)]
        path = []
        for i in range(len(corners) - 1):
            ax, ay = corners[i][0], corners[i][1]
            bx, by = corners[i + 1][0], corners[i + 1][1]
            dx, dy = bx - ax, by - ay
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.1:
                path.append((cx + bx, cy + by))
                continue
            n_steps = max(1, int(round(dist / step)))
            for k in range(1, n_steps + 1):
                t = k / n_steps
                px = cx + (ax + t * dx)
                py = cy + (ay + t * dy)
                path.append((px, py))
        self.fixed_square_waypoints = path

    # daire veya kare orbit waypoint'leri
    def _build_orbit_waypoints(self):
        cx, cy = self.target_local_x, self.target_local_y
        z = self.satellite_altitude
        r = self.orbit_radius
        n = max(4, self.orbit_waypoints)
        waypoints = []
        if self.orbit_shape == 'circle':
            for i in range(n):
                angle = 2.0 * math.pi * i / n
                waypoints.append((cx + r * math.cos(angle), cy + r * math.sin(angle), z))
        else:  # square
            half = r
            pts_per_side = max(1, n // 4)
            for side in range(4):
                for j in range(pts_per_side):
                    t = j / max(1, pts_per_side - 1) if pts_per_side > 1 else 1
                    if side == 0:   # North: (cx-half) -> (cx+half), y = cy-half
                        waypoints.append((cx - half + 2 * half * t, cy - half, z))
                    elif side == 1:  # East: x = cx+half, y (cy-half) -> (cy+half)
                        waypoints.append((cx + half, cy - half + 2 * half * t, z))
                    elif side == 2:  # South: (cx+half) -> (cx-half), y = cy+half
                        waypoints.append((cx + half - 2 * half * t, cy + half, z))
                    else:            # West: x = cx-half, y (cy+half) -> (cy-half)
                        waypoints.append((cx - half, cy + half - 2 * half * t, z))
        return waypoints

    # zigzag tarama yolu (lawnmower)
    # zigzag tarama yolu (lawnmower)
    def _build_lawnmower_path(self):
        cx, cy = self.target_local_x, self.target_local_y
        hw, hh = self.lawnmower_half_width_north, self.lawnmower_half_height_east
        lane = self.lawnmower_lane_spacing
        path = []
        x = cx - hw  # North
        y = cy - hh  # East
        path.append((x, y))
        direction = 1  # 1 = East, -1 = West
        while x <= cx + hw + 0.1:
            if direction == 1:
                path.append((x, cy + hh))
                x += lane
                if x > cx + hw:
                    break
                path.append((x, cy + hh))
                direction = -1
            else:
                path.append((x, cy - hh))
                x += lane
                if x > cx + hw:
                    break
                path.append((x, cy - hh))
                direction = 1
        return path

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def _timestamp_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def publish_offboard_control_mode(self, velocity_mode=False):
        msg = OffboardControlMode()
        msg.position = not velocity_mode  # position mode: position=True
        msg.velocity = velocity_mode     # velocity mode: velocity=True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self._timestamp_us()
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = np.array([float(x), float(y), float(z)], dtype=np.float32)
        msg.velocity = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        msg.yaw = float(yaw)
        msg.timestamp = self._timestamp_us()
        self.trajectory_setpoint_publisher.publish(msg)

    # hiz modunda komut (orbit icin) - yukseklik P ile tutuluyor
    def publish_trajectory_setpoint_velocity(self, vx, vy, z_target):
        z = self.vehicle_local_position.z
        vz = -self.orbit_altitude_kp * (z - z_target)  # P: pull to target alt
        msg = TrajectorySetpoint()
        msg.position = np.array([np.nan, np.nan, np.nan], dtype=np.float32)  # velocity-only: position not used
        msg.velocity = np.array([float(vx), float(vy), float(vz)], dtype=np.float32)
        msg.yaw = float('nan')
        msg.timestamp = self._timestamp_us()
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = self.vehicle_id + 1  # PX4 multi: MAV_SYS_ID = px4_instance+1 (1→2, 2→3, ...)
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._timestamp_us()
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def timer_callback(self):
        # Velocity mode: orbit (circle/lawnmower) or fixed square — constant velocity instead of position waypoints = smoother
        use_velocity = (
            self.mission_state == 'SQUARE_PATROL'
            or (
                self.mission_state == 'ORBIT_SCAN'
                and self.orbit_control_mode == 'velocity'
                and (self.orbit_shape == 'circle' or self.orbit_shape == 'lawnmower')
            )
        )
        self.publish_offboard_control_mode(velocity_mode=use_velocity)

        # akis: once 50m cik, zone merkezine git, 80m ye cik, sonra tarama veya bekle
        accept_alt_m = 3.0
        accept_xy_m = 4.0

        if self.mission_state == 'INIT':
            # Aim for 50m at spawn (climb first so we don't hit terrain)
            self.publish_trajectory_setpoint(0.0, 0.0, self.transit_altitude_local)
            if self.counter > 30:
                self.engage_offboard_mode()
                self.mission_state = 'WAIT_FOR_OFFBOARD'
            self.counter += 1

        elif self.mission_state == 'WAIT_FOR_OFFBOARD':
            self.publish_trajectory_setpoint(0.0, 0.0, self.transit_altitude_local)
            if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.get_logger().info('Offboard on, climbing to 50m at spawn')
                self.arm()
                self.mission_state = 'CLIMB_50M'
            else:
                if self.counter % 20 == 0:
                    self.engage_offboard_mode()
            self.counter += 1

        elif self.mission_state == 'CLIMB_50M':
            self.publish_trajectory_setpoint(0.0, 0.0, self.transit_altitude_local)
            dz = self.vehicle_local_position.z - self.transit_altitude_local
            if abs(dz) < accept_alt_m:
                self.get_logger().info(
                    f'50m done, going to zone centre: local ({self.zone_center_local_x:.1f}, {self.zone_center_local_y:.1f})'
                )
                self.mission_state = 'TRANSIT_TO_ZONE'

        elif self.mission_state == 'TRANSIT_TO_ZONE':
            # Go to zone centre, stay at 50m
            self.publish_trajectory_setpoint(
                self.zone_center_local_x, self.zone_center_local_y, self.transit_altitude_local
            )
            dx = self.vehicle_local_position.x - self.zone_center_local_x
            dy = self.vehicle_local_position.y - self.zone_center_local_y
            dz = self.vehicle_local_position.z - self.transit_altitude_local
            if math.sqrt(dx*dx + dy*dy) < accept_xy_m and abs(dz) < accept_alt_m:
                self.get_logger().info('At zone centre (50m), climbing to 80m')
                self.mission_state = 'CLIMB_80M'

        elif self.mission_state == 'CLIMB_80M':
            self.publish_trajectory_setpoint(
                self.zone_center_local_x, self.zone_center_local_y, self.satellite_altitude
            )
            dx = self.vehicle_local_position.x - self.zone_center_local_x
            dy = self.vehicle_local_position.y - self.zone_center_local_y
            dz = self.vehicle_local_position.z - self.satellite_altitude
            if math.sqrt(dx*dx + dy*dy) < accept_xy_m and abs(dz) < accept_alt_m:
                if self.scan_enable and self.patrol_mode == 'fixed_square' and self.fixed_square_waypoints:
                    self.square_index = 0
                    self.get_logger().info(f"80m done, square patrol starting ({len(self.fixed_square_waypoints)} wp)")
                    self.mission_state = 'SQUARE_PATROL'
                elif self.scan_enable and self.orbit_scan_enable and (self.orbit_shape in ('circle', 'square', 'lawnmower')):
                    self.orbit_index = 0
                    if self.orbit_shape == 'lawnmower' and self.orbit_control_mode == 'velocity':
                        self.lawnmower_path = self._build_lawnmower_path()
                        self.lawnmower_segment = 0
                        self.orbit_waypoints_list = []
                        self.get_logger().info(f"80m done, lawnmower {len(self.lawnmower_path)} points")
                    elif self.orbit_control_mode == 'velocity' and self.orbit_shape == 'circle':
                        self.orbit_waypoints_list = []
                        self.get_logger().info(f"80m done, circle orbit radius {self.orbit_radius}m")
                    else:
                        self.orbit_waypoints_list = self._build_orbit_waypoints()
                        self.get_logger().info(f"80m done, orbit {self.orbit_shape} {len(self.orbit_waypoints_list)} wp")
                    self.mission_state = 'ORBIT_SCAN'
                else:
                    self.get_logger().info(
                        f'80m done, holding at zone centre (scan off), '
                        f'local ({self.zone_center_local_x:.1f}, {self.zone_center_local_y:.1f})'
                    )
                    self.mission_state = 'HOLD'

        elif self.mission_state == 'HOLD':
            self.publish_trajectory_setpoint(
                self.zone_center_local_x, self.zone_center_local_y, self.satellite_altitude
            )

        elif self.mission_state == 'SQUARE_PATROL' and self.fixed_square_waypoints:
            # Go with velocity: constant speed toward target, no waypoint "push" — smooth for camera
            x, y = self.vehicle_local_position.x, self.vehicle_local_position.y
            wp = self.fixed_square_waypoints[self.square_index]
            tx, ty = wp[0], wp[1]
            dx, dy = tx - x, ty - y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < self.square_acceptance_radius:
                self.square_index = (self.square_index + 1) % len(self.fixed_square_waypoints)
                if self.square_index == 0:
                    self.get_logger().info('Square lap done, starting over')
            else:
                # Direction unit vector * constant speed
                inv = 1.0 / max(dist, 0.01)
                vx = dx * inv * self.square_speed
                vy = dy * inv * self.square_speed
                self.publish_trajectory_setpoint_velocity(vx, vy, self.satellite_altitude)

        elif self.mission_state == 'ORBIT_SCAN':
            cx, cy = self.target_local_x, self.target_local_y
            x, y = self.vehicle_local_position.x, self.vehicle_local_position.y

            if self.orbit_control_mode == 'velocity' and self.orbit_shape == 'lawnmower' and len(self.lawnmower_path) >= 2:
                path = self.lawnmower_path
                seg = min(self.lawnmower_segment, len(path) - 2)
                p0, p1 = path[seg], path[seg + 1]
                dx_seg = p1[0] - p0[0]
                dy_seg = p1[1] - p0[1]
                dist_seg = math.sqrt(dx_seg * dx_seg + dy_seg * dy_seg)
                if dist_seg < 0.1:
                    self.lawnmower_segment = (seg + 1) % max(1, len(path) - 1)
                else:
                    vx = (dx_seg / dist_seg) * self.lawnmower_speed
                    vy = (dy_seg / dist_seg) * self.lawnmower_speed
                    self.publish_trajectory_setpoint_velocity(vx, vy, self.satellite_altitude)
                    dist_to_end = math.sqrt((x - p1[0]) ** 2 + (y - p1[1]) ** 2)
                    if dist_to_end < self.orbit_acceptance_radius:
                        self.lawnmower_segment = seg + 1
                        if self.lawnmower_segment >= len(path) - 1:
                            self.lawnmower_segment = 0

            elif self.orbit_control_mode == 'velocity' and self.orbit_shape == 'circle':
                # Tangent velocity for circle; orbit_direction 1=CCW, -1=CW
                dx, dy = x - cx, y - cy
                r = math.sqrt(dx * dx + dy * dy)
                r = max(r, 0.5)
                vx = -self.orbit_direction * self.orbit_speed * dy / r
                vy = self.orbit_direction * self.orbit_speed * dx / r
                self.publish_trajectory_setpoint_velocity(vx, vy, self.satellite_altitude)
            else:
                wp = self.orbit_waypoints_list[self.orbit_index]
                self.publish_trajectory_setpoint(wp[0], wp[1], wp[2])
                dx = x - wp[0]
                dy = y - wp[1]
                if math.sqrt(dx * dx + dy * dy) < self.orbit_acceptance_radius:
                    self.orbit_index = (self.orbit_index + 1) % len(self.orbit_waypoints_list)
                    if self.orbit_index % 12 == 0:
                        self.get_logger().info(f"Orbit wp {self.orbit_index}/{len(self.orbit_waypoints_list)}")

        elif self.mission_state == 'SATELLITE_MODE':
            self.publish_trajectory_setpoint(
                self.target_local_x, self.target_local_y, self.satellite_altitude
            )


def main(args=None):
    rclpy.init(args=args)
    node = SatelliteScoutNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
