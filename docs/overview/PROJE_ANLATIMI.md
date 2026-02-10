# How to Explain the Project to Someone

This doc describes **multi_uav_recon_ws** step by step, like you're explaining it to somebody: what it does, what the parts are, how data flows and why it's built this way.

---

## 1. Big picture: what does the project do?

The project is a **multi-UAV recon + target detection + task allocation** simulation. The environment is **Gazebo Classic** and **PX4 SITL**; control and fusion run on **ROS 2 Humble**.

Short flow:

1. **Sim** starts: a map in Gazebo (e.g. mili world), **4 "leaders"** (iris_leader with down-facing camera) and **6 "workers"** (normal iris) are spawned. All fly with PX4.
2. **Leaders** go to their zones and hold at 80 m (optional square/lawnmower/circle scan). Their down cameras see blue targets (cubes etc).
3. **Detection** is per leader: blue blob in image -> pixel coords -> using drone position and camera model we get **world (X, Y)**. These points are published on ROS topics.
4. **Fusion** node listens to the four leaders' detection topics; merges detections in the same area into one target (distance + centroid). Output: one "fused target list" in world coords.
5. **MRTA** (task allocation) takes that list; assigns the 6 worker drones to "nearest unassigned target", they fly there and **drop a payload** at a given altitude (spawn in Gazebo). When targets are done they return to base.

So: **4 drones do recon and produce a target list, 6 drones go to those targets and drop.** All use PX4 offboard, ROS 2 topics and (optional) Micro XRCE-DDS agent.

---

## 2. Infrastructure: sim and coordinates

### 2.1 Gazebo + PX4

- **Gazebo Classic** (gazebo 11) is started with a world file (e.g. mili.world). It has ground, buildings, blue cubes (targets) etc.
- **PX4 SITL** is one process per drone: `px4 -i 1`, `px4 -i 2`, ... PX4 does flight dynamics, sensors, offboard commands; talks to the sim via the Gazebo plugin.
- **Drones** are spawned into Gazebo with SDF (jinja sets ports etc per PX4 instance). 4 leaders = iris_leader (down cam), 6 workers = iris.

Spawn positions are fixed in the script: leaders around 185–189, 56–60; workers around 193–197, 54–64. These numbers have to match **coverage_partitions** and **worker_drones** config; otherwise world <-> local NED transforms are wrong.

### 2.2 Why coordinate transform matters

- **Gazebo world:** X = East, Y = North, Z = Up. Origin depends on the map.
- **PX4 (per drone):** NED — North, East, Down. Origin is that drone's **spawn point**. So for drone 1, (0,0,-50) = "50 m above spawn".
- Detections are kept in **world coordinates** (common reference). MRTA gives target as "world (x,y)"; each worker converts to local NED from its spawn and sends setpoint to PX4.

Formulas:

- World -> a drone's local NED: `local_north = world_y - spawn_y`, `local_east = world_x - spawn_x`. Altitude in NED is negative (e.g. -80 = 80 m up).
- So in config, each drone's `spawn_x_world`, `spawn_y_world` (spawn in Gazebo) must be correct.

### 2.3 Micro XRCE-DDS agent

PX4 SITL doesn't talk to ROS 2 directly; it uses **DDS**. The Micro XRCE-DDS agent connects to PX4 over UDP (e.g. 8888) and bridges DDS to ROS 2. If the agent isn't running, the `px4_X/fmu/in/...` and `px4_X/fmu/out/...` topics in ROS 2 exist but PX4 doesn't get commands; drones don't move.

---

## 3. Packages and what they do

### 3.1 px4_msgs

Message types used by PX4 (VehicleLocalPosition, VehicleStatus, OffboardControlMode, TrajectorySetpoint, VehicleCommand etc) are defined on the ROS 2 side. Other packages use these to send commands to PX4 or read position/status. The msgs live here; building creates the `px4_msgs` package.

### 3.2 scout_mission — satellite_scout_node

Controls **one leader drone**. So for 4 leaders the launch runs 4 nodes.

- **Input:** From PX4 `vehicle_local_position`, `vehicle_status` (ROS 2 topics via the agent).
- **Output:** To the same PX4 `offboard_control_mode`, `trajectory_setpoint`, `vehicle_command` (arm, offboard mode, land etc).

Flow (short):

1. Send position setpoint and request offboard; when PX4 is in offboard, arm.
2. Climb to 50 m (transit), then go to **zone centre** (from config, world coords converted to local NED).
3. Climb to 80 m; optional square / lawnmower / circle scan, or just hold.
4. All targets in NED; setpoints in the format PX4 expects (position or velocity mode).

Parameters: `vehicle_id`, `spawn_x_world`, `spawn_y_world`, `zone_center_world_x/y`, scan zone min/max, altitude, speed, acceptance radius etc. The launch reads from coverage_partitions and passes different values per drone.

### 3.3 map_object_detector — blue_target_mapper

From **one leader's camera** it detects blue targets and outputs **world coordinate** points. 4 leaders -> 4 mapper nodes.

- **Input:**  
  - Camera: `image_raw` + `camera_info`  
  - PX4: `vehicle_local_position`, `vehicle_attitude`
- **Output:** `PointStamped` (header.frame_id = world), x, y = world, z = type or 0.

Algorithm summary:

1. Blue colour filter (HSV), find contours, filter by area and aspect ratio (cube-like blobs).
2. Blob centre (e.g. moments for subpixel).
3. **Pixel -> world:**  
   - camera_info (or fallback K) pinhole unproject -> 3D ray in camera.  
   - Fixed camera-to-body transform (down-facing) + drone quaternion -> body to NED.  
   - Intersect NED ray with z = 0 (ground); intersection is local North, East.  
   - Add spawn offset: world_x = spawn_x + local_east, world_y = spawn_y + local_north.
4. Only publish detections when altitude is high enough (e.g. ~80 m); disable on landing.

So each leader writes to its topic only "world (x,y) of blue targets I see"; merging is in another layer.

### 3.4 multi_scout — MRCPP layer (fusion + viz + recording)

This package ties together "4 leaders + central map". One launch starts:

- 4× satellite_scout_node (scout_mission)
- 4× blue_target_mapper (map_object_detector)
- 1× fusion_node
- 1× zone_boundaries_node
- 1× fused_map_image_node
- 1× detected_targets_recorder_node

**Fusion node:**

- Subscribes: `/scout/detections_1` … `detections_4` (PointStamped, world).
- Drops points outside the enemy zone (min/max x,y in config).
- Compares new point with existing targets: if one is within `merge_radius` (e.g. 4 m) it adds to that target's "observation list" and updates target position to centroid; otherwise adds a new target.
- Publishes targets with at least `min_observations` on `/scout/fused_targets` (PoseArray) and optionally `/scout/detected_targets_for_center`. Optional MarkerArray for RViz.

**Zone boundaries:** Draws the 4 zones from coverage_partitions as lines in RViz (frame_id = world).

**Fused map image:** Takes fused_targets and draws a 2D top-down view (grid + target points); publishes an Image topic (e.g. watch in rqt_image_view).

**Detected targets recorder:** Writes fused_targets to YAML on a timer or only when the node shuts down.

Config: `coverage_partitions.yaml` (4 drones, spawn, scan zone, zone centre), `fusion_params.yaml` (merge_radius, enemy zone, topics, min_observations).

### 3.5 task_allocation — MRTA

Single node that decides for **worker drones** "which target you go to, when you drop".

- **Input:** `/scout/fused_targets` (PoseArray), for each worker `px4_X/fmu/out/vehicle_local_position`.
- **Output:** For each worker `px4_X/fmu/in/offboard_control_mode`, `trajectory_setpoint`, `vehicle_command` (arm, offboard, land). Also Gazebo `/spawn_entity` service (drop = model spawn).

Assignment (simple greedy):

- Runs periodically (e.g. 2 s).
- List unassigned and not-yet-visited targets.
- For idle (IDLE) or returned (RTH_LANDED) workers: compute distance to each unassigned target (worker position converted to world from spawn).
- Assign the nearest (worker, target) pair; target becomes "assigned", worker moves to next state.

Worker state machine (short):

- IDLE -> offboard + arm (ARMING) -> climb to transit alt (CLIMB) -> go to target (GO) -> descend to drop alt (DESCEND) -> DROP (spawn_entity) -> short wait (AFTER_DROP) -> climb again (CLIMB_AFTER_DROP). Then either new target (back to ARMING) or if no targets left return to base (RETURN_TO_BASE) -> when landed RTH_LANDED.
- Only one drop per (worker, target); target marked "visited".

Config: worker list (vehicle_id, spawn_x_world, spawn_y_world), transit/drop altitude, acceptance radius, spawn service, optional payload SDF. Spawn coords must match the sim script's worker positions.

---

## 4. Data flow (topics summary)

- **Leaders:**  
  - PX4 -> ROS 2: `px4_{1..4}/fmu/out/vehicle_local_position`, `vehicle_status`, `vehicle_attitude`.  
  - Camera: `leader_{1..4}/camera/image_raw`, `camera_info`.  
  - Detections: `scout/detections_{1..4}` (PointStamped, world).

- **Fusion:**  
  - Input: `detections_1..4`.  
  - Output: `scout/fused_targets` (PoseArray), optional `scout/fused_targets_markers`, `scout/detected_targets_for_center`.

- **Workers:**  
  - PX4 -> ROS 2: `px4_{5..10}/fmu/out/vehicle_local_position`.  
  - MRTA -> PX4: `px4_{5..10}/fmu/in/offboard_control_mode`, `trajectory_setpoint`, `vehicle_command`.

- **Viz / recording:**  
  - Zone: `scout/zone_boundaries` (MarkerArray).  
  - Map image: Image topic from fused_map_image node.  
  - MRTA status: `mrta/status`, `mrta/markers` (from config).

---

## 5. Run order (when explaining)

1. **Agent:** `MicroXRCEAgent udp4 -p 8888` (or script). Keep it running.
2. **Sim:** `run_sim_four_drones.sh` — Gazebo + 10 PX4 (4 leaders, 6 workers). World and spawns are set here.
3. **multi_scout launch:** 4 scout + 4 mapper + fusion + zone + fused map + recorder. That fills `/scout/fused_targets`.
4. **MRTA launch:** Workers get assigned from the target list, fly, drop when they reach the target.
5. Optional: RViz (frame: world/map), rqt_image_view (map image).

Summary: "First sim and agent, then recon + fusion, last task allocation; coords and spawns must match config." That sentence is a good frame when explaining the project.

---

## 6. Why designed like this? (short)

- **Leader / worker split:** Recon (camera, high alt) vs carry/drop (low alt, payload) are different; we split roles instead of using the same drones for both.
- **4 zones:** Map split 2×2, each leader takes one quarter; less overlap, clear coverage.
- **Simple fusion:** Distance + centroid; no heavy SLAM, target list is quick and easy to follow.
- **Greedy MRTA:** Not optimal routing but "assign nearest target"; easy to implement, behaviour is predictable.
- **World coords as common language:** Detection, fusion and MRTA all use the same reference; only when talking to PX4 each drone converts to NED from its spawn.

You can use this doc like "explaining the project in detail to someone"; you can also focus on specific packages (e.g. only fusion or only MRTA) and go deeper if you want.
