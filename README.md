<h1 align="center">Multi-UAV Reconnaissance & Task Allocation System</h1>

<p align="center">
  <strong>4 Scout Drones + 6 Worker Drones | Autonomous Target Detection, Sensor Fusion & Task Allocation</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros&logoColor=white" alt="ROS 2 Humble"/>
  <img src="https://img.shields.io/badge/PX4-Autopilot-orange?logo=drone&logoColor=white" alt="PX4"/>
  <img src="https://img.shields.io/badge/Gazebo-Classic_11-green?logo=gazebo&logoColor=white" alt="Gazebo"/>
  <img src="https://img.shields.io/badge/Python-3.10-yellow?logo=python&logoColor=white" alt="Python"/>
  <img src="https://img.shields.io/badge/OpenCV-4.x-red?logo=opencv&logoColor=white" alt="OpenCV"/>
  <img src="https://img.shields.io/badge/License-MIT-brightgreen" alt="License"/>
</p>

<p align="center">
  <a href="#-demo">Demo</a> &nbsp;&bull;&nbsp;
  <a href="#-features">Features</a> &nbsp;&bull;&nbsp;
  <a href="#%EF%B8%8F-system-architecture">Architecture</a> &nbsp;&bull;&nbsp;
  <a href="#-algorithms">Algorithms</a> &nbsp;&bull;&nbsp;
  <a href="#-installation">Installation</a> &nbsp;&bull;&nbsp;
  <a href="#-usage">Usage</a> &nbsp;&bull;&nbsp;
  <a href="#-results--metrics">Results</a>
</p>

---

## About

An end-to-end multi-UAV reconnaissance and task allocation pipeline built on **ROS 2**, **PX4 SITL** and **Gazebo Classic**. Ten autonomous drones operate in a simulated military environment — no real hardware required.

**4 scout (leader) drones** fly to their assigned zones at 80 m altitude, detect colored ground targets using downward-facing cameras, and compute world coordinates via **pinhole camera model + ground-plane ray intersection**. A central **fusion node** merges detections from all four scouts using distance-based centroid averaging to produce a single, noise-filtered target map. **6 worker drones** are then assigned to those targets by an **MRTA (Multi-Robot Task Allocation)** node using a greedy nearest-first strategy. Each worker flies to its target, descends, drops a payload, and either proceeds to the next target or returns to base for resupply.

<p align="center">
  <img src="docs/media/full_pipeline_demo.png" alt="Full Pipeline Demo" width="95%"/>
</p>

---

## Demo

<p align="center">
  <img src="docs/media/gazebo_world.png" alt="Gazebo Simulation World" width="95%"/>
</p>

<p align="center"><em>Gazebo simulation environment — military fortress with ground targets (blue, magenta, cyan cubes), tanks, buildings and mountainous terrain.</em></p>

<br/>

<table>
  <tr>
    <td align="center" width="50%">
      <img src="docs/media/rviz_fused_targets.png" alt="Fused Target Map" width="100%"/>
      <br/><em>Fused target map — 4 color-coded zones with detected targets (numbered markers) published on <code>/scout/fused_map_image</code></em>
    </td>
    <td align="center" width="50%">
      <img src="docs/media/mrta_command_panel.png" alt="MRTA Command Panel" width="100%"/>
      <br/><em>Full system running — Gazebo world, RViz markers, camera feeds and MRTA command panel side by side</em>
    </td>
  </tr>
</table>

---

## Features

| Feature | Description |
|---------|-------------|
| **Multi-Robot Coverage (MRCPP)** | Map is split into 4 zones; each scout drone covers its zone (satellite hover, lawnmower, or circular scan) |
| **Camera-Based Target Detection** | HSV color segmentation + contour analysis + `cv2.moments()` for subpixel centroid |
| **Pixel-to-World Geolocation** | Pinhole camera model (`image_geometry`) → 3D ray → ground-plane intersection → NED-to-world transform |
| **Central Sensor Fusion** | Detections from 4 drones merged by distance (merge radius + centroid); minimum observation filter removes noise |
| **MRTA Task Allocation** | Greedy nearest-first assignment; 6 workers with full state machine (ARM → CLIMB → GO → DESCEND → DROP → RTH) |
| **Payload Drop** | Gazebo `/spawn_entity` service spawns a model at the target location |
| **Live Visualization** | RViz MarkerArray (targets, assignment lines, drone states), 2D fused map image, MRTA status panel |
| **Capacity & Resupply** | Workers have different package capacities (1–3) and speed profiles; return to base when empty |
| **Scenario Analysis** | `scenario_analysis.py` generates ground-truth vs detection plots, RMSE, error histograms |

---

## System Architecture

<p align="center">
  <img src="docs/media/architecture_figure.png" alt="System Architecture" width="85%"/>
</p>

<p align="center"><em>Adaptive MRTA Framework — Multi-Layer Drone Swarm System Architecture</em></p>

The system is organized into **5 layers**:

| Layer | Components | Role |
|-------|-----------|------|
| **1. Simulation** | Gazebo Classic + PX4 SITL (10 instances) | Physics, sensors, flight dynamics |
| **2. Communication** | Micro XRCE-DDS Agent (UDP 8888) | Bridges PX4 DDS ↔ ROS 2 topics |
| **3. Detection & Scouting** | `scout_mission` (4x) + `map_object_detector` (4x) | Offboard flight control + HSV detection + pixel→world |
| **4. Data Fusion** | `multi_scout` (fusion, zones, map image, recorder) | Merge detections → single target list |
| **5. Task Allocation** | `task_allocation` (MRTA node) | Assign workers → fly → drop → return |

### ROS 2 Topic Flow

```
Scout Drones (PX4 → ROS 2)
├── px4_{1..4}/fmu/out/vehicle_local_position
├── px4_{1..4}/fmu/out/vehicle_attitude
└── leader_{1..4}/camera/image_raw + camera_info

Detection → Fusion
├── /scout/detections_{1..4}          (PointStamped)
├── /scout/fused_targets              (PoseArray)
├── /scout/fused_targets_markers      (MarkerArray)
├── /scout/zone_boundaries            (MarkerArray)
└── /scout/fused_map_image            (Image)

MRTA → Worker Drones
├── px4_{5..10}/fmu/in/trajectory_setpoint
├── px4_{5..10}/fmu/in/vehicle_command
├── /mrta/markers                     (MarkerArray)
└── /mrta/status                      (String)
```

---

## Algorithms

### 1. Pixel → 3D Ray (Unprojection)
Camera intrinsics (K matrix) from `CameraInfo` are used with `image_geometry.PinholeCameraModel`. For each detected pixel, `projectPixelTo3dRay(u, v)` returns a unit direction vector in the camera optical frame.

### 2. Camera Optical → Body NED Transform
A fixed rotation matrix maps the downward camera's optical frame to PX4 body NED:
- Optical Z (forward) → Body Z (down)
- Optical X (right) → Body Y (east)
- -Optical Y → Body X (north)

### 3. Ground-Plane Ray Intersection
The 3D ray is rotated into NED using the drone's quaternion, then intersected with the **z = 0** ground plane:

```
t = -z_drone / ray_z
North = x_drone + t * ray_x
East  = y_drone + t * ray_y
```

Spawn offset is added to convert local NED → world coordinates.

### 4. Subpixel Detection Center
`cv2.moments(contour)` computes the centroid — more accurate than bounding box center:

```
u = M10 / M00,  v = M01 / M00
```

### 5. Color-Based Target Detection
HSV segmentation (H: 100–140 for blue) with morphological opening. Contours filtered by area and aspect ratio.

### 6. Central Map Fusion
Detections within `merge_radius` (e.g. 4 m) are treated as the same target; position is updated as the centroid of all observations. Only targets with `min_observations` are published.

### 7. Worker Drone State Machine

<p align="center">
  <img src="docs/media/worker_diagram.png" alt="Worker State Machine" width="75%"/>
</p>

<p align="center"><em>Worker drone state machine — from IDLE to DROP and back. If a new target is detected, the cycle repeats; otherwise the worker returns to base.</em></p>

---

## Installation

### Prerequisites

| Component | Version / Notes |
|-----------|----------------|
| **OS** | Ubuntu 22.04 LTS |
| **ROS 2** | Humble Hawksbill |
| **PX4-Autopilot** | SITL + Gazebo Classic |
| **Gazebo** | Classic 11 |
| **Python** | 3.10+ |
| **OpenCV** | 4.x (`pip install opencv-python`) |
| **NumPy** | `pip install numpy` |
| **Matplotlib** | `pip install matplotlib` (for scenario analysis) |

### Step-by-Step Setup

**1. Clone the repository:**
```bash
git clone https://github.com/<YOUR_USERNAME>/multi_uav_recon_ws.git
cd multi_uav_recon_ws
```

**2. Install PX4-Autopilot and initialize the Gazebo Classic submodule:**
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot

git submodule update --init Tools/simulation/gazebo-classic/sitl_gazebo-classic
make px4_sitl_default sitl_gazebo-classic
```

**3. Add px4_msgs to the workspace:**
```bash
cd ~/multi_uav_recon_ws/src
git clone https://github.com/PX4/px4_msgs.git
cd ..
```

**4. Link Gazebo model assets:**
```bash
./scripts/setup_assets.sh
```
> Creates `assets/` symlinks. Requires `common_models` and `mili_tech` in `~/gazebo_maps`. Adjust `GAZEBO_MAPS_DIR` in the script if your models are elsewhere.

**5. Build the workspace:**
```bash
source /opt/ros/humble/setup.bash

colcon build --packages-up-to multi_scout
colcon build --packages-select task_allocation

source install/setup.bash
```

---

## Usage

The system requires **4 terminals**, launched in order. Run `./scripts/run_demo.sh` to see all commands at a glance.

### Terminal 1 — Micro XRCE-DDS Agent
```bash
./scripts/run_microxrce_agent.sh
# or: MicroXRCEAgent udp4 -p 8888
```

### Terminal 2 — Simulation (Gazebo + 10 PX4 instances)
```bash
./scripts/run_sim_four_drones.sh
# headless mode: ./scripts/run_sim_four_drones.sh headless
```

### Terminal 3 — Scout + Fusion (MRCPP)
```bash
source install/setup.bash
ros2 launch multi_scout multi_scout.launch.py
```

### Terminal 4 — MRTA (after fusion publishes targets)
```bash
source install/setup.bash
ros2 launch task_allocation mrta.launch.py
```

### Optional Tools

```bash
# Live MRTA panel
ros2 run task_allocation mrta_panel

# Camera / map image viewer
ros2 run rqt_image_view rqt_image_view

# MRTA markers in RViz
rviz2 -d src/task_allocation/config/mrta.rviz

# MRTA status in terminal
ros2 topic echo /mrta/status
```

### RViz Marker Legend

| Color | Meaning |
|-------|---------|
| **Green box** | Unvisited target |
| **Grey box** | Visited target (payload dropped) |
| **Yellow line** | Drone → assigned target link |
| **White label** | Drone state (e.g. `W5: GO`, `W7: DESCEND`) |

---

## Results & Metrics

The system was tested across **4 scenarios** with a total of **55 ground-truth target points**. Detection accuracy was evaluated by comparing ground-truth coordinates against fused detections.

### Overall Performance

| Metric | Value |
|--------|-------|
| **Total test points** | 55 (4 scenarios) |
| **Mean Euclidean error** | 0.808 m |
| **RMSE** | 0.930 m |
| **Max error** | 2.812 m |

| Scenario | Points | Mean Error | RMSE |
|----------|:------:|:----------:|:----:|
| Scenario 1 | 13 | 0.982 m | 1.189 m |
| Scenario 2 | 14 | 0.765 m | 0.840 m |
| Scenario 3 | 14 | 0.775 m | 0.861 m |
| Scenario 4 | 14 | 0.722 m | 0.797 m |

### Analysis Plots

Generate all plots with:
```bash
python3 scripts/scenario_analysis.py
# Output: output/figures/
```

<table>
  <tr>
    <td align="center" width="50%">
      <img src="docs/media/figures/01_gt_vs_detected_xy.png" alt="GT vs Detected" width="100%"/>
      <br/><em>Ground Truth vs Detected (X & Y)</em>
    </td>
    <td align="center" width="50%">
      <img src="docs/media/figures/02_error_vectors_2d.png" alt="Error Vectors" width="100%"/>
      <br/><em>Position error vectors (GT → Detected)</em>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="docs/media/figures/03_error_histogram.png" alt="Error Histogram" width="100%"/>
      <br/><em>Euclidean error distribution</em>
    </td>
    <td align="center" width="50%">
      <img src="docs/media/figures/04_error_by_scenario_boxplot.png" alt="Error Boxplot" width="100%"/>
      <br/><em>Localization error by scenario</em>
    </td>
  </tr>
  <tr>
    <td align="center" width="50%">
      <img src="docs/media/figures/05_rmse_mean_per_scenario.png" alt="RMSE per Scenario" width="100%"/>
      <br/><em>Mean error & RMSE per scenario</em>
    </td>
    <td align="center" width="50%">
      <img src="docs/media/figures/06_xy_error_by_scenario.png" alt="XY Error" width="100%"/>
      <br/><em>X and Y component errors by scenario</em>
    </td>
  </tr>
</table>

---

## Configuration

### Scout Drones (`coverage_partitions.yaml`)

| Drone | Zone | Center (X, Y) | Spawn (X, Y) |
|-------|------|:--------------:|:-------------:|
| Leader 1 | Top Right | (38.59, 32.34) | (185.41, 56.52) |
| Leader 2 | Top Left | (-35.22, 32.34) | (189.41, 56.52) |
| Leader 3 | Bottom Left | (-35.22, -32.29) | (189.41, 60.52) |
| Leader 4 | Bottom Right | (38.59, -32.29) | (185.41, 60.52) |

### Worker Drones (`worker_drones.yaml`)

| Drone | ID | Package Capacity | Speed Scale | Type |
|-------|----|:----------------:|:-----------:|------|
| Worker 5–6 | 5, 6 | 3 | 0.5x (slow) | Heavy load |
| Worker 7–8 | 7, 8 | 2 | 0.7x (mid) | Medium load |
| Worker 9–10 | 9, 10 | 1 | 1.0x (fast) | Light load |

---

## Tech Stack

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white"/>
  <img src="https://img.shields.io/badge/PX4-Autopilot-F05032?style=for-the-badge&logo=drone&logoColor=white"/>
  <img src="https://img.shields.io/badge/Gazebo-Classic_11-E88C1F?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Python-3.10-3776AB?style=for-the-badge&logo=python&logoColor=white"/>
  <img src="https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white"/>
  <img src="https://img.shields.io/badge/NumPy-013243?style=for-the-badge&logo=numpy&logoColor=white"/>
  <img src="https://img.shields.io/badge/Matplotlib-11557c?style=for-the-badge"/>
</p>

---

## Known Limitations

- Single world (military fortress); fixed spawn positions and zone layout
- MRTA uses greedy nearest-first — no optimal routing (TSP/VRP)
- No priority-based assignment or failure-aware reassignment
- Color-based detection — may produce false positives in complex environments

## Future Work

- [ ] Priority-aware MRTA (different priorities per target type)
- [ ] Failure-aware reassignment (worker failure → automatic reassignment)
- [ ] Dynamic target injection during simulation
- [ ] Configurable world maps and zone layouts
- [ ] Feature-based target detection (replacing color-only)
- [ ] Optimal routing algorithms (TSP / VRP-based)

---

## Documentation

| Directory | Contents |
|-----------|----------|
| `docs/overview/` | Project description, CV / LinkedIn summary |
| `docs/architecture/` | MRTA visualization, multi-drone control |
| `docs/missions/` | Scout and satellite scout parameters |
| `docs/sensors/` | Camera setup and topics |
| `docs/legacy/` | Old baselines and backups (reference) |

Per-package READMEs are available at `src/<package_name>/README.md`.

---

## References

<details>
<summary><strong>Academic references and sources</strong> (click to expand)</summary>

### Camera Model & Projection
1. **R. Hartley & A. Zisserman**, *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge University Press, 2003.
2. **ROS image_geometry**, [PinholeCameraModel](https://docs.ros.org/en/api/image_geometry/html/python/)

### Transforms (Quaternion, Coordinate Frames)
3. **J. Diebel**, "Representing attitude: Euler angles, unit quaternions, and rotation vectors," Stanford University, 2006.
4. **PX4 Development Guide**, [Coordinate Frames](https://docs.px4.io/main/en/coordinate_frames/README.html)

### Multi-Robot Coverage
5. **H. Choset**, "Coverage of known spaces: The boustrophedon cellular decomposition," *Autonomous Robots*, 9(3), 2000.
6. **E. Galceran & M. Carreras**, "A survey on coverage path planning for robotics," *Robotics and Autonomous Systems*, 61(12), 2013.

### Data Fusion
7. **S. Thrun, W. Burgard, D. Fox**, *Probabilistic Robotics*, MIT Press, 2005.

### Image Processing
8. **R. Szeliski**, *Computer Vision: Algorithms and Applications*, 2nd ed., Springer, 2022.
9. **OpenCV Documentation**, [Structural Analysis and Shape Descriptors](https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html)

</details>

---

## License

This project is licensed under the [MIT License](LICENSE).

---

<p align="center">
  <sub>Built by <strong><a href="https://github.com/<YOUR_USERNAME>">Tarik Kahraman</a></strong></sub><br/>
  <sub>Questions or suggestions? <a href="https://github.com/<YOUR_USERNAME>/multi_uav_recon_ws/issues">Open an issue</a></sub>
</p>
