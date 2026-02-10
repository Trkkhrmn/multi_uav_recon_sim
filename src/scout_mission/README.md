# scout_mission

This package has one main node: **satellite_scout_node**. It controls a single leader drone (iris with downward camera). The drone takes off, flies to the centre of a zone that you define, and then either just hovers there or does a scan pattern around it (circle, square, or lawnmower). Everything is driven by PX4 offboard mode, so you need the Micro XRCE-DDS agent running or the drone will not receive any commands.

## What it actually does

The node talks to PX4 over ROS 2 topics (offboard control, trajectory setpoint, vehicle command). The tricky part is the coordinate frames. In Gazebo the world is usually X = East, Y = North. PX4 uses NED (North-East-Down) with the origin at the drone’s spawn point. So whenever we have a target in “world” coordinates (e.g. the centre of the scan zone), we have to convert it to local NED: `local_north = world_y - spawn_y`, `local_east = world_x - spawn_x`. The altitude is negative in NED (e.g. -80 means 80 m above the ground).

The state machine is roughly: INIT (send setpoints for a bit, then request offboard), wait until PX4 is in offboard, arm, then climb to a transit altitude (to avoid hitting hills). After that it goes to the zone centre. If scan is enabled it can do an orbit (circle or lawnmower pattern) around that centre; otherwise it just holds position. All of this is parameterised (spawn position, zone bounds, altitude, speeds, acceptance radius for waypoints). The launch file that uses this is in the multi_scout package, which starts four of these nodes (one per leader drone) with different zone centres.

## Main parameters

Things you’ll care about: `vehicle_id` (which PX4 instance, 1–4 for the four leaders), `spawn_x_world` and `spawn_y_world` (where the drone spawns in Gazebo, must match the sim script), `satellite_altitude` (height in metres, as a negative NED value), and the scan zone limits (`scan_zone_world_min/max_x` and same for y). There are more for the orbit shape, lawnmower size, and speeds; they’re all in the code and can be overridden via ROS params.

## How to run it

Normally you don’t run this package by itself. You run `ros2 launch multi_scout multi_scout.launch.py`, which starts four satellite_scout nodes plus the rest of the pipeline. If you want to test a single drone you can run `ros2 run scout_mission satellite_scout_node --ros-args -p vehicle_id:=1` after the sim and the agent are up. Make sure the agent is running first or the drone will not move.

For more detail on parameters and the mission flow (transit, hover, orbit options), see the docs in the workspace (e.g. SATELLITE_SCOUT.md).
