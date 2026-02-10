# task_allocation

This package implements **priority-aware MRTA with live visualization**. After the scouts publish fused targets, the MRTA node assigns worker drones to visit them and drop payloads. The **mrta_panel** (run with `ros2 run task_allocation mrta_panel`) is the main UI for monitoring assignments and worker states in real time; it is the recommended way to watch the mission. The rest of this README describes the core node logic. After the scout drones have found the targets and the fusion node has published them on /scout/fused_targets, this node takes that list and assigns the “worker” drones (the six normal iris, not the four leaders) to fly to each target and drop a payload. So the scouts do the recon and the workers do the delivery.

## How assignment works

We use a simple greedy rule: each worker is assigned to the target that is closest to it and that hasn’t been visited yet. The node subscribes to /scout/fused_targets (PoseArray in world coordinates) and to each worker’s vehicle_local_position. We have to convert between world and local NED because PX4 talks in local (origin at that drone’s spawn). So for each worker we know its spawn (x, y) in world frame from the config, and we can convert a target (world_x, world_y) to that worker’s local NED: local_north = world_y - spawn_y, local_east = world_x - spawn_x. Then we compute distance from the worker’s current position to each unvisited target (in a consistent frame). Every few seconds we re-run the assignment: for each worker that’s idle or finished its last task we pick the closest unvisited target and set that as its current goal. So it’s “nearest first” and we don’t do any optimisation like minimising total distance; it’s just to get something that works.

## Worker state machine

Each worker has a state. Roughly: IDLE -> we send offboard and arm (ARMING), then it climbs to a transit altitude (CLIMB). When it’s high enough we set the setpoint to the target’s (x, y) at transit height (GO). When it’s close enough in xy we switch to a lower altitude (DESCEND) for the drop. When it’s at drop height we call the Gazebo spawn service to spawn the payload model at that position (DROP), then we wait a short time (AFTER_DROP), then climb back up (CLIMB_AFTER_DROP). After that we either assign the next target or send it back to base (RETURN_TO_BASE) and when it’s there we consider it RTH_LANDED. So each worker keeps looping: get next target, go, descend, drop, climb, repeat until no targets left, then return home.

We only drop once per (worker, target) pair so we don’t spawn ten boxes on the same spot. Visited targets are tracked and not reassigned.

## Config and parameters

The worker list (vehicle IDs and their spawn positions in world frame) comes from parameters. You can set them in the launch file or in a yaml that’s loaded (e.g. config/worker_drones.yaml). The spawn positions must match where the sim actually spawns the worker drones (run_sim_four_drones.sh spawns 4 leaders and 6 workers at specific coordinates). Other parameters: transit altitude, drop altitude, acceptance radius for “close enough”, time to wait after drop before climbing again, and the Gazebo spawn_entity service name. We can also pass a path to an SDF file for the payload model; if not set we use a simple box.

## RViz and status

The node can publish a MarkerArray (e.g. on /mrta/markers) so you can see in RViz which targets are still to do (e.g. green) and which are done (e.g. grey), and lines from each worker to its assigned target. There’s also a status topic that prints a short text summary of each worker’s state so you can debug without opening RViz.

## How to run

After the sim and multi_scout are running (so /scout/fused_targets has data), run `ros2 launch task_allocation mrta.launch.py`. The node will wait a bit (start_delay) then start assigning workers to the targets that fusion has published. Make sure the Micro XRCE-DDS agent is running so the workers get the commands.
