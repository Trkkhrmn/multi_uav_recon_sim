# multi_scout

This package ties together the four leader drones and turns their detections into one shared map. When you run `ros2 launch multi_scout multi_scout.launch.py` it starts: four satellite_scout nodes (from scout_mission), four blue_target_mapper nodes (from map_object_detector), one fusion node, one node that draws the zone boundaries in RViz, one that draws a 2D map image with the fused targets, and one that can write the fused targets to a YAML file. So it’s basically the “MRCPP” layer: multi-robot coverage and the central fusion.

## Coverage and partitions

The area we care about (the “enemy zone”) is split into four parts in a 2x2 grid. Each leader drone has its own cell. The split is defined in `config/coverage_partitions.yaml`: for each drone you have spawn position, the min/max world X and Y of its cell, and the centre of that cell. The launch file reads this and passes the right parameters to each satellite_scout_node and each blue_target_mapper. So each drone flies to the centre of its cell and the mapper uses the right spawn offset for that drone. The zone_boundaries node reads the same config and publishes lines in RViz so you can see the four rectangles.

## Fusion node

The fusion node subscribes to the four detection topics (/scout/detections_1 through 4). Each message is a PointStamped in world coordinates. When a new point comes in we check if it’s inside the enemy zone (we have min/max x and y in the config). If it is, we either add it as a new target or merge it with an existing one. Merging is by distance: if the new point is within merge_radius (e.g. 4 m) of an existing target we don’t create a new one, we add this observation to that target and update the target’s position to the centroid of all observations. So the more drones see the same object, the more stable the position gets. We only publish targets that have at least min_observations (often 1). The output is a PoseArray on /scout/fused_targets and the same list is republished on /scout/detected_targets_for_center for whatever needs the final target list (e.g. the MRTA node). Optionally we also publish a MarkerArray for RViz so you can see the fused targets as spheres.

So the idea is: four streams of (x, y) points in world frame, filter by zone, cluster by distance, take centroid per cluster, publish one list. There is no weighting or time decay: every observation has equal weight and the centroid is a plain average of all points in that cluster. When a new detection arrives within merge_radius of an existing target, we append it to that target’s list and recompute the centroid. Simple and robust for this setup.

## Other nodes in this package

- **zone_boundaries_node**: Reads coverage_partitions and publishes the four zone rectangles as LineStrip markers. Useful in RViz to see the grid.
- **fused_map_image_node**: Subscribes to /scout/fused_targets and draws a top-down 2D image: grid, zone rectangles, and the fused targets as dots. Publishes an Image so you can view it in rqt_image_view.
- **detected_targets_recorder_node**: Subscribes to /scout/fused_targets and writes the list to a YAML file. You can set the path and how often (e.g. every 30 seconds or only when the node shuts down). The launch file has arguments for the output path and the interval.

## Config files

- `coverage_partitions.yaml`: Defines the four drones (vehicle_id, spawn, zone bounds, zone centre). Must match the spawn positions in the sim script (run_sim_four_drones.sh).
- `fusion_params.yaml`: merge_radius, enemy zone limits, topic names, min_observations, etc.

## Topics

Inputs to fusion: `/scout/detections_1` … `/scout/detections_4` (PointStamped). Output: `/scout/fused_targets` (PoseArray, world frame). The recorder and the map image node both listen to fused_targets. If you want to see the numbers you can do `ros2 topic echo /scout/fused_targets`.
