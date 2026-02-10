# map_object_detector

This package is used for detecting blue targets in the camera image and turning them into world coordinates. The part that’s actually used in the pipeline is **blue_target_mapper**. There’s also **camera_view_node** if you just want to view a camera topic. The mapper runs once per leader drone (four instances in total when you launch multi_scout).

## What blue_target_mapper does

It subscribes to one camera image topic and the corresponding camera_info, plus the drone’s position and attitude from PX4 (vehicle_local_position and vehicle_attitude). In the image it finds blue blobs (HSV colour filter, then contours). For each blob it keeps only the ones that pass an area and aspect-ratio check so we don’t get random noise or things that don’t look like our targets. The centre of each contour is computed with cv2.moments so we get a subpixel centre instead of just the bounding box centre, which helps a bit with accuracy.

The main part is turning that pixel (u, v) into a point on the ground in world coordinates. First we get a 3D ray from the camera using the pinhole model. We use the camera_info (or a fallback if it’s not there) to build the intrinsics matrix and then unproject the pixel to a ray in the camera’s optical frame. The camera is pointing down, so we have a fixed rotation from optical frame to the body frame (NED), and then we use the drone’s quaternion to rotate from body to the local NED frame. So we get a ray in NED. The ground is at z = 0 (in local NED, so same height as the drone’s home). We do a ray–plane intersection: find the t such that the ray hits z = 0, then the hit point in local NED is the North and East of the target. Finally we add the spawn offset (spawn_x_world, spawn_y_world) to convert from local NED to Gazebo world coordinates. That’s what we publish as PointStamped on the detection topic.

So the pipeline is: pixel -> pinhole unproject -> optical ray -> body NED -> local NED ray -> intersect with ground plane -> local (North, East) -> add spawn offset -> world (x, y). We only publish when the drone is high enough (target altitude roughly), so we don’t get crazy values when it’s still on the ground.

## Dependencies

You need OpenCV (for the image and contours), cv_bridge (ROS image to OpenCV), and ideally the image_geometry package for the PinholeCameraModel. If image_geometry is not there we fall back to using the K matrix from camera_info ourselves. We also need px4_msgs for the position and attitude.

## How it’s used

The multi_scout launch file starts four blue_target_mapper nodes. Each one is bound to one leader camera (e.g. leader_1, leader_2, …) and one detection output topic (e.g. /scout/detections_1, …). The fusion node in multi_scout reads those four topics and merges the detections into a single list. So this package doesn’t do the merging, it just does the per-drone detection and coordinate conversion.
