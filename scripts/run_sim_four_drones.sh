#!/usr/bin/env bash
# Manual start of 10 drones: Gazebo Classic + PX4 SITL (4 leaders iris_leader + 6 iris).
# Don't use 4001 / "gz" — for Gazebo Classic you need 10015 (gazebo-classic_iris).
# If drones aren't spawned into Gazebo, PX4 can't simulate; this script does spawn + PX4 in the right order.

set -e

# --- 1. Paths ---
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSETS_DIR="${WORKSPACE_ROOT}/assets"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
SRC_PATH="${PX4_DIR}"
SITL_GAZEBO="${SRC_PATH}/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
JINJA_SCRIPT="${SITL_GAZEBO}/scripts/jinja_gen.py"
MODELS_PATH="${SITL_GAZEBO}"
WORLDS_DIR="${SITL_GAZEBO}/worlds"
# World: mili.world from workspace assets/mili_tech, else PX4 empty.world
MILI_WORLD="${ASSETS_DIR}/mili_tech/mili.world"
EMPTY_WORLD="${WORLDS_DIR}/empty.world"
if [[ -f "$MILI_WORLD" ]]; then
	WORLD_FILE="$MILI_WORLD"
else
	WORLD_FILE="$EMPTY_WORLD"
fi

# --- 1b. Require PX4 Gazebo Classic submodule (jinja_gen.py, worlds, models) ---
if [[ ! -f "$JINJA_SCRIPT" ]]; then
	echo "ERROR: PX4 Gazebo Classic submodule not initialized."
	echo "  Missing: $JINJA_SCRIPT"
	echo "  Run: cd ${PX4_DIR} && git submodule update --init Tools/simulation/gazebo-classic/sitl_gazebo-classic"
	exit 1
fi
if [[ ! -f "$WORLD_FILE" ]]; then
	echo "ERROR: No world file found. Expected: ${ASSETS_DIR}/mili_tech/mili.world or $EMPTY_WORLD"
	echo "  Run: cd ${PX4_DIR} && git submodule update --init Tools/simulation/gazebo-classic/sitl_gazebo-classic"
	echo "  For mili.world: ensure assets/mili_tech/mili.world exists (e.g. run setup_assets.sh to link mili_tech)."
	exit 1
fi

# --- 2. Gazebo Classic + PX4 environment (not gz harmonic) ---
source "${PX4_DIR}/Tools/simulation/gazebo-classic/setup_gazebo.bash" "${PX4_DIR}" "${BUILD_DIR}"
source /opt/ros/humble/setup.bash 2>/dev/null || true

# Needed for map models (mili_map, blue_cube, grass_plane etc); blue_cube is in PX4 models
export GAZEBO_MODEL_PATH="${MODELS_PATH}/models:${ASSETS_DIR}/common_models:${ASSETS_DIR}/mili_tech:${HOME}/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="${GAZEBO_RESOURCE_PATH:-}:/usr/share/gazebo-11:/usr/share/gazebo"
export ROS_VERSION=2

# --- 3. Cleanup ---
echo "Stopping old processes..."
killall -9 px4 gzserver gzclient 2>/dev/null || true
sleep 2

# --- 4. Start Gazebo (plugins for ROS 2 camera topics) ---
echo "Gazebo Classic starting ($(basename "$WORLD_FILE"))..."
ROS_PLUGINS=""
[[ -n "$ROS_VERSION" ]] && [[ "$ROS_VERSION" == "2" ]] && ROS_PLUGINS="-s libgazebo_ros_init.so -s libgazebo_ros_factory.so"
gzserver --verbose "$WORLD_FILE" $ROS_PLUGINS &
GZ_PID=$!
sleep 8

if [[ "${1:-}" != "headless" ]]; then
  gzclient &
  sleep 3
fi

# --- 5. For each drone: start PX4, generate SDF, spawn into Gazebo ---
# Order in Gazebo Classic: PX4 runs first (listens on TCP 4560+N), then we spawn the model so the plugin can connect.

spawn_one() {
  local MODEL=$1
  local N=$2
  local X=$3
  local Y=$4
  local Z=$5
  local n=$((N - 1))   # rootfs/0, rootfs/1, ...

  # Gazebo Classic airframe for PX4 (4001 = gz harmonic, wrong; 10015 = gazebo-classic iris)
  export PX4_SIM_MODEL="gazebo-classic_iris"
  working_dir="${BUILD_DIR}/rootfs/${n}"
  mkdir -p "$working_dir"

  echo ">>> Starting drone ${N} (${MODEL}): PX4 -i ${N}, spawn @ ${X} ${Y} ${Z}"

  # Start PX4 in background (it waits until Gazebo plugin connects)
  (cd "$working_dir" && "${BUILD_DIR}/bin/px4" -i "$N" -d "${BUILD_DIR}/etc" > out.log 2> err.log) &
  sleep 3

  # Generate SDF with jinja — mavlink_id=N keeps px4_1..px4_4 consistent for fmu and camera
  python3 "$JINJA_SCRIPT" \
    "${MODELS_PATH}/models/${MODEL}/${MODEL}.sdf.jinja" \
    "$MODELS_PATH" \
    --mavlink_tcp_port $((4560 + N)) \
    --mavlink_udp_port $((14560 + N)) \
    --mavlink_id "$N" \
    --gst_udp_port $((5600 + N)) \
    --video_uri $((5600 + N)) \
    --mavlink_cam_udp_port $((14530 + N)) \
    --output-file "/tmp/${MODEL}_${N}.sdf"

  # Spawn model into Gazebo (matches PX4 over TCP)
  gz model --spawn-file="/tmp/${MODEL}_${N}.sdf" --model-name="${MODEL}_${N}" -x "$X" -y "$Y" -z "$Z"
  sleep 2
}

# 4 leaders (iris_leader, down cam), 6 workers (iris)
spawn_one iris_leader 1  185.41 56.52 2
spawn_one iris_leader 2  189.41 56.52 2
spawn_one iris_leader 3  189.41 60.52 2
spawn_one iris_leader 4  185.41 60.52 2
spawn_one iris        5  193.41 56.52 2
spawn_one iris        6  193.41 60.52 2
spawn_one iris        7  193.41 64.52 2
spawn_one iris        8  197.41 56.52 2
spawn_one iris        9  197.41 60.52 2
spawn_one iris       10  197.41 64.52 2

echo "All drones up (4 leaders + 6 workers). Leader cams: /leader_1/.. /leader_4/camera/..., workers: px4_5..px4_10/fmu/..."
wait $GZ_PID 2>/dev/null || wait
