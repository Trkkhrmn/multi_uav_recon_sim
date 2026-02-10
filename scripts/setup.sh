#!/usr/bin/env bash
# One-time setup: px4_msgs, Gazebo assets, colcon build.
# Run from workspace root: ./scripts/setup.sh

set -e
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"

echo "[1/3] px4_msgs (required for PX4-ROS2 bridge)"
if [ ! -d "src/px4_msgs" ]; then
	git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
	echo "  Cloned px4_msgs into src/"
else
	echo "  src/px4_msgs already present, skipping."
fi

echo "[2/3] Gazebo assets (symlinks for world models)"
if [ -x "scripts/setup_assets.sh" ]; then
	./scripts/setup_assets.sh
else
	echo "  Warning: scripts/setup_assets.sh not found or not executable."
fi

echo "[3/3] Build workspace"
colcon build --packages-up-to multi_scout
colcon build --packages-select task_allocation
echo ""
echo "Done. Source the workspace: source install/setup.bash"
echo "Then run simulation and nodes (see README or ./scripts/run_demo.sh --help)."
