#!/usr/bin/env bash
# Print the exact commands to run the full demo (agent, sim, multi_scout, MRTA).
# Usage: ./scripts/run_demo.sh   (prints instructions)
#        ./scripts/run_demo.sh agent   (starts only the agent in foreground)
#        ./scripts/run_demo.sh sim     (starts only the simulation)

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"

print_usage() {
	echo "Run the full stack in 4 terminals (order matters):"
	echo ""
	echo "  Terminal 1 - Agent:"
	echo "    $WORKSPACE_ROOT/scripts/run_microxrce_agent.sh"
	echo ""
	echo "  Terminal 2 - Simulation:"
	echo "    $WORKSPACE_ROOT/scripts/run_sim_four_drones.sh"
	echo ""
	echo "  Terminal 3 - Scout + fusion:"
	echo "    source $WORKSPACE_ROOT/install/setup.bash"
	echo "    ros2 launch multi_scout multi_scout.launch.py"
	echo ""
	echo "  Terminal 4 - MRTA:"
	echo "    ros2 launch task_allocation mrta.launch.py"
	echo ""
	echo "Optional: ros2 run task_allocation mrta_panel   (live MRTA UI)"
	echo "          ros2 run rqt_image_view rqt_image_view   (camera/map)"
}

case "${1:-}" in
	--help|-h) print_usage; exit 0 ;;
	agent)     exec "$WORKSPACE_ROOT/scripts/run_microxrce_agent.sh" ;;
	sim)       exec "$WORKSPACE_ROOT/scripts/run_sim_four_drones.sh" "$@" ;;
	*)         print_usage ;;
esac
