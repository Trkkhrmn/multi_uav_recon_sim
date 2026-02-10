#!/usr/bin/env bash
# Micro XRCE-DDS Agent — bridge between PX4 and ROS 2.
# PX4 SITL (rcS) expects the agent on UDP port 8888 by default.
# Run this script in a terminal *before* starting the sim and leave it running.

PORT="${PX4_UXRCE_DDS_PORT:-8888}"

if ! command -v MicroXRCEAgent &>/dev/null; then
	echo "MicroXRCEAgent not found. Install it:"
	echo "  sudo snap install micro-xrce-dds-agent"
	echo "  or from source: https://github.com/eProsima/Micro-XRCE-DDS-Agent"
	exit 1
fi

echo "Starting Micro XRCE-DDS Agent (UDP port $PORT)..."
echo "Run the sim (run_sim_four_drones.sh) and multi_scout / MRTA launches in other terminals."
echo "Ctrl+C to stop."
exec MicroXRCEAgent udp4 -p "$PORT"
