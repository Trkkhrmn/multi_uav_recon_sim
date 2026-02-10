#!/usr/bin/env bash
# Creates symlinks under assets/ to ~/gazebo_maps for common_models and mili_tech.
# Run once when you first set up the workspace.

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ASSETS_DIR="${WORKSPACE_ROOT}/assets"
GAZEBO_MAPS="${GAZEBO_MAPS_DIR:-$HOME/gazebo_maps}"

mkdir -p "$ASSETS_DIR"
cd "$ASSETS_DIR"

for name in common_models mili_tech; do
	if [[ -L "$name" ]]; then
		echo "  $name already a symlink."
	elif [[ -d "$name" ]]; then
		echo "  $name already exists (dir)."
	elif [[ -d "$GAZEBO_MAPS/$name" ]]; then
		ln -s "$GAZEBO_MAPS/$name" "$name"
		echo "  $name -> $GAZEBO_MAPS/$name"
	else
		echo "  Warning: $GAZEBO_MAPS/$name not found, skipping $name."
	fi
done
echo "Done. If needed: GAZEBO_MAPS_DIR=/path/to/gazebo_maps $0"
