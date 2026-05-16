#!/usr/bin/env bash
# Save the current slam_toolbox occupancy grid to bringup/maps/.
#
# Usage:
#     ros2 run morphonav_bringup save_map.sh                    # saves morphonav_map.{pgm,yaml}
#     ros2 run morphonav_bringup save_map.sh my_office          # saves my_office.{pgm,yaml}
#
# The map is written to the installed share/morphonav_bringup/maps/ directory
# (and back to the source tree if you built with --symlink-install).

set -euo pipefail

NAME="${1:-morphonav_map}"
OUT_DIR="$(ros2 pkg prefix morphonav_bringup)/share/morphonav_bringup/maps"

mkdir -p "$OUT_DIR"
ros2 run nav2_map_server map_saver_cli -f "$OUT_DIR/$NAME"

echo
echo "Saved:"
ls -l "$OUT_DIR/$NAME".pgm "$OUT_DIR/$NAME".yaml
