#!/bin/bash
# This script saves the map using the map_saver_cli tool from the nav2_map_server package.
# It generates a timestamped filename for the saved map files.

# Timestamped default filename
FILE_TIMESTAMP=$(date +"%d-%m-%Y_%H-%M")
DEFAULT_SAVE_FILE="map_$FILE_TIMESTAMP"

# Use first argument if provided, otherwise use default
SAVE_FILE=${1:-$DEFAULT_SAVE_FILE}

echo "Save file: $SAVE_FILE"

ros2 run nav2_map_server map_saver_cli -f my_map