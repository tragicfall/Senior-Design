#!/bin/bash
# chmod +x lidar.sh

# Grant permissions to USB device
sudo chmod 777 /dev/ttyUSB0

# Launch the RPLIDAR A1 node
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py

# Set traps to clean up when exiting (both EXIT and SIGINT)
trap "echo 'Cleaning up...'; pkill -f ros2; exit" EXIT SIGINT

# Wait for all background jobs to finish
wait
