#!/bin/bash
# chmod +x demo.sh

# Start driver node
ros2 run driver_ros2 cmd_vel_to_serial &

# Give access to the LiDAR port
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1

sudo chmod 777 /dev/mcu
sudo chmod 777 /dev/lidar

# Start sllidar node
ros2 launch sllidar_ros2 sllidar_a1_launch.py &

# Start rf2o_laser_odometry with filtered logs
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py | grep -v "\[INFO\]" | grep -v "\[WARN\]" &

# Start robot_state_publisher
# ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/tlc/ros2_ws/opt/my_robot.urdf)" &
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /home/tlc/ros2_ws/opt/my_robot.urdf)" -p publish_frequency:=20.0 &

# Start SLAM Toolbox in async mode for mapping
sleep 2 && ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/tlc/ros2_ws/opt/slam_toolbox.yaml &

# launch nav2
sleep 2 && ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false autostart:=true &

# Start RViz with map view
rviz2 -d /home/tlc/ros2_ws/opt/nav.rviz

# Set traps to clean up when exiting (both EXIT and SIGINT)
trap "echo 'Cleaning up...'; pkill -f ros2; exit" EXIT SIGINT

# Wait for all background jobs to finish
wait

