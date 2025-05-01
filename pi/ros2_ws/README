ROS 2 Workspace Overview
=========================

This is the main ROS 2 workspace for the Roambot project. It includes custom packages and configuration files necessary for LiDAR processing, odometry, robot state publishing, and navigation.

Workspace Structure:

1. **src/** — Contains custom ROS 2 packages:
   - **driver_ros2**: Listens to the `cmd_vel` topic and converts the velocity commands into serial motor commands for the Redboard microcontroller.
   - **sllidar_ros2**: Captures and publishes LiDAR data from the RPLIDAR A1.
   - **rf2o_laser_odometry**: Converts LiDAR scan data into odometry information published on the `/odom` topic.

2. **opt/** — Stores configuration and support files:
   - **slam_toolbox.yaml**: Custom configuration for SLAM Toolbox.
   - **my_robot.urdf**: Defines the robot geometry and frame relationships for transformation publishing.
   - **Saved RViz Files**: Store visual configurations for quick testing and demos.
   - **Shell Scripts**:
     - **lidar.sh**: Launches the LiDAR driver node.
     - **odom.sh**: Starts laser odometry using rf2o.
     - **map.sh**: Launches SLAM Toolbox for mapping.
     - **demo.sh**: Full system demo with navigation using Nav2.

Usage:
------
- Build the workspace using `colcon build` from the root of `ros2_ws`.
- Source the workspace: `source install/setup.bash`
- Use the provided `.sh` scripts in `opt/` to run individual components or full system demos.

This folder is essential for building and running the robot's ROS 2-based autonomy stack.
