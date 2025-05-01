# RoamBot ROS 2 Jazzy Setup Guide

This guide explains how to install and run the `roam_bot` package using ROS 2 Jazzy on a Raspberry Pi. It includes necessary dependency installations and instructions for running the bot with both mock and real hardware.

---

## 🧰 Prerequisites

- A Raspberry Pi running Ubuntu 22.04 (64-bit)
- ROS 2 Jazzy installed
- `colcon` and `vcstool` installed
- Workspace initialized (e.g., `~/ros2_ws`)

---

## 📦 Installation

### 1. Install ROS 2 Jazzy

If ROS 2 Jazzy is not installed on your Raspberry Pi, follow the official guide:

👉 [Install ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### 2. Install ros2_control and Dependencies

Install `ros2_control` core packages:

```bash
sudo apt update
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

Install serial communication library:

```bash
sudo apt install libserial-dev
```

You can optionally follow the official guide for setting up `ros2_control`:

👉 [Getting Started with ros2_control](https://control.ros.org/jazzy/doc/getting_started/getting_started.html)

---

## 🏗️ Workspace Setup

Create and build your ROS 2 workspace if you haven't already:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Clone your `roam_bot` and other required packages into the `src/` folder before building.

---

## 🔧 Environment Setup for Real Hardware

Before running your application with **real hardware**, set the following environment variable:

```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

> You may add this to your `.bashrc` to set it automatically on boot.

---

## 🚀 Running the RoamBot

### 1. Test the Robot Visualization

To verify that the robot URDF and control setup works:

```bash
ros2 launch ros2_control_demo_example_2 view_robot.launch.py
```

### 2. Run with Mock Hardware

To simulate the robot:

```bash
ros2 launch roam_bot diffbot.launch.py use_mock_hardware:=True
```

> Open a second terminal, **source your workspace**, and run the same command to control the robot:
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch roam_bot diffbot.launch.py use_mock_hardware:=True
```

### 3. Run with Real Hardware

Ensure the environment variable is set (see above), then run:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch roam_bot diffbot.launch.py use_mock_hardware:=False
```

---

## 🧹 Troubleshooting

- Ensure all terminals are sourced with `source ~/ros2_ws/install/setup.bash`
- Use `ros2 interface list` and `ros2 topic echo` to verify data flow
- Use `rqt_graph` or `rqt` for visual debugging

---

## 📚 References

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [ros2_control Docs](https://control.ros.org/jazzy/)
- [Serial Library](https://github.com/wjwwood/serial)