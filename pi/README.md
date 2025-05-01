Raspberry Pi Directory Overview
==========================

This repository contains resources to set up and test a Raspberry Pi 5-based robot system. It is divided into four key sections:

1. **roambot/** - For testing serial (current) and GPIO (old) methods of sending data from the Raspberry Pi 5 to the microcontroller to control the wheels using Python.
2. **ros2_help/** - Contains a README with detailed instructions on installing ROS 2 and corresponding ROS 2 packages.
3. **ros2_ws/** - Holds custom ROS 2 packages and bash script files for launching various tests.
4. **udev.txt** - Instructions and rules to set up symbolic links under `/dev` for interfacing with the microcontroller and LiDAR via serial.

Below is additional setup and configuration information for the Raspberry Pi 5.


Raspberry Pi 5 Setup & Configuration (Team TLC)
===============================================

Device Overview
---------------
Device: Raspberry Pi 5B
RAM: 16GB
Hostname: pi16
Username: tlc
Password: tlc

Tailscale Connection:
- SSH: ssh tlc@100.66.165.102
- Address: 100.66.165.102 (via Tailscale)

Operating System
----------------
Ubuntu Server 24.04.2 LTS (64-bit)
- Codename: Noble Numbat (Debian-based)
- Image: ubuntu-24.04.2-preinstalled-server-arm64+raspi.img
- Download: https://ubuntu.com/download/raspberry-pi

Ubuntu Desktop 24.04.2 LTS (64-bit)
- Flashed with Raspberry Pi Imager (on Windows)
- MicroSD: 64GB
- Configuration:
  - Device: Raspberry Pi 5
  - Hostname: pi
  - Username: tlc
  - Password: tlc
  - Wi-Fi:
    - SSID: UTA Bonjour
    - Password: UTAbonjour

SD Card Recovery (Corruption Fix)
1. Download and install SD Card Formatter: https://www.sdcard.org/downloads/formatter/
2. Select card and use Overwrite format
3. Reflash image with Raspberry Pi Imager

Network Configuration
---------------------
ERB Lab 208 Wi-Fi
- SSID: skynet
- Password: t3rm1n4t0r

UTA Bonjour Wi-Fi (MAC Whitelist)
- SSID: UTA Bonjour
- Password: UTAbonjour
- MAC: 2ccf67c9b907

Netplan Configurations
----------------------

Ethernet (DHCP)
File: /etc/netplan/50-cloud-init.yaml

network:
  version: 2
  ethernets:
    eth0:
      optional: true
      dhcp4: true

Apply with: sudo netplan apply

Wi-Fi (EAP Authentication)
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "SSID":
          auth:
            key-management: eap
            identity: "username"
            password: "password"

Wi-Fi (PSK Authentication)
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "SSID":
          auth:
            key-management: "psk"
            password: "password"

Apply with: sudo netplan apply

IP Address History (Raspberry Pi 5)
-----------------------------------
- 10.236.1.127 — 4/16
- 10.236.1.127 — 4/17
- 10.236.1.127 — 4/18

System Info Commands
--------------------
Memory Info           : cat /proc/meminfo
CPU Temperature       : cat /sys/class/thermal/thermal_zone0/temp
CPU Usage             : top (press 1)
Network Details       : ifconfig
OS Info               : cat /etc/os-release
Edit Netplan Config   : sudo nano /etc/netplan/50-cloud-init.yaml
Shutdown              : sudo shutdown now

Team Accounts
-------------
Raspberry Pi
- Username: tlc
- Password: tlc

Outlook
- Email: teamtlc100@outlook.com
- Password: tlcrover123

Tailscale
- Email: teamtlc100@outlook.com
- Password: Single Sign-On (No Password)
