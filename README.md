# PX4-ROS2-Gazebo Setup Guide (with Distrobox)

A clean, beginner-friendly guide to installing and running PX4 SITL, ROS2, Gazebo Classic, and QGroundControl, using Distrobox for maximum compatibility across any Linux distribution.

This guide is written to be simple, stable, minimal, and developer-friendly.

![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)

### 1. Overview

This repository helps you set up a complete drone simulation environment that includes:

ROS2 Humble

PX4 Autopilot (SITL)

Gazebo Classic simulator

QGroundControl

Distrobox + Ubuntu 22.04 container (for non-Ubuntu systems)

After completing this setup, you will be able to run:

Drone SITL simulation

Gazebo world with PX4

QGroundControl connected via MAVLink

ROS2 topics from PX4


### 2. Requirements
If you use Arch / Garuda / Manjaro / Fedora / Any non-Ubuntu distro:

You must install Distrobox, because PX4 + ROS2 + Gazebo work best inside Ubuntu 22.04.

If you use Ubuntu / Debian:

You can skip Distrobox and install everything directly on your system.


### 3. Install Distrobox (Only for non-Ubuntu systems)

Install Distrobox + Podman:
```bash
sudo pacman -S distrobox podman        # For Arch / Garuda
```

Create and enter an Ubuntu 22.04 container:
```bash
distrobox create -n ubuntu-px4 --image ubuntu:22.04
distrobox enter ubuntu-px4
```

### 4. Install ROS2 Humble (Inside Distrobox)

Update packages:
```bash
sudo apt update
```

Install ROS2:
```bash
sudo apt install -y ros-humble-desktop
```

Source ROS2 automatically:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. Install PX4 Autopilot (Inside Distrobox)

Clone PX4:
```bash
cd ~
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
```

Run PX4 setup script:
```bash
bash ./Tools/setup/ubuntu.sh --no-nuttx
```

### 6. Install Gazebo Classic (Inside Distrobox)

Install Gazebo Classic and required dependencies:
```bash
sudo apt update
sudo apt install -y \
    gazebo \
    libgazebo-dev \
    libeigen3-dev \
    libopencv-dev \
    ninja-build \
    cmake \
    git \
    python3-pip \
    protobuf-compiler
```

Test Gazebo:
``` bash
gazebo
```

### 7. Build PX4 SITL with Gazebo (Inside Distrobox)
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo -j$(nproc)
```
If the build completes successfully, PX4 SITL + Gazebo will launch automatically.

### 8. Install QGroundControl (Run on Host System)

Download the AppImage:
``` bash
cd ~/Downloads
wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.4.0/QGroundControl.AppImage -O QGC.AppImage
chmod +x QGC.AppImage
./QGC.AppImage
```

If your system uses Wayland:
```bash
QT_QPA_PLATFORM=xcb ./QGC.AppImage
```
### 9. PX4 → Gazebo → QGroundControl Connection

PX4 SITL automatically exposes the following ports:

Purpose	Port
Simulator (TCP)	4560
SITL MAVLink	14540
QGC listening port	14550

If QGC does not auto-connect, enter the PX4 shell (pxh>) and run:
``` bash
param set MAV_0_BROADCAST 1
reboot
```

QGroundControl will detect the SITL automatically after reboot.

### 10. Running Everything
Start Distrobox:
```bash
distrobox enter ubuntu-px4
```
Start PX4 SITL:
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```
Launch QGroundControl (on host):
```bash
./QGC.AppImage
```
All components will connect automatically.

### 11. Simulation Workflow Structure
```bash
Host System (Linux)
│
├── QGroundControl (AppImage)
│
└── Distrobox: Ubuntu 22.04 Container
      ├── PX4 SITL
      ├── Gazebo Classic
      └── ROS2 Humble
```

### Contact

If you face any issues, suggestions, or want to contribute:

Email: nagarajsbhat12@gmail.com

Feel free to open issues or contribute to improvements in this repository.
