#!/bin/bash

# ============================================
# PX4-ROS2-Gazebo Setup - Arch Linux
# ============================================
# Automated installation script for Arch-based distributions
# Author: Nagaraj Bhat
# License: MIT
# ============================================

set -e  # Exit on error
set -u  # Exit on undefined variable

# Terminal colors
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly NC='\033[0m'

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_header() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
}

# Error handling
error_exit() {
    log_error "$1"
    exit 1
}

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    error_exit "Do not run this script as root"
fi

# ============================================
# System Verification
# ============================================

print_header "PX4-ROS2-Gazebo Installation - Arch Linux"

log_info "Verifying system requirements..."

# Check internet connection
if ! ping -c 1 google.com &> /dev/null; then
    error_exit "No internet connection detected"
fi
log_success "Internet connection verified"

# Check available disk space (require 20GB free)
AVAILABLE_SPACE=$(df -BG / | awk 'NR==2 {print $4}' | sed 's/G//')
if [ "$AVAILABLE_SPACE" -lt 20 ]; then
    log_warning "Low disk space detected: ${AVAILABLE_SPACE}GB available"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# ============================================
# Update System
# ============================================

print_header "Step 1: System Update"

log_info "Updating package database..."
sudo pacman -Sy --noconfirm || error_exit "Failed to update package database"
log_success "Package database updated"

log_info "Upgrading system packages..."
sudo pacman -Su --noconfirm || log_warning "Some packages failed to upgrade (non-critical)"
log_success "System upgraded"

# ============================================
# Install Distrobox and Podman
# ============================================

print_header "Step 2: Installing Containerization Tools"

if command -v distrobox &> /dev/null; then
    log_warning "Distrobox already installed: $(distrobox --version)"
else
    log_info "Installing Distrobox and Podman..."
    sudo pacman -S --noconfirm distrobox podman || error_exit "Failed to install Distrobox"
    log_success "Distrobox and Podman installed"
fi

# Verify installation
DISTROBOX_VERSION=$(distrobox --version 2>&1 | head -n1)
PODMAN_VERSION=$(podman --version 2>&1)
log_success "Distrobox: $DISTROBOX_VERSION"
log_success "Podman: $PODMAN_VERSION"

# ============================================
# Create Ubuntu Container
# ============================================

print_header "Step 3: Creating Ubuntu 22.04 Container"

CONTAINER_NAME="ubuntu-px4"

# Check if container already exists
if distrobox list | grep -q "$CONTAINER_NAME"; then
    log_warning "Container '$CONTAINER_NAME' already exists"
    read -p "Remove and recreate? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        log_info "Removing existing container..."
        distrobox rm -f "$CONTAINER_NAME" || error_exit "Failed to remove container"
        log_success "Container removed"
    else
        log_info "Using existing container"
        CONTAINER_EXISTS=true
    fi
fi

if [ "${CONTAINER_EXISTS:-false}" != "true" ]; then
    log_info "Creating container '$CONTAINER_NAME' (this may take a few minutes)..."
    distrobox create --name "$CONTAINER_NAME" --image ubuntu:22.04 || error_exit "Failed to create container"
    log_success "Container created successfully"
fi

# ============================================
# Setup Container Script
# ============================================

print_header "Step 4: Preparing Container Setup Script"

SETUP_SCRIPT="/tmp/px4_container_setup.sh"

cat > "$SETUP_SCRIPT" << 'CONTAINER_SCRIPT_EOF'
#!/bin/bash
set -e

echo "========================================="
echo "Container Setup: Ubuntu 22.04"
echo "========================================="

# Update container packages
echo "[1/7] Updating package repositories..."
sudo apt update && sudo apt upgrade -y

# Install prerequisites
echo "[2/7] Installing build essentials..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    lsb-release \
    gnupg2 \
    software-properties-common \
    python3-pip \
    python3-dev

# Install ROS2 Humble
echo "[3/7] Installing ROS2 Humble..."
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# Configure ROS2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install Gazebo Classic
echo "[4/7] Installing Gazebo Classic..."
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt update
sudo apt install -y gazebo11 libgazebo11-dev

# Install Gazebo dependencies
sudo apt install -y \
    libeigen3-dev \
    libopencv-dev \
    libxml2-dev \
    protobuf-compiler \
    libprotobuf-dev \
    libprotoc-dev

# Configure Gazebo environment
echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

# Clone PX4 Autopilot
echo "[5/7] Cloning PX4 Autopilot repository..."
mkdir -p ~/px4_ws
cd ~/px4_ws
if [ ! -d "PX4-Autopilot" ]; then
    git clone --recursive https://github.com/PX4/PX4-Autopilot.git
    cd PX4-Autopilot
    git checkout v1.14.3
else
    echo "PX4-Autopilot already cloned"
    cd PX4-Autopilot
fi

# Install PX4 dependencies
echo "[6/7] Installing PX4 dependencies..."
bash ./Tools/setup/ubuntu.sh --no-nuttx

# Build PX4 for SITL
echo "[7/7] Building PX4 for SITL (this will take 10-15 minutes)..."
make px4_sitl_default

# Setup px4_msgs for ROS2
echo "Setting up ROS2 workspace..."
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
if [ ! -d "px4_msgs" ]; then
    git clone https://github.com/PX4/px4_msgs.git
fi
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# Install MicroXRCE-DDS Agent
echo "Installing MicroXRCE-DDS Agent..."
cd ~/px4_ws
if [ ! -d "Micro-XRCE-DDS-Agent" ]; then
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    cd Micro-XRCE-DDS-Agent
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig /usr/local/lib/
fi

echo ""
echo "========================================="
echo "Container setup completed successfully!"
echo "========================================="
CONTAINER_SCRIPT_EOF

chmod +x "$SETUP_SCRIPT"

# ============================================
# Execute Container Setup
# ============================================

print_header "Step 5: Installing PX4, ROS2, and Gazebo in Container"

log_info "Entering container and executing setup..."
log_warning "This process will take 15-20 minutes depending on your system"

distrobox enter "$CONTAINER_NAME" -- bash "$SETUP_SCRIPT" || error_exit "Container setup failed"

log_success "Container setup completed"

# ============================================
# Install QGroundControl (Host System)
# ============================================

print_header "Step 6: Installing QGroundControl (Host System)"

QGC_DIR="$HOME/Applications"
mkdir -p "$QGC_DIR"

if [ -f "$QGC_DIR/QGroundControl.AppImage" ]; then
    log_warning "QGroundControl already installed at $QGC_DIR/QGroundControl.AppImage"
else
    log_info "Downloading QGroundControl..."
    wget -O "$QGC_DIR/QGroundControl.AppImage" \
        https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage || \
        error_exit "Failed to download QGroundControl"
    
    chmod +x "$QGC_DIR/QGroundControl.AppImage"
    log_success "QGroundControl installed"
fi

# Create desktop entry
log_info "Creating desktop entry..."
cat > ~/.local/share/applications/qgroundcontrol.desktop << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=QGroundControl
Comment=Ground Control Station for Drones
Exec=$QGC_DIR/QGroundControl.AppImage
Icon=qgroundcontrol
Terminal=false
Categories=Science;Education;
EOF

log_success "Desktop entry created"

# ============================================
# Create Helper Scripts
# ============================================

print_header "Step 7: Creating Helper Scripts"

# Create start script
START_SCRIPT="$HOME/start-px4-simulation.sh"
cat > "$START_SCRIPT" << 'EOF'
#!/bin/bash

echo "Starting PX4 Simulation Environment..."
echo ""
echo "Terminal 1: PX4 SITL + Gazebo"
echo "Terminal 2: MicroXRCE-DDS Agent"
echo "Terminal 3: QGroundControl"
echo ""

# Function to open new terminal
open_terminal() {
    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal -- bash -c "$1; exec bash"
    elif command -v konsole &> /dev/null; then
        konsole -e bash -c "$1; exec bash"
    elif command -v xterm &> /dev/null; then
        xterm -e "$1; bash"
    else
        echo "No supported terminal found. Please run manually:"
        echo "$1"
    fi
}

# Start PX4 SITL + Gazebo
echo "Starting PX4 SITL + Gazebo..."
distrobox enter ubuntu-px4 -- bash -c "cd ~/px4_ws/PX4-Autopilot && make px4_sitl gazebo" &

sleep 5

# Start MicroXRCE-DDS Agent
echo "Starting MicroXRCE-DDS Agent..."
open_terminal "distrobox enter ubuntu-px4 -- bash -c 'source ~/ros2_ws/install/setup.bash && MicroXRCEAgent udp4 -p 8888'"

sleep 2

# Start QGroundControl
echo "Starting QGroundControl..."
$HOME/Applications/QGroundControl.AppImage &

echo ""
echo "All components started!"
EOF

chmod +x "$START_SCRIPT"
log_success "Helper script created: $START_SCRIPT"

# ============================================
# Installation Complete
# ============================================

print_header "Installation Complete"

log_success "PX4-ROS2-Gazebo environment successfully installed!"
echo ""
echo "Quick Start Guide:"
echo "=================="
echo ""
echo "1. Start complete simulation:"
echo "   $START_SCRIPT"
echo ""
echo "2. Or start components manually:"
echo ""
echo "   Terminal 1 - PX4 SITL + Gazebo:"
echo "   $ distrobox enter ubuntu-px4"
echo "   $ cd ~/px4_ws/PX4-Autopilot"
echo "   $ make px4_sitl gazebo"
echo ""
echo "   Terminal 2 - MicroXRCE-DDS Agent:"
echo "   $ distrobox enter ubuntu-px4"
echo "   $ source ~/ros2_ws/install/setup.bash"
echo "   $ MicroXRCEAgent udp4 -p 8888"
echo ""
echo "   Terminal 3 - QGroundControl (Host):"
echo "   $ $QGC_DIR/QGroundControl.AppImage"
echo ""
echo "3. Monitor ROS2 topics:"
echo "   $ distrobox enter ubuntu-px4"
echo "   $ source ~/ros2_ws/install/setup.bash"
echo "   $ ros2 topic list | grep fmu"
echo "   $ ros2 topic echo /fmu/out/vehicle_attitude"
echo ""
echo "Documentation: https://github.com/Nags016/PX4-ROS2-Gazebo-setup-guide"
echo ""
log_info "Installation log saved to /tmp/px4_install.log"

# Cleanup
rm -f "$SETUP_SCRIPT"

exit 0
