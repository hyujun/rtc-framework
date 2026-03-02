#!/bin/bash
# install.sh - Complete Installation Script for UR5e RT Controller

set -e  # Exit on error

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🚀 UR5e RT Controller v4.0.0 Installation${NC}"
echo "=================================================="

# Check ROS2
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}❌ ROS2 not found. Please install ROS2 Humble first.${NC}"
    exit 1
fi

ROS_DISTRO=$(ros2 --version | grep -oP 'ROS \K[^ ]+')
echo -e "${GREEN}✅ ROS2 detected: ${ROS_DISTRO}${NC}"

# Check Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
if [[ "$UBUNTU_VERSION" != "22.04" ]]; then
    echo -e "${YELLOW}⚠️  Non-standard Ubuntu ($UBUNTU_VERSION). Continuing...${NC}"
fi

# Create workspace
WORKSPACE=~/ur_ws
echo -e "${BLUE}📁 Setting up workspace: $WORKSPACE${NC}"
mkdir -p $WORKSPACE/src
cd $WORKSPACE/src

# Install system dependencies
echo -e "${BLUE}📦 Installing system dependencies...${NC}"
sudo apt update

# UR Robot Driver
sudo apt install -y \
    ros-humble-ur-robot-driver \
    ros-humble-ur-msgs \
    ros-humble-ur-description \
    ros-humble-control-msgs \
    ros-humble-industrial-msgs

# Build tools
sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-gtest \
    ros-humble-ament-lint \
    python3-colcon-common-extensions \
    python3-vcstool

# Python visualization
pip3 install --user -U matplotlib pandas numpy scipy

echo -e "${GREEN}✅ System dependencies installed${NC}"

# Clone package (if not exists)
if [[ ! -d "ur5e_rt_controller" ]]; then
    echo -e "${BLUE}📥 Cloning ur5e_rt_controller...${NC}"
    git clone https://github.com/your-repo/ur5e_rt_controller.git
    cd ur5e_rt_controller
    ./organize_files.sh
else
    echo -e "${YELLOW}📁 ur5e_rt_controller already exists${NC}"
    cd ur5e_rt_controller
fi

# Make executables
echo -e "${BLUE}🔧 Setting executable permissions...${NC}"
chmod +x scripts/*.py launch/*.launch.py install.sh organize_files.sh

# Build
echo -e "${BLUE}🔨 Building package...${NC}"
cd $WORKSPACE
colcon build --packages-select ur5e_rt_controller --symlink-install

# Source
source install/setup.bash
echo "export ROS_WORKSPACE=$WORKSPACE" >> ~/.bashrc
echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc

# Verify installation
echo -e "${BLUE}✅ Verifying installation...${NC}"
if ros2 pkg list | grep -q ur5e_rt_controller; then
    echo -e "${GREEN}✅ Package successfully installed!${NC}"
else
    echo -e "${RED}❌ Installation failed${NC}"
    exit 1
fi

# Test executables
echo -e "${BLUE}🔍 Testing executables...${NC}"
ros2 pkg executables ur5e_rt_controller

# Create log directories
mkdir -p /tmp/ur5e_logs /tmp/ur5e_stats ~/ur_plots
echo -e "${GREEN}✅ Log directories created${NC}"

echo ""
echo -e "${GREEN}🎉 Installation COMPLETE!${NC}"
echo "=================================================="
echo ""
echo "🚀 Quick Start Commands:"
echo ""
echo "1. Full system:"
echo "   ros2 launch ur5e_rt_controller ur_control.launch.py robot_ip:=192.168.1.10"
echo ""
echo "2. Hand UDP only:"
echo "   ros2 launch ur5e_rt_controller hand_udp.launch.py"
echo ""
echo "3. Health monitor:"
echo "   ros2 run ur5e_rt_controller monitor_data_health_v2.py"
echo ""
echo "📊 Logs saved to: /tmp/ur5e_control_log.csv"
echo ""
echo "🔧 Troubleshooting: docs/troubleshooting.md"
