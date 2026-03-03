#!/bin/bash
# install.sh - Complete Installation Script for UR5e RT Controller

set -e  # Exit on error

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🚀 UR5e RT Controller v4.3.0 Installation${NC}"
echo "==================================================="

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

# Pinocchio — rigid-body dynamics library (required for model-based controllers v4.3.0+)
echo -e "${BLUE}📦 Installing Pinocchio...${NC}"
if sudo apt install -y ros-humble-pinocchio 2>/dev/null; then
    echo -e "${GREEN}✅ Pinocchio installed via ros-humble-pinocchio${NC}"
else
    echo -e "${YELLOW}⚠️  ros-humble-pinocchio not found, trying robotpkg...${NC}"
    sudo sh -c "echo 'deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg' \
        > /etc/apt/sources.list.d/robotpkg.list"
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.key \
        | sudo apt-key add - 2>/dev/null || true
    sudo apt update
    if sudo apt install -y robotpkg-py310-pinocchio 2>/dev/null; then
        echo -e "${GREEN}✅ Pinocchio installed via robotpkg${NC}"
        echo "export PATH=/opt/openrobots/bin:\$PATH" >> ~/.bashrc
        echo "export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:\$PKG_CONFIG_PATH" >> ~/.bashrc
        echo "export LD_LIBRARY_PATH=/opt/openrobots/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
        echo "export CMAKE_PREFIX_PATH=/opt/openrobots:\$CMAKE_PREFIX_PATH" >> ~/.bashrc
    else
        echo -e "${YELLOW}⚠️  Pinocchio not installed — PinocchioController / ClikController / OperationalSpaceController will be unavailable${NC}"
        echo -e "${YELLOW}   See: https://stack-of-tasks.github.io/pinocchio/download.html${NC}"
    fi
fi

# Build tools
sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-gtest \
    ros-humble-ament-lint \
    python3-colcon-common-extensions \
    python3-vcstool

# Python dependencies from requirements.txt
echo -e "${BLUE}🐍 Installing Python dependencies...${NC}"
if [[ -f "ur5e-rt-controller/requirements.txt" ]]; then
    pip3 install --user -r ur5e-rt-controller/requirements.txt
else
    echo -e "${YELLOW}⚠️  requirements.txt not found, installing defaults${NC}"
    pip3 install --user matplotlib pandas numpy scipy
fi

echo -e "${GREEN}✅ System dependencies installed${NC}"

# Clone package (if not exists)
if [[ ! -d "ur5e-rt-controller" ]]; then
    echo -e "${BLUE}📥 Cloning ur5e-rt-controller...${NC}"
    git clone https://github.com/hyujun/ur5e-rt-controller.git
    cd ur5e-rt-controller
else
    echo -e "${YELLOW}📁 ur5e-rt-controller already exists${NC}"
    cd ur5e-rt-controller
fi

# Build
echo -e "${BLUE}🔨 Building package...${NC}"
cd $WORKSPACE
colcon build --packages-select ur5e_rt_controller --symlink-install

# Source
source install/setup.bash
echo "export ROS_WORKSPACE=$WORKSPACE" >> ~/.bashrc
echo "source $WORKSPACE/install/setup.bash" >> ~/.bashrc

# RT permissions for v4.2.0+ parallel computing
echo ""
echo -e "${BLUE}⚙️  Setting up RT permissions for v4.2.0+ parallel computing...${NC}"
if ! groups | grep -q realtime; then
    echo -e "${YELLOW}Creating realtime group and setting limits...${NC}"
    sudo groupadd -f realtime
    sudo usermod -aG realtime $USER
    
    # Check if already configured
    if ! grep -q "@realtime.*rtprio" /etc/security/limits.conf; then
        echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
    fi
    if ! grep -q "@realtime.*memlock" /etc/security/limits.conf; then
        echo "@realtime - memlock unlimited" | sudo tee -a /etc/security/limits.conf
    fi
    
    echo -e "${GREEN}✅ RT permissions configured${NC}"
    echo -e "${YELLOW}⚠️  IMPORTANT: Please LOGOUT and LOGIN again for RT permissions to take effect${NC}"
    echo -e "${YELLOW}   Then verify with: ulimit -r (should show 99)${NC}"
else
    echo -e "${GREEN}✅ RT permissions already configured${NC}"
fi

# Verify installation
echo ""
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
echo "==================================================="
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
echo "   ros2 run ur5e_rt_controller monitor_data_health.py"
echo ""
echo "4. Plot trajectory:"
echo "   ros2 run ur5e_rt_controller plot_ur_trajectory.py /tmp/ur5e_control_log.csv"
echo ""
echo "5. Motion editor GUI:"
echo "   ros2 run ur5e_rt_controller motion_editor_gui.py"
echo ""
echo "6. Hand simulator:"
echo "   ros2 run ur5e_rt_controller hand_udp_sender_example.py"
echo ""
echo "📋 Logs saved to: /tmp/ur5e_control_log.csv"
echo ""
echo "📚 Additional resources:"
echo "   - RT optimization guide: docs/RT_OPTIMIZATION.md"
echo "   - Full documentation: README.md"
echo "   - Changelog: docs/CHANGELOG.md"
echo ""
echo "⚠️  For v4.2.0+ parallel computing:"
echo "   1. Logout and login after installation"
echo "   2. Verify RT permissions: ulimit -r (should be 99)"
echo "   3. See docs/RT_OPTIMIZATION.md for CPU isolation setup"
echo ""
echo "🤖 Pinocchio-based controllers (v4.3.0+):"
echo "   PinocchioController  — joint-space PD + gravity/Coriolis compensation"
echo "   ClikController       — Cartesian position control (CLIK, 3-DOF + null-space)"
echo "   OperationalSpaceController — full 6-DOF Cartesian PD (position + orientation)"
echo "   URDF path required:  /opt/ros/humble/share/ur_description/urdf/ur5e.urdf"
echo "   See README.md for usage and migration guide"
