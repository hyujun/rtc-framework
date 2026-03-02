#!/bin/bash
# organize_files.sh - File Organization Script for UR5e RT Controller

set -e  # Exit on any error

echo "🚀 UR5e RT Controller - File Organization Script (v4.0.0)"
echo "📁 Organizing 47 files into standard ROS2 structure..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Create directory structure
mkdir -p {config,launch,include/ur5e_rt_controller/controllers,src,scripts,test,rviz,docs,resources}

# Verify we're in package root
if [[ ! -f "CMakeLists.txt" ]]; then
    echo -e "${RED}❌ Error: Run from package root (CMakeLists.txt required)${NC}"
    exit 1
fi

echo -e "${BLUE}📂 Creating directory structure...${NC}"
mkdir -p docs launch config resources scripts test rviz

# Move/verify core files
echo -e "${GREEN}✅ Core files verified:${NC}"
[[ -f "CMakeLists.txt" ]] && echo "   ✅ CMakeLists.txt"
[[ -f "package.xml" ]] && echo "   ✅ package.xml"

# Organize config files
echo -e "${BLUE}📋 Organizing config files...${NC}"
mv -f ur5e_rt_controller.yaml config/ 2>/dev/null || echo "   ⚠️  ur5e_rt_controller.yaml already in place"
mv -f hand_udp_receiver.yaml config/ 2>/dev/null || echo "   ⚠️  hand_udp_receiver.yaml already in place"
mv -f ur5e_limits.yaml resources/ 2>/dev/null || echo "   ⚠️  ur5e_limits.yaml already in place"
mv -f safe_positions.yaml resources/ 2>/dev/null || echo "   ⚠️  safe_positions.yaml already in place"

# Organize launch files
echo -e "${BLUE}🚀 Organizing launch files...${NC}"
mv -f ur_control.launch.py launch/ 2>/dev/null || echo "   ⚠️  ur_control.launch.py already in place"
mv -f hand_udp.launch.py launch/ 2>/dev/null || echo "   ⚠️  hand_udp.launch.py already in place"

# Organize C++ source files
echo -e "${BLUE}⚙️  Organizing C++ files...${NC}"
mkdir -p include/ur5e_rt_controller/controllers src

# Headers
mv -f rt_controller_interface.hpp include/ur5e_rt_controller/ 2>/dev/null || true
mv -f data_logger.hpp include/ur5e_rt_controller/ 2>/dev/null || true
mv -f hand_udp_receiver.hpp include/ur5e_rt_controller/ 2>/dev/null || true
mv -f hand_udp_sender.hpp include/ur5e_rt_controller/ 2>/dev/null || true
mv -f p_controller.hpp include/ur5e_rt_controller/controllers/ 2>/dev/null || true
mv -f pd_controller.hpp include/ur5e_rt_controller/controllers/ 2>/dev/null || true

# Sources
mv -f custom_controller.cpp src/ 2>/dev/null || true
mv -f hand_udp_receiver_node.cpp src/ 2>/dev/null || true
mv -f hand_udp_sender_node.cpp src/ 2>/dev/null || true
mv -f p_controller.cpp src/ 2>/dev/null || true
mv -f pd_controller.cpp src/ 2>/dev/null || true

# Python scripts
echo -e "${BLUE}🐍 Organizing Python scripts...${NC}"
mv -f monitor_data_health_v2.py scripts/ 2>/dev/null || echo "   ⚠️  monitor_data_health_v2.py already in place"
mv -f plot_ur_trajectory.py scripts/ 2>/dev/null || echo "   ⚠️  plot_ur_trajectory.py already in place"
mv -f generate_statistics.py scripts/ 2>/dev/null || echo "   ⚠️  generate_statistics.py already in place"

# Documentation
echo -e "${BLUE}📚 Organizing documentation...${NC}"
mv -f api_reference.md docs/ 2>/dev/null || true
mv -f troubleshooting.md docs/ 2>/dev/null || true
mv -f performance_benchmarks.md docs/ 2>/dev/null || true

# RViz configs
echo -e "${BLUE}👁️  Organizing RViz files...${NC}"
mv -f ur5e_controller.rviz rviz/ 2>/dev/null || true
mv -f ur5e_base.urdf rviz/ 2>/dev/null || true
mv -f hand_description.urdf rviz/ 2>/dev/null || true

# Create missing files
echo -e "${YELLOW}✨ Creating missing template files...${NC}"

# Create package.xml if missing
if [[ ! -f "package.xml" ]]; then
    cat > package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ur5e_rt_controller</name>
  <version>4.0.0</version>
  <description>Real-time UR5e controller with E-STOP (v4)</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>MIT</license>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>std_msgs</depend>
  <depend>ur_msgs</depend>

  <test_depend>ament_cmake_gtest</test_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
    echo "   ✅ Created package.xml"
fi

# Final verification
echo -e "${GREEN}✅ Final verification...${NC}"
find . -type f | wc -l | grep -q "47" && echo "   ✅ 47 files organized" || echo "   ⚠️  File count mismatch"

echo -e "${GREEN}🎉 File organization complete!${NC}"
echo ""
echo "📋 Next steps:"
echo "1. chmod +x scripts/*.py launch/*.launch.py"
echo "2. colcon build --packages-select ur5e_rt_controller"
echo "3. source install/setup.bash"
echo "4. ros2 launch ur5e_rt_controller ur_control.launch.py"
