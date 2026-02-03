#!/bin/bash
# Phase 2: Install Dependencies for SLAM and Nav2 (Fixed Version)
# This script checks for ROS2 first and provides helpful error messages

echo "=========================================="
echo "Phase 2: Installing Dependencies"
echo "=========================================="
echo ""

# Check if ROS2 is installed
echo "Step 0: Checking for ROS2 Humble..."
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble found at /opt/ros/humble"
elif [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source $HOME/ros2_ws/install/setup.bash
    echo "✅ ROS2 workspace found at $HOME/ros2_ws"
else
    echo "❌ ERROR: ROS2 Humble not found!"
    echo ""
    echo "ROS2 Humble must be installed first."
    echo "See ROS2_INSTALLATION_GUIDE.md for installation instructions."
    echo ""
    echo "Quick install:"
    echo "  sudo apt update"
    echo "  sudo apt install software-properties-common"
    echo "  sudo add-apt-repository universe"
    echo "  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -"
    echo "  sudo sh -c 'echo \"deb [arch=\$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu \$(lsb_release -cs) main\" > /etc/apt/sources.list.d/ros2-latest.list'"
    echo "  sudo apt update"
    echo "  sudo apt install ros-humble-desktop -y"
    echo "  source /opt/ros/humble/setup.bash"
    echo ""
    exit 1
fi

# Check if ros2 command works
if ! command -v ros2 &> /dev/null; then
    echo "❌ ERROR: ros2 command not found after sourcing ROS2"
    echo "Please source ROS2 manually: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS2 is working"
echo ""

# Update package list
echo "Step 1: Updating package list..."
sudo apt update

# Install SLAM_Toolbox
echo ""
echo "Step 2: Installing SLAM_Toolbox..."
if sudo apt install -y ros-humble-slam-toolbox; then
    echo "✅ SLAM_Toolbox installed successfully"
else
    echo "❌ Failed to install SLAM_Toolbox"
    echo "This might mean ROS2 repositories are not configured."
    echo "See ROS2_INSTALLATION_GUIDE.md for help."
fi

# Install Nav2 Stack
echo ""
echo "Step 3: Installing Nav2 Stack..."
if sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup; then
    echo "✅ Nav2 Stack installed successfully"
else
    echo "❌ Failed to install Nav2 Stack"
    echo "This might mean ROS2 repositories are not configured."
    echo "See ROS2_INSTALLATION_GUIDE.md for help."
fi

# Install Map Server
echo ""
echo "Step 4: Installing Nav2 Map Server..."
if sudo apt install -y ros-humble-nav2-map-server; then
    echo "✅ Nav2 Map Server installed successfully"
else
    echo "❌ Failed to install Nav2 Map Server"
    echo "This might mean ROS2 repositories are not configured."
    echo "See ROS2_INSTALLATION_GUIDE.md for help."
fi

# Verify installations
echo ""
echo "=========================================="
echo "Verifying Installations..."
echo "=========================================="

# Source ROS2 again to ensure environment is set
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

echo ""
echo "Checking SLAM_Toolbox:"
if ros2 pkg list | grep -q slam_toolbox; then
    echo "✅ SLAM_Toolbox installed"
    ros2 pkg list | grep slam_toolbox
else
    echo "❌ SLAM_Toolbox NOT found"
fi

echo ""
echo "Checking Nav2 packages:"
nav2_count=$(ros2 pkg list | grep nav2 | wc -l)
if [ $nav2_count -gt 0 ]; then
    echo "✅ Nav2 installed ($nav2_count packages)"
    ros2 pkg list | grep nav2 | head -5
else
    echo "❌ Nav2 NOT found"
fi

echo ""
echo "Checking Nav2 Map Server:"
if ros2 pkg list | grep -q nav2_map_server; then
    echo "✅ Nav2 Map Server installed"
    ros2 pkg list | grep nav2_map_server
else
    echo "❌ Nav2 Map Server NOT found"
fi

echo ""
echo "=========================================="
if ros2 pkg list | grep -q slam_toolbox && [ $(ros2 pkg list | grep nav2 | wc -l) -gt 0 ]; then
    echo "✅ Phase 2 Complete! All packages installed."
    echo "Proceed to Phase 3: Test SLAM"
else
    echo "⚠️  Phase 2 Incomplete - Some packages missing"
    echo "See ROS2_INSTALLATION_GUIDE.md for troubleshooting"
fi
echo "=========================================="
