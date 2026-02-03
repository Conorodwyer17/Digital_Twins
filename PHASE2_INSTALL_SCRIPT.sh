#!/bin/bash
# Phase 2: Install Dependencies for SLAM and Nav2
# Run this script to install all required packages

echo "=========================================="
echo "Phase 2: Installing Dependencies"
echo "=========================================="
echo ""

# Update package list
echo "Step 1: Updating package list..."
sudo apt update

# Install SLAM_Toolbox
echo ""
echo "Step 2: Installing SLAM_Toolbox..."
sudo apt install -y ros-humble-slam-toolbox

# Install Nav2 Stack
echo ""
echo "Step 3: Installing Nav2 Stack..."
sudo apt install -y ros-humble-navigation2 ros-humble-nav2-bringup

# Install Map Server
echo ""
echo "Step 4: Installing Nav2 Map Server..."
sudo apt install -y ros-humble-nav2-map-server

# Verify installations
echo ""
echo "=========================================="
echo "Verifying Installations..."
echo "=========================================="

source /opt/ros/humble/setup.bash

echo ""
echo "Checking SLAM_Toolbox:"
ros2 pkg list | grep slam_toolbox && echo "✅ SLAM_Toolbox installed" || echo "❌ SLAM_Toolbox NOT found"

echo ""
echo "Checking Nav2 packages:"
nav2_count=$(ros2 pkg list | grep nav2 | wc -l)
echo "Found $nav2_count Nav2 packages"
if [ $nav2_count -gt 0 ]; then
    echo "✅ Nav2 installed"
    ros2 pkg list | grep nav2 | head -5
else
    echo "❌ Nav2 NOT found"
fi

echo ""
echo "Checking Nav2 Map Server:"
ros2 pkg list | grep nav2_map_server && echo "✅ Nav2 Map Server installed" || echo "❌ Nav2 Map Server NOT found"

echo ""
echo "=========================================="
echo "Phase 2 Complete!"
echo "=========================================="
echo ""
echo "If all packages are installed, proceed to Phase 3: Create SLAM Launch File"
echo "See ULTIMATE_TODO_COMPLETE.md for next steps"
