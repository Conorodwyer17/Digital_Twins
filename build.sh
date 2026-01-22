#!/bin/bash
set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Building ROS2 workspace...${NC}"
cd /home/ros_user/workspace
colcon build --symlink-install

echo -e "${GREEN}Build complete!${NC}"
echo -e "${BLUE}To use the workspace, run:${NC}"
echo "source install/setup.bash"
echo ""
echo -e "${BLUE}To launch the robot:${NC}"
echo "ros2 launch my_bot rsp.launch.py"
echo ""
echo -e "${BLUE}In another terminal, to visualize:${NC}"
echo "rviz2"
