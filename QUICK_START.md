# Quick Start: Fix ROS2 Installation Issue

## Problem
ROS2 Humble is not installed on your host system, so the SLAM and Nav2 packages can't be installed.

## Solution Options

### Option 1: Install ROS2 Humble on Host (Recommended - 10 minutes)

This is the fastest way to get everything working:

```bash
# Add ROS2 repository
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Source ROS2 (add to ~/.bashrc for persistence)
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Now install SLAM and Nav2
cd /home/conor/digital_twins/my_bot
./PHASE2_INSTALL_SCRIPT_FIXED.sh
```

### Option 2: Use Docker (If you prefer containers)

The Dockerfile has been updated to include SLAM and Nav2. Rebuild the container:

```bash
cd /home/conor/digital_twins/my_bot
docker build -t my_bot .
docker run -it --rm \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    my_bot
```

Inside the container, ROS2 and all packages will be available.

### Option 3: Use the Fixed Installation Script

The new script (`PHASE2_INSTALL_SCRIPT_FIXED.sh`) will check for ROS2 first and give you helpful error messages:

```bash
cd /home/conor/digital_twins/my_bot
./PHASE2_INSTALL_SCRIPT_FIXED.sh
```

## Recommended: Option 1

**I recommend Option 1** because:
- ✅ Works directly with your current setup
- ✅ No container overhead
- ✅ Easier to debug
- ✅ Direct access to Gazebo GUI
- ✅ Takes ~10 minutes

## After Installation

Once ROS2 is installed and packages are installed, you can:

1. **Test SLAM:**
   ```bash
   ros2 launch my_bot launch_sim_with_slam.launch.py
   ```

2. **Drive and Map:**
   ```bash
   # In another terminal
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. **Save Map:**
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/kitchen_map
   ```

## Verification

After installing ROS2, verify it works:
```bash
source /opt/ros/humble/setup.bash
ros2 --help
ros2 pkg list | grep slam
```

If you see the ros2 command and packages, you're ready to proceed!
