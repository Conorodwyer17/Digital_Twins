# ROS2 Humble Installation Guide

## Issue Detected

The installation script failed because ROS2 Humble is not installed on your system. The packages `ros-humble-slam-toolbox` and `ros-humble-navigation2` require ROS2 Humble to be installed first.

## Solution Options

### Option 1: Install ROS2 Humble on Host System (Recommended for Native Development)

#### Step 1: Add ROS2 Repository
```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
```

#### Step 2: Install ROS2 Humble Desktop
```bash
sudo apt update
sudo apt install ros-humble-desktop -y
```

#### Step 3: Source ROS2
```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

#### Step 4: Install Dependencies
```bash
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server
```

### Option 2: Use Docker (If Project is Docker-Based)

If you're using Docker, you need to install packages inside the container:

```bash
# Build and run container
cd /home/conor/digital_twins/my_bot
docker build -t my_bot .
docker run -it --rm my_bot

# Inside container, run:
sudo apt update
sudo apt install -y \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server
```

### Option 3: Build from Source (Advanced)

If apt packages aren't available, you can build SLAM_Toolbox and Nav2 from source:

#### Install SLAM_Toolbox from Source
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/SteveMacenski/slam_toolbox.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

#### Install Nav2 from Source
```bash
cd ~/ros2_ws/src
git clone https://github.com/ros-planning/navigation2.git -b humble
cd ~/ros2_ws
rosdep install -y -r --from-paths src --ignore-src --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

## Verification

After installation, verify:
```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep slam_toolbox
ros2 pkg list | grep nav2
```

## Updated Installation Script

I'll create an updated installation script that checks for ROS2 first and provides helpful error messages.

## Quick Check: Are you using Docker?

Check if you're supposed to be in a Docker container:
```bash
cat /home/conor/digital_twins/my_bot/Dockerfile | head -5
```

If the Dockerfile shows `FROM osrf/ros:humble-desktop`, then ROS2 should be in the container, not on the host.

## Recommended Approach

**For this assignment, I recommend Option 1** (install ROS2 Humble on host) because:
- Easier to work with
- No container overhead
- Direct access to Gazebo GUI
- Simpler debugging

Let me know which approach you prefer, and I'll update the installation script accordingly!
