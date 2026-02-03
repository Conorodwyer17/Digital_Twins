# Rebuild Instructions for SLAM Launch Files

## Issue
After adding new launch files, they won't be available until you rebuild the package.

## Solution: Rebuild Package

### Step 1: Clean Build (If Needed)
If you get CMakeCache errors, clean first:
```bash
cd /home/conor/digital_twins/my_bot  # or your workspace path
rm -rf build install log
```

### Step 2: Rebuild
```bash
colcon build --symlink-install
```

### Step 3: Source Setup
```bash
source install/setup.bash
```

### Step 4: Verify Launch Files
```bash
ros2 launch my_bot slam.launch.py --help
# Should show help, not "file not found" error
```

## Quick Rebuild Command

```bash
cd /home/conor/digital_twins/my_bot
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## After Rebuild: Test SLAM

**Terminal 1:**
```bash
cd /home/conor/digital_twins/my_bot
source install/setup.bash
ros2 launch my_bot launch_sim_with_slam.launch.py
```

**Terminal 2:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Note for Different Workspace Paths

If you're working in `/home/ros_user/workspace` instead:
1. Make sure the `my_bot` package is there
2. Rebuild in that location
3. Source from that location

The launch files are in the source code, but need to be "installed" via colcon build.
