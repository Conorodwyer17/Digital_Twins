# Phase 2: Install Dependencies - Status

## ✅ Push to GitHub Complete!

Successfully pushed all Phase 1 changes to: https://github.com/Conorodwyer17/Digital_Twins.git

## Phase 2: Install Dependencies

### Task 2.1: Install SLAM_Toolbox
- [ ] Run: `sudo apt install ros-humble-slam-toolbox`
- [ ] Verify: `ros2 pkg list | grep slam_toolbox`

### Task 2.2: Install Nav2 Stack
- [ ] Run: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
- [ ] Verify: `ros2 pkg list | grep nav2` (should show multiple packages)

### Task 2.3: Install Map Server
- [ ] Run: `sudo apt install ros-humble-nav2-map-server`
- [ ] Verify: `ros2 pkg list | grep nav2_map_server`

## Quick Install (All at Once)

**Option 1: Use the install script**
```bash
cd /home/conor/digital_twins/my_bot
./PHASE2_INSTALL_SCRIPT.sh
```

**Option 2: Manual install**
```bash
sudo apt update
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-map-server
```

## Verification

After installation, verify:
```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep slam_toolbox
ros2 pkg list | grep nav2
```

**Expected Output:**
- `slam_toolbox` package should appear
- Multiple `nav2_*` packages should appear (nav2_bringup, nav2_core, nav2_map_server, etc.)

## Next Steps

Once Phase 2 is complete:
1. Proceed to **Phase 3: Create SLAM Launch File**
2. Follow `ULTIMATE_TODO_COMPLETE.md` for detailed instructions

## Notes

- These installations won't break existing code
- They just add new ROS2 packages to your system
- Your robot code remains unchanged
- Installation takes ~5-10 minutes depending on internet speed
