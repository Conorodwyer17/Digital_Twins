# Quick Start: Test SLAM Now! 🚀

## ✅ Phase 2 Complete - Packages Installed!

All packages are installed. Now you need to rebuild in your workspace.

## Rebuild in Your Workspace

Based on your terminal, you're working in `/home/ros_user/workspace`. Rebuild there:

```bash
cd /home/ros_user/workspace
rm -rf build install log  # Clean old build
colcon build --symlink-install
source install/setup.bash
```

## Test SLAM (Two Options)

### Option 1: Separate Launch (Easier to Debug)

**Terminal 1: Launch Simulation**
```bash
cd /home/ros_user/workspace
source install/setup.bash
ros2 launch my_bot launch_sim_with_rviz.launch.py
```

**Terminal 2: Launch SLAM**
```bash
cd /home/ros_user/workspace
source install/setup.bash
ros2 launch my_bot slam.launch.py
```

**Terminal 3: Drive Robot**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Option 2: Combined Launch (After Rebuild)

**Terminal 1: Launch Everything**
```bash
cd /home/ros_user/workspace
source install/setup.bash
ros2 launch my_bot launch_sim_with_slam.launch.py
```

**Terminal 2: Drive Robot**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## In RViz: Enable Map

1. Click "Add" → "Map"
2. Topic: `/map`
3. You should see map building as you drive!

## Verify It's Working

```bash
# Check topics
ros2 topic list | grep -E "scan|odom|map"

# Check map is publishing
ros2 topic hz /map

# Check SLAM node
ros2 node list | grep slam
```

## Save Map (After Mapping)

```bash
ros2 run nav2_map_server map_saver_cli -f ~/kitchen_map
```

## Troubleshooting

**If launch file not found:**
- Make sure you rebuilt: `colcon build --symlink-install`
- Make sure you sourced: `source install/setup.bash`
- Check you're in the right directory

**If map not building:**
- Check `/scan` topic: `ros2 topic hz /scan`
- Check `/odom` topic: `ros2 topic hz /odom`
- Check SLAM node is running: `ros2 node list | grep slam`

## Next Steps

Once SLAM is working and you've saved a map:
- Proceed to **Phase 4: Nav2 Configuration**
- See `ULTIMATE_TODO_COMPLETE.md` for Nav2 setup
