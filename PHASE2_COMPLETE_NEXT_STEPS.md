# Phase 2 Complete! ✅ Next Steps

## ✅ Installation Successful

All packages installed:
- ✅ SLAM_Toolbox
- ✅ Nav2 (30 packages)
- ✅ Nav2 Map Server

## ⚠️ Important: Rebuild Package

After adding new launch files, you **must rebuild** the package:

```bash
cd /home/conor/digital_twins/my_bot  # or /home/ros_user/workspace if using that
colcon build --symlink-install
source install/setup.bash
```

## Testing SLAM

### Option 1: Separate Launch (Recommended for First Test)

**Terminal 1: Launch Simulation**
```bash
cd /home/conor/digital_twins/my_bot  # or your workspace path
source install/setup.bash
ros2 launch my_bot launch_sim_with_rviz.launch.py
```

**Terminal 2: Launch SLAM**
```bash
cd /home/conor/digital_twins/my_bot
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
cd /home/conor/digital_twins/my_bot
source install/setup.bash
ros2 launch my_bot launch_sim_with_slam.launch.py
```

**Terminal 2: Drive Robot**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## In RViz: Enable Map Display

1. **Add Map Display:**
   - Click "Add" in Displays panel
   - Select "Map"
   - Set Topic to `/map`
   - Set Color Scheme to "costmap" or "map"

2. **Verify Topics:**
   ```bash
   ros2 topic list | grep -E "scan|odom|map"
   ```
   Should see:
   - `/scan` - LiDAR data
   - `/odom` - Odometry
   - `/map` - SLAM map (after SLAM starts)

3. **Check Map Publishing:**
   ```bash
   ros2 topic hz /map
   ```
   Should show map updates

## Mapping Tips

1. **Drive Slowly** - Better map quality
2. **Cover Entire Area** - Drive along walls, around furniture
3. **Close Loops** - Return to previously visited areas
4. **Watch Map Build** - Should see walls and obstacles appear

## Save Map

After mapping the kitchen:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/kitchen_map
```

This creates:
- `~/kitchen_map.pgm` - Map image
- `~/kitchen_map.yaml` - Map metadata

## Troubleshooting

### Launch File Not Found
- **Solution**: Rebuild package: `colcon build --symlink-install`
- **Then**: Source setup: `source install/setup.bash`

### Map Not Building
- Check SLAM is running: `ros2 node list | grep slam`
- Check topics: `ros2 topic list | grep map`
- Check TF tree: `ros2 run tf2_tools view_frames.py`

### Robot Not Moving
- Make sure teleop terminal has focus
- Check `/cmd_vel` topic: `ros2 topic echo /cmd_vel`

## Next: Phase 4 - Nav2 Configuration

Once you have a saved map, proceed to Phase 4 to configure Nav2 for autonomous navigation!
