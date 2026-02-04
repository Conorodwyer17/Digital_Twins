# Launch Nav2 Now! 🚀

## ✅ Current Status

- ✅ Map saved: `/home/ros_user/kitchen_map.yaml`
- ✅ Package built successfully
- ✅ Simulation running (Gazebo + RViz)
- ✅ Robot spawned and working

## Next Step: Launch Nav2

**Open a NEW terminal** (Terminal 2):

```bash
# Enter Docker container
docker compose exec ros2_humble bash

# Navigate to workspace
cd /home/ros_user/workspace

# Source workspace
source install/setup.bash

# Launch Nav2
ros2 launch my_bot nav2.launch.py map:=~/kitchen_map.yaml
```

## In RViz: Set Up Navigation

After Nav2 launches (wait ~10 seconds for it to fully start):

### 1. Set Initial Pose (Where Robot Is)

1. Click **"2D Pose Estimate"** tool in RViz toolbar (looks like a crosshair)
2. Click on the **map** where your robot is currently located
3. **Drag** to set the robot's orientation (which way it's facing)

**You should see**: Robot's position updates in RViz

### 2. Set Navigation Goal (Where Robot Should Go)

1. Click **"2D Nav Goal"** tool in RViz toolbar (looks like a navigation flag)
2. Click on the **map** where you want the robot to navigate
3. **Drag** to set the target orientation

**You should see**:
- Green/yellow path appears (planned route)
- Robot starts moving autonomously! 🎉
- Robot follows the path and avoids obstacles

## Verify It's Working

**Check Nav2 nodes:**
```bash
ros2 node list | grep nav2
# Should see: nav2_amcl, nav2_bt_navigator, nav2_controller_server, etc.
```

**Check robot is moving:**
```bash
ros2 topic echo /cmd_vel
# Should show velocity commands being published
```

## Troubleshooting

### Nav2 Not Starting
- Make sure you're in the workspace: `cd /home/ros_user/workspace`
- Make sure you sourced: `source install/setup.bash`
- Check map file exists: `ls ~/kitchen_map.yaml`

### Robot Not Moving After Setting Goal
- **Check initial pose is set** - This is critical!
- Check Nav2 nodes are running: `ros2 node list | grep nav2`
- Check `/cmd_vel` topic: `ros2 topic echo /cmd_vel`
- Look for errors in Terminal 2 (Nav2 launch)

### Path Not Planning
- Check map is loaded in RViz (should see map display)
- Check costmaps are visible (enable in RViz displays)
- Check LiDAR is working: `ros2 topic hz /scan`

## Success Indicators

✅ **Nav2 is working when you see:**
- Green path line from robot to goal
- Robot moving autonomously
- Robot following the planned path
- Robot avoiding obstacles
- Robot reaching the goal

## Demo Ready! 🎬

Once the robot navigates autonomously, you're **100% complete**!

For your assignment demo:
1. Show the simulation running
2. Show the map
3. Set initial pose
4. Set navigation goal
5. Show robot navigating autonomously
6. Explain your robot design

**You've got this!** 🚀
