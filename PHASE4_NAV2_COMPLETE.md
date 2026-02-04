# Phase 4: Nav2 Configuration Complete! ✅

## Files Created

### 1. `nav2_params.yaml` - Nav2 Configuration
- Location: `my_bot/my_bot/my_bot/config/nav2/nav2_params.yaml`
- Configured for your Roomba robot:
  - Robot radius: 0.18m
  - Max velocity: 0.5 m/s
  - Max angular velocity: 1.0 rad/s
  - LiDAR scan topic: `/scan`
  - Odometry topic: `/odom`

### 2. `nav2.launch.py` - Standalone Nav2 Launch
- Location: `my_bot/my_bot/my_bot/launch/nav2.launch.py`
- Usage: `ros2 launch my_bot nav2.launch.py map:=~/kitchen_map.yaml`

### 3. `launch_full_nav.launch.py` - Complete Launch
- Location: `my_bot/my_bot/my_bot/launch/launch_full_nav.launch.py`
- Launches: Simulation + RViz + Nav2 (requires saved map)

## Next Steps: Complete the Assignment

### Step 1: Save Your Map (If Not Already Done)

While SLAM is running and you have a good map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/kitchen_map
```

This creates:
- `~/kitchen_map.pgm` - Map image
- `~/kitchen_map.yaml` - Map metadata

### Step 2: Rebuild Package

```bash
cd /home/ros_user/workspace  # or your workspace path
colcon build --symlink-install
source install/setup.bash
```

### Step 3: Test Autonomous Navigation

**Option A: Separate Launch (Recommended for First Test)**

**Terminal 1: Launch Simulation**
```bash
cd /home/ros_user/workspace
source install/setup.bash
ros2 launch my_bot launch_sim_with_rviz.launch.py
```

**Terminal 2: Launch Nav2**
```bash
cd /home/ros_user/workspace
source install/setup.bash
ros2 launch my_bot nav2.launch.py map:=~/kitchen_map.yaml
```

**Terminal 3: In RViz - Set Initial Pose**
1. Click "2D Pose Estimate" tool (toolbar)
2. Click on the map where the robot is located
3. Drag to set robot orientation

**Terminal 4: In RViz - Set Navigation Goal**
1. Click "2D Nav Goal" tool (toolbar)
2. Click on the map where you want the robot to go
3. Drag to set target orientation
4. **Robot should autonomously navigate there!** 🎉

**Option B: Combined Launch (After Rebuild)**

```bash
cd /home/ros_user/workspace
source install/setup.bash
ros2 launch my_bot launch_full_nav.launch.py map:=~/kitchen_map.yaml
```

Then in RViz:
1. Set initial pose (2D Pose Estimate)
2. Set navigation goal (2D Nav Goal)
3. Watch robot navigate autonomously!

## Verify Nav2 is Working

```bash
# Check Nav2 nodes
ros2 node list | grep nav2

# Check topics
ros2 topic list | grep -E "goal|plan|path|cmd_vel"

# Check if robot is moving
ros2 topic echo /cmd_vel
```

## Assignment Completion Checklist

- [x] ✅ Gazebo environment
- [x] ✅ Robot design (Roomba dimensions)
- [x] ✅ Physical properties configured
- [x] ✅ LiDAR sensor
- [x] ✅ IMU sensor
- [x] ✅ SLAM mapping working
- [ ] **TODO**: Save map (if not done)
- [ ] **TODO**: Rebuild package
- [ ] **TODO**: Test Nav2 autonomous navigation
- [ ] **TODO**: Demo: Navigate from start to goal

## Demo Requirements

For the assignment demo, you need to:
1. **Launch simulation** with Nav2
2. **Set initial pose** in RViz (where robot starts)
3. **Set navigation goal** in RViz (where robot should go)
4. **Show robot navigating autonomously** from start to goal
5. **Robot should avoid obstacles** and follow a planned path

## Troubleshooting

### Nav2 Not Starting
- Check map file exists: `ls ~/kitchen_map.yaml`
- Check Nav2 params file: `ls install/my_bot/share/my_bot/config/nav2/nav2_params.yaml`
- Rebuild package: `colcon build --symlink-install`

### Robot Not Moving
- Check initial pose is set correctly
- Check `/cmd_vel` topic: `ros2 topic echo /cmd_vel`
- Check Nav2 nodes: `ros2 node list | grep nav2`

### Path Planning Fails
- Check map is loaded: Look for map in RViz
- Check costmaps: Should see red/yellow/green in RViz
- Check LiDAR is working: `ros2 topic hz /scan`

## You're Almost Done! 🚀

After testing Nav2 navigation, you'll have completed:
- ✅ All required components
- ✅ SLAM mapping
- ✅ Autonomous navigation

**Optional**: Add visualization mesh (STL file) for extra credit!
