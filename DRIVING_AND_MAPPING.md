# Driving and Mapping Guide

## Quick Start - Drive the Robot

### Step 1: Launch Simulation (Terminal 1)

Make sure Gazebo is running with the kitchen world:
```bash
cd workspace
source install/setup.bash
ros2 launch my_bot launch_sim.launch.py
```

You should see:
- Gazebo with kitchen/dining room
- Robot spawned in the kitchen
- LiDAR rays visible (stopping at walls)

### Step 2: Launch Teleop (Terminal 2)

Open a **new terminal** (or new container session):
```bash
cd workspace
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Important:** Make sure this terminal window has focus (click on it) before pressing keys.

### Step 3: Control the Robot

Use these keyboard keys to drive:

| Key | Action |
|-----|--------|
| `i` | Move **forward** |
| `,` | Move **backward** |
| `j` | Rotate **left** (counter-clockwise) |
| `l` | Rotate **right** (clockwise) |
| `u` | Move forward + turn left |
| `o` | Move forward + turn right |
| `m` | Move backward + turn left |
| `.` | Move backward + turn right |
| `k` | **Stop** (zero velocity) |
| `q` | Quit teleop |

**Speed Control:**
- Press keys repeatedly to increase speed
- Press `k` to stop
- Speed increases with each key press

### Step 4: Verify Movement

**In Gazebo:**
- Robot should move smoothly
- Wheels should rotate
- Robot should turn in place when using `j`/`l`

**In RViz:**
- LiDAR scan should update as robot moves
- Scan should show walls and obstacles
- Robot position should update in TF tree

## Mapping with SLAM Toolbox

### Prerequisites

Make sure you have SLAM Toolbox installed:
```bash
sudo apt install ros-humble-slam-toolbox
```

### Step 1: Launch SLAM (Terminal 3)

Open a **third terminal**:
```bash
cd workspace
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py
```

This will:
- Start SLAM Toolbox node
- Subscribe to `/scan` (LiDAR) and `/odom` (odometry)
- Publish `/map` topic

### Step 2: Configure RViz for Mapping

In RViz (if not already open):
```bash
ros2 run rviz2 rviz2 -d install/my_bot/share/my_bot/config/visualize.rviz
```

**Enable Map Display:**
1. In the "Displays" panel, find "Map"
2. Check the box to enable it
3. Verify it's subscribed to `/map` topic
4. You should see the map building as you drive

### Step 3: Drive and Map

1. **Start driving** using teleop (Terminal 2)
2. **Move slowly** around the kitchen
3. **Cover the entire area** - drive along walls, around furniture
4. **Watch the map build** in RViz

**Mapping Tips:**
- Drive slowly for better map quality
- Make sure to cover all areas
- Close loops (return to previously visited areas)
- Avoid driving too fast (can cause mapping errors)

### Step 4: Save the Map

Once you've mapped the area, save it:

```bash
# In a new terminal
cd workspace
source install/setup.bash

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/kitchen_map
```

This creates:
- `~/kitchen_map.pgm` - Map image
- `~/kitchen_map.yaml` - Map metadata

## Navigation with Nav2 (Optional)

Once you have a map, you can use Nav2 for autonomous navigation:

### Step 1: Launch Nav2

```bash
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=true \
    map:=~/kitchen_map.yaml
```

### Step 2: Set Initial Pose

In RViz:
1. Click "2D Pose Estimate" tool
2. Click on the map where the robot is
3. Drag to set robot orientation

### Step 3: Set Navigation Goal

In RViz:
1. Click "2D Nav Goal" tool
2. Click on the map where you want to go
3. Drag to set target orientation
4. Robot will autonomously navigate there!

## Troubleshooting

### Robot Not Moving

1. **Check cmd_vel topic:**
   ```bash
   ros2 topic echo /cmd_vel
   ```
   Should show velocity commands when you press keys

2. **Check teleop is running:**
   - Make sure terminal has focus
   - Check for error messages

3. **Check Gazebo plugin:**
   - Verify robot is spawned
   - Check for errors in Gazebo terminal

### LiDAR Not Updating in RViz

1. **Check scan topic:**
   ```bash
   ros2 topic hz /scan
   ```
   Should show ~30Hz

2. **Check RViz settings:**
   - Fixed Frame: `odom`
   - LaserScan enabled
   - Topic: `/scan`

### Map Not Building

1. **Check SLAM is running:**
   ```bash
   ros2 node list | grep slam
   ```

2. **Check topics:**
   ```bash
   ros2 topic list | grep -E "scan|odom|map"
   ```
   All should exist

3. **Check TF tree:**
   ```bash
   ros2 run tf2_tools view_frames.py
   ```
   Should show `odom -> base_link -> laser`

## Quick Reference

**Topics:**
- `/cmd_vel` - Velocity commands (from teleop)
- `/scan` - LiDAR data
- `/odom` - Odometry
- `/map` - SLAM map

**Frames:**
- `odom` - Odometry frame
- `base_link` - Robot base
- `laser` - LiDAR sensor

**Useful Commands:**
```bash
# View all topics
ros2 topic list

# Check topic rate
ros2 topic hz /scan

# View topic data
ros2 topic echo /scan --once

# View TF tree
ros2 run tf2_tools view_frames.py
```

---

**Ready to map!** Start with teleop, then add SLAM when ready.
