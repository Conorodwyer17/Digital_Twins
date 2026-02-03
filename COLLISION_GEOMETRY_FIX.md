# Kitchen Dining Model - Collision Geometry Fix

## Problem Summary

The LiDAR sensor was emitting rays (visible as blue lines in Gazebo), but the rays were passing **through walls** without detection. This meant:
- No range data was being published to `/scan` topic
- RViz showed no LiDAR scan visualization
- SLAM and mapping were impossible

## Root Cause

The `kitchen_dining` model from `models.gazebosim.org` only had **visual geometry** (for rendering) but **no collision geometry** (for physics/sensor detection). Without collision geometry:
- LiDAR rays pass through walls
- Robots can pass through walls
- No physics interactions occur

## Solution Applied

### 1. Collision Geometry Added

All three SDF files have been updated with collision geometry:
- `model.sdf` (SDF version 1.5)
- `model-1_4.sdf` (SDF version 1.4)
- `model-1_3.sdf` (SDF version 1.3)

**Collision geometry added:**
```xml
<collision name="collision">
  <geometry>
    <mesh>
      <uri>model://kitchen_dining/meshes/kitchen_dining.dae</uri>
      <scale>1 1 1</scale>
    </mesh>
  </geometry>
</collision>
```

This uses the same mesh file as the visual geometry, ensuring perfect alignment.

### 2. Launch File Updated

The launch file now uses `kitchen_dining_world.world` instead of `empty_world.world`:
- **File**: `launch/launch_sim.launch.py`
- **Change**: Switched from `empty_world.world` to `kitchen_dining_world.world`

### 3. Model Path Configuration

The `GAZEBO_MODEL_PATH` is automatically set to include the local model:
- **Path**: `workspace/install/my_bot/share/my_bot/worlds/model`
- This ensures Gazebo finds the **local model with collision geometry** instead of downloading from the online repository

## File Locations

```
my_bot/
├── worlds/
│   ├── kitchen_dining_world.world    # World file with kitchen model
│   └── model/
│       └── kitchen_dining/
│           ├── model.config          # Model configuration
│           ├── model.sdf             # SDF v1.5 (with collision)
│           ├── model-1_4.sdf          # SDF v1.4 (with collision)
│           ├── model-1_3.sdf          # SDF v1.3 (with collision)
│           ├── meshes/
│           │   └── kitchen_dining.dae # Mesh file (used for both visual and collision)
│           └── materials/
```

## Verification Steps

### 1. Rebuild Workspace
```bash
cd workspace
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Simulation
```bash
ros2 launch my_bot launch_sim.launch.py
```

### 3. Check in Gazebo
- **LiDAR rays should STOP at walls** (not pass through)
- Blue rays should terminate when hitting walls/objects
- You should see the rays bouncing off surfaces

### 4. Check in RViz
```bash
ros2 run rviz2 rviz2 -d install/my_bot/share/my_bot/config/visualize.rviz
```

**Expected results:**
- Red points/dots showing LiDAR scan data
- Walls should appear as clusters of points
- Scan should update at 30Hz
- No more "empty" scans

### 5. Verify Topic is Publishing
```bash
# Check topic exists
ros2 topic list | grep scan

# Check topic is publishing data
ros2 topic hz /scan

# View scan data
ros2 topic echo /scan --once
```

You should see:
- Topic `/scan` exists
- Publishing rate ~30Hz
- Range data with valid values (not all zeros or max range)

## Important Notes

### If You Manually Inserted the Kitchen Model

If you manually inserted the "Kitchen and Dining" model from Gazebo's model browser:
1. **Remove the manually inserted model** (it's from the online repository without collision)
2. **Restart Gazebo** with the launch file (which loads the local model with collision)

### Model Path Priority

Gazebo searches for models in this order:
1. `GAZEBO_MODEL_PATH` environment variable (set by launch file)
2. `~/.gazebo/models/`
3. Online repositories (`http://models.gazebosim.org/`)

The launch file ensures the local model (with collision) is found first.

### Performance Note

Adding collision geometry to complex meshes can slow down simulation. If performance is an issue:
- Consider using simplified collision meshes
- Run Gazebo in headless mode: `gazebo --headless`

## Troubleshooting

### LiDAR Still Not Detecting Walls

1. **Check model is loaded from local path:**
   ```bash
   echo $GAZEBO_MODEL_PATH
   ```
   Should include: `.../install/my_bot/share/my_bot/worlds/model`

2. **Verify collision geometry in SDF:**
   ```bash
   cat install/my_bot/share/my_bot/worlds/model/kitchen_dining/model.sdf | grep -A 5 collision
   ```
   Should show the collision mesh definition.

3. **Check world file is using kitchen model:**
   ```bash
   cat install/my_bot/share/my_bot/worlds/kitchen_dining_world.world
   ```
   Should include: `<uri>model://kitchen_dining</uri>`

4. **Restart Gazebo completely:**
   - Close Gazebo
   - Kill any remaining processes: `pkill gzserver; pkill gzclient`
   - Relaunch with the launch file

### Scan Topic Not Publishing

1. **Check sensor is enabled:**
   - In Gazebo, verify LiDAR sensor is active
   - Check for errors in terminal output

2. **Verify sensor configuration:**
   - Check `sensors.xacro` has correct plugin configuration
   - Ensure `frame_name` is `laser`

3. **Check TF tree:**
   ```bash
   ros2 run tf2_tools view_frames.py
   ```
   Should show `laser` frame connected to `base_link`

## Expected Results After Fix

✅ **Gazebo:**
- LiDAR rays stop at walls (blue lines terminate)
- Rays bounce off surfaces correctly
- No rays passing through geometry

✅ **RViz:**
- Red points showing LiDAR scan
- Walls visible as point clusters
- Continuous scan updates

✅ **Topics:**
- `/scan` topic publishing at ~30Hz
- Valid range data (0.12m to 8.0m)
- 360 samples per scan

✅ **SLAM Ready:**
- LiDAR data suitable for SLAM Toolbox
- Walls and obstacles properly detected
- Ready for autonomous navigation

---

**Last Updated**: After collision geometry fix
**Status**: Ready for testing
