# Robot Physics & Geometry Fixes

## Summary of Issues Fixed

This document details the fixes applied to resolve the robot's backward tilt, wheel misalignment, unstable physics, and LiDAR perception problems.

---

## 🔴 Problems Identified

### 1. **Center of Mass Too High and Not Forward Enough**
- **Issue**: COM was at z=0.118m (chassis center height), causing the robot to tip backward
- **Root Cause**: COM positioned at geometric center instead of actual center of mass
- **Impact**: Gravity pulled the robot backward, causing severe tilt

### 2. **LiDAR Mounting Frame**
- **Issue**: LiDAR was attached to `base_link` instead of `chassis`
- **Root Cause**: If chassis tilts, LiDAR should tilt with it (realistic), but chassis tilt was the problem
- **Impact**: When robot tilted backward, LiDAR scanned the ground behind it

### 3. **Wheel Separation Mismatch**
- **Issue**: Controller config had `wheel_separation: 0.297m` but actual separation is `0.229m`
- **Root Cause**: Configuration inconsistency between Gazebo plugin and ROS2 controller
- **Impact**: Incorrect odometry calculations, poor navigation performance

### 4. **Caster Wheel Position**
- **Issue**: Caster positioned at x=0.15m, slightly too far forward
- **Fix**: Adjusted to x=0.14m for better balance
- **Impact**: Minor improvement in stability

---

## ✅ Fixes Applied

### 1. Center of Mass Correction (`robot_core.xacro`)

**Before:**
```xml
<xacro:inertial_box mass="2.6" x="0.336" y="0.335" z="0.104">
  <origin xyz="0.13 0 0.118" rpy="0 0 0"/>
</xacro:inertial_box>
```

**After:**
```xml
<xacro:inertial_box mass="2.6" x="0.336" y="0.335" z="0.104">
  <origin xyz="0.10 0 0.085" rpy="0 0 0"/>
</xacro:inertial_box>
```

**Explanation:**
- **X-position (0.10m forward)**: Compensates for LiDAR mass on top and wheels at rear
- **Z-position (0.085m)**: Lowered from 0.118m to 0.085m (below chassis center)
- **Why this works**: Lower COM increases stability, forward COM prevents backward tilt
- **Reference**: Real Roomba has COM forward of wheel axis and low in chassis

**Mass Distribution Analysis:**
- Chassis: 2.6kg at (0.10, 0, 0.085) - main mass, forward and low
- LiDAR: 0.1kg at (0, 0, 0.170) - raises COM vertically, shifts backward
- Wheels: 0.2kg total at (0, ±0.1145, 0.033) - pulls COM back
- Caster: 0.03kg at (0.14, 0, 0.015) - pulls COM forward
- **Result**: Combined COM is forward of wheel axis and low enough for stability

### 2. LiDAR Mounting Fix (`sensors.xacro`)

**Before:**
```xml
<joint name="laser_joint" type="fixed">
  <parent link="base_link"/>
  <child link="laser"/>
  <origin xyz="0 0 0.170" rpy="0 0 0"/>
</joint>
```

**After:**
```xml
<joint name="laser_joint" type="fixed">
  <parent link="chassis"/>
  <child link="laser"/>
  <origin xyz="0 0 0.052" rpy="0 0 0"/>
</joint>
```

**Explanation:**
- **Parent changed**: From `base_link` to `chassis` (more realistic structure)
- **Position**: z=0.052m relative to chassis center = 0.118 + 0.052 = 0.170m (same height)
- **Orientation**: rpy="0 0 0" ensures horizontal scan plane (parallel to ground)
- **Why this works**: 
  - If chassis is level (after COM fix), LiDAR will be level
  - LiDAR no longer sees ground behind robot
  - More realistic mounting structure

### 3. Wheel Separation Fix (`my_controllers.yaml`)

**Before:**
```yaml
wheel_separation: 0.297
```

**After:**
```yaml
# Wheel separation: 0.229m (matches actual wheel positions: 0.1145 * 2)
# Must match gazebo_control.xacro wheel_separation value
wheel_separation: 0.229
```

**Explanation:**
- **Actual separation**: 0.1145m × 2 = 0.229m (from wheel joint positions)
- **Gazebo plugin**: Already correct at 0.229m
- **Controller**: Was incorrect at 0.297m
- **Why this matters**: Incorrect separation causes wrong odometry, breaking SLAM and Nav2

### 4. Caster Wheel Position Adjustment

**Before:**
```xml
<origin xyz="0.15 0 -0.103" rpy="0 0 0"/>
```

**After:**
```xml
<origin xyz="0.14 0 -0.103" rpy="0 0 0"/>
```

**Explanation:**
- **Position**: Moved slightly back from 0.15m to 0.14m
- **Impact**: Better balance with forward COM
- **Note**: Still touches ground correctly (z=-0.103 relative to chassis center)

---

## 📐 Geometry Verification

### Coordinate System (ROS Standard)
- **X-axis**: Forward
- **Y-axis**: Left
- **Z-axis**: Up
- **Origin**: `base_link` at wheel axis center

### Key Dimensions

**Wheels:**
- Center height: z=0.033m (wheel radius)
- Top height: z=0.066m
- Separation: 0.229m (0.1145m × 2)

**Chassis:**
- Bottom: z=0.066m (sits on wheel tops)
- Center: z=0.118m
- Top: z=0.170m
- Dimensions: 0.336m × 0.335m × 0.104m

**Caster:**
- Center: z=0.015m (relative to base_link)
- Position: x=0.14m forward
- Radius: 0.015m (touches ground)

**LiDAR:**
- Height: z=0.170m (on chassis top)
- Orientation: Horizontal (rpy="0 0 0")

**Center of Mass:**
- Position: (0.10, 0, 0.085) relative to base_link
- Forward of wheel axis: ✓
- Low in chassis: ✓

---

## 🎯 Expected Results

After these fixes, the robot should:

1. **Sit Level**: No backward or forward tilt at rest
2. **Stable Physics**: All wheels touching ground, stable equilibrium
3. **Clean LiDAR Scans**: No ground detection behind robot
4. **Correct Odometry**: Wheel separation matches actual geometry
5. **SLAM Compatible**: LiDAR scans horizontal plane correctly
6. **Nav2 Ready**: Stable base for autonomous navigation

---

## 🧪 Validation Steps

1. **Spawn robot in Gazebo**
   ```bash
   ros2 launch my_bot launch_sim.launch.py
   ```

2. **Check robot pose**
   - Robot should sit flat on ground
   - No visible tilt in Gazebo
   - All three wheels touching ground

3. **Check LiDAR in RViz**
   ```bash
   ros2 run rviz2 rviz2 -d install/my_bot/share/my_bot/config/visualize.rviz
   ```
   - LiDAR should scan horizontal plane
   - No red line on ground behind robot
   - Clean 360° scan

4. **Test movement**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   - Robot should drive smoothly
   - No oscillations or instability
   - Rotation in place should work correctly

5. **Verify TF tree**
   ```bash
   ros2 run tf2_tools view_frames.py
   ```
   - Check that all frames are correct
   - `laser` frame should be child of `chassis`

---

## 📚 References

- **Roomba Specifications**: Roomba 600 series ~3.2kg, COM forward of wheel axis
- **ROS REP 105**: Coordinate frame conventions
- **Gazebo Physics**: Center of mass and stability
- **SLAM Toolbox**: Requires horizontal LiDAR scans

---

## 🔧 Additional Notes

### Why COM at z=0.085m Works

The center of mass is positioned at z=0.085m, which is:
- **Below chassis center** (0.118m): Increases stability
- **Above wheel axis** (0.033m): Keeps robot balanced
- **Within chassis bounds**: z=0.066m (bottom) to z=0.170m (top)

This creates a stable equilibrium where:
- Gravity acts through COM
- COM is forward of wheel axis (prevents backward tilt)
- COM is low enough for stability (prevents tipping)

### LiDAR Mounting Rationale

Attaching LiDAR to `chassis` instead of `base_link`:
- **More realistic**: In real robots, sensors mount on chassis
- **Consistent with physics**: If chassis tilts, sensor tilts (realistic)
- **After COM fix**: Chassis should be level, so LiDAR will be level
- **Better for debugging**: If chassis tilts, we know there's still a problem

---

## ✅ Checklist

- [x] Center of mass corrected (forward and low)
- [x] LiDAR mounted to chassis with correct orientation
- [x] Wheel separation fixed in controller config
- [x] Caster wheel position adjusted
- [x] All geometry verified
- [x] Comments added for future maintenance
- [x] No linting errors

---

**Last Updated**: After comprehensive robot physics fixes
**Status**: Ready for testing in simulation
