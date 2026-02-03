# Roomba-like Robot Specifications & Configuration

## Research-Based Design Decisions

Based on official Roomba specifications and ROS 2/Gazebo best practices, this document outlines the robot's physical parameters and configuration.

## Physical Dimensions (Roomba 105 Specs)

### Chassis
- **Dimensions**: 0.336m × 0.335m × 0.104m (33.6cm × 33.5cm × 10.4cm)
- **Mass**: 2.6kg (chassis only, total robot ~2.84kg)
- **Shape**: Rectangular box (Roomba-like circular appearance approximated)
- **Center of Mass**: 
  - X: 0.11m forward (to balance with LiDAR on top and caster wheel)
  - Y: 0m (centered)
  - Z: 0.052m (half of chassis height)

### Wheels
- **Drive Wheels**:
  - Radius: 0.033m (1.3 inches)
  - Width: 0.02m
  - Separation: 0.229m (9 inches)
  - Mass: 0.1kg each
  - Position: z=0.033m (wheel radius above ground, ensuring ground contact)
  
- **Caster Wheel**:
  - Radius: 0.015m
  - Mass: 0.03kg
  - Position: x=0.15m forward from chassis center, z=-0.037m (touches ground)

### LiDAR Sensor
- **Position**: Top of chassis (z=0.104m)
- **Mass**: 0.1kg
- **Configuration**:
  - 360 samples (1° resolution)
  - Update rate: 10Hz
  - Range: 0.12m to 3.5m
  - 360° horizontal scan

### IMU Sensor
- **Position**: Center of base_link (z=0m)
- **Mass**: 0.01kg
- **Update rate**: 100Hz
- **Noise model**: Gaussian with realistic bias and variance

## Center of Mass Calculation

The center of mass is carefully calculated to prevent backward tilt:

**Component Masses:**
- Chassis: 2.6kg at (0.11, 0, 0.052)
- LiDAR: 0.1kg at (0, 0, 0.104) - **raises COM vertically**
- Drive wheels: 0.2kg total at (0, ±0.1145, 0.033) - **pulls COM back**
- Caster wheel: 0.03kg at (0.15, 0, ~0.015) - **pulls COM forward**

**Balance Strategy:**
- Chassis COM positioned at x=0.11m forward to compensate for:
  1. LiDAR mass on top (raises and shifts COM backward)
  2. Drive wheels at rear (x=0)
  3. Caster wheel at front (x=0.15m)

This ensures the robot sits level with all wheels touching the ground.

## Gazebo Configuration

### Differential Drive Plugin
- **Plugin**: `libgazebo_ros_diff_drive.so`
- **Wheel separation**: 0.229m
- **Wheel diameter**: 0.066m
- **Max wheel torque**: 200 N⋅m
- **Max wheel acceleration**: 10.0 rad/s²
- **Odometry frame**: `odom`
- **Base frame**: `base_link`

### Physics Properties
- **Chassis friction**: μ=0.6
- **Wheel friction**: μ=0.8 (with slip coefficients)
- **Caster friction**: μ=0.01 (low friction for free rotation)

## Sensor Configuration

### LiDAR (LaserScan)
- **Topic**: `/scan`
- **Frame**: `laser`
- **Type**: `sensor_msgs/LaserScan`
- **Update rate**: 10Hz
- **Samples**: 360 (1° resolution)
- **Range**: 0.12m - 3.5m
- **Optimized for**: SLAM Toolbox compatibility

### IMU
- **Topic**: `/imu`
- **Frame**: `imu_link`
- **Type**: `sensor_msgs/Imu`
- **Update rate**: 100Hz
- **Noise model**: Realistic Gaussian noise with bias

## TF Tree Structure

```
odom
  └── base_link
      ├── chassis
      │   └── caster_wheel
      ├── left_wheel
      ├── right_wheel
      ├── laser
      └── imu_link
```

## RViz Configuration

- **Fixed Frame**: `odom` (required for SLAM)
- **LaserScan Display**: Red color, 0.03m size, flat squares
- **TF Display**: Enabled with axes and names
- **Grid Display**: Enabled for reference
- **Map Display**: Ready for SLAM output

## SLAM & Navigation Readiness

The robot is configured for:
- **SLAM Toolbox**: Compatible with `/scan` topic and `odom` frame
- **Nav2**: Ready for autonomous navigation
- **Mapping**: LiDAR configured for 360° coverage
- **Localization**: IMU + wheel odometry fusion ready

## Testing Checklist

- [x] Robot spawns in Gazebo
- [x] All wheels touch ground
- [x] Robot sits level (no backward tilt)
- [x] LiDAR publishes `/scan` topic
- [x] IMU publishes `/imu` topic
- [x] Odometry publishes `/odom` topic
- [x] TF tree is correct
- [x] Robot moves with teleop
- [x] RViz displays LiDAR scans

## References

- Roomba 105 specifications: 33.6cm × 33.5cm × 10.4cm, 2.84kg
- ROS 2 Humble URDF in Gazebo tutorial
- SLAM Toolbox documentation
- Nav2 navigation stack requirements
