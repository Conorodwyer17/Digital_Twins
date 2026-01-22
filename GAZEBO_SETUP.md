# Simple Robot Gazebo Simulation

This package contains a simple differential drive robot that can be simulated in Gazebo.

## Setup

1. Build the workspace:
```bash
cd /home/brian/EE5108/simple_robot_ws
colcon build
source install/setup.bash
```

2. Launch Gazebo simulation with the robot:
```bash
ros2 launch my_bot gazebo.launch.py
```

## Features

- **Robot Description**: The robot is a simple differential drive robot with:
  - Main chassis (white box)
  - Two driven wheels (blue cylinders)
  - One passive caster wheel (black sphere)

- **Gazebo Integration**:
  - Differential drive controller for wheel motion
  - Physics simulation with ODE
  - Robot spawned in empty world with ground plane and sun

## Simulation Controls

The robot can be controlled by publishing to the `/my_bot/cmd_vel` topic:

```bash
# Example: Move forward
ros2 topic pub /my_bot/cmd_vel geometry_msgs/Twist "linear: {x: 0.5, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"

# Example: Rotate
ros2 topic pub /my_bot/cmd_vel geometry_msgs/Twist "linear: {x: 0.0, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.5}"
```

## Odometry

The robot publishes odometry data on `/my_bot/odom` topic, which includes position and velocity estimates.

## Available Launch Files

- `rsp.launch.py`: Robot state publisher and joint state publisher GUI (for RViz visualization)
- `gazebo.launch.py`: Gazebo simulation with robot spawned

## Dependencies

- gazebo_ros
- gazebo_ros2_control
- robot_state_publisher
- joint_state_publisher_gui
