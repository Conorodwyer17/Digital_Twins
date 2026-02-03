# Ultimate To-Do List: Complete Assignment (Build on Existing)

## 🎯 Goal: Complete SLAM + Nav2 Integration Without Breaking Anything

**Strategy**: Add new features incrementally, test at each step, keep existing functionality intact.

---

## Phase 1: Preparation & Verification ✅ (Do First)

### Task 1.1: Verify Current Setup Works
- [ ] Launch simulation: `ros2 launch my_bot launch_sim_with_rviz.launch.py`
- [ ] Verify robot spawns correctly in Gazebo
- [ ] Verify robot is horizontal (not tilting)
- [ ] Verify all 3 wheels touching ground
- [ ] Test keyboard teleop (I/K/L/J keys work)
- [ ] Verify LiDAR scanning in RViz (blue rays visible)
- [ ] Check topics: `ros2 topic list | grep -E "scan|odom|cmd_vel"`
- [ ] **STOP if anything is broken** - fix before proceeding

**Expected Output:**
- Robot visible in Gazebo, balanced
- `/scan` topic publishing at ~30Hz
- `/odom` topic publishing
- `/cmd_vel` topic exists
- Keyboard control works

---

## Phase 2: Install Dependencies 📦

### Task 2.1: Install SLAM_Toolbox
```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

- [ ] Verify installation: `ros2 pkg list | grep slam_toolbox`
- [ ] Should see: `slam_toolbox`

### Task 2.2: Install Nav2 Stack
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

- [ ] Verify installation: `ros2 pkg list | grep nav2`
- [ ] Should see multiple nav2 packages

### Task 2.3: Install Map Server (for saving maps)
```bash
sudo apt install ros-humble-nav2-map-server
```

- [ ] Verify: `ros2 pkg list | grep nav2_map_server`

**Note**: These installations won't break existing code - they just add new packages.

---

## Phase 3: Add SLAM Integration (Incremental) 🗺️

### Task 3.1: Create SLAM Launch File (New File - Won't Break Existing)
- [ ] Create: `my_bot/my_bot/my_bot/launch/slam.launch.py`
- [ ] Include SLAM_Toolbox online_async_launch.py
- [ ] Configure for your robot (scan topic, odom frame)
- [ ] Set use_sim_time:=true

**Template to use:**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'max_laser_range': 8.0,
            'resolution': 0.05,
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        slam_toolbox,
    ])
```

- [ ] Test: Launch simulation + SLAM separately
- [ ] Verify `/map` topic appears: `ros2 topic list | grep map`
- [ ] **Don't modify existing launch files yet**

### Task 3.2: Test SLAM Mapping (Manual Test)
- [ ] Terminal 1: `ros2 launch my_bot launch_sim_with_rviz.launch.py`
- [ ] Terminal 2: `ros2 launch my_bot slam.launch.py`
- [ ] Terminal 3: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`
- [ ] Drive robot around kitchen slowly
- [ ] Check RViz - add Map display, subscribe to `/map`
- [ ] Verify map builds as you drive
- [ ] **If map doesn't build, debug before proceeding**

### Task 3.3: Create Combined Launch File (Optional - Keep Original)
- [ ] Create: `my_bot/my_bot/my_bot/launch/launch_sim_with_slam.launch.py`
- [ ] Include existing `launch_sim_with_rviz.launch.py` functionality
- [ ] Add SLAM launch as include
- [ ] **Keep original `launch_sim_with_rviz.launch.py` unchanged**

**Why**: This way you can still use the original launch file if needed.

### Task 3.4: Save Map Test
- [ ] Drive robot to map entire kitchen
- [ ] Save map: `ros2 run nav2_map_server map_saver_cli -f ~/kitchen_map`
- [ ] Verify files created: `~/kitchen_map.pgm` and `~/kitchen_map.yaml`
- [ ] Check map looks correct (walls, obstacles visible)

---

## Phase 4: Add Nav2 Integration (Incremental) 🧭

### Task 4.1: Create Nav2 Config Directory
- [ ] Create: `my_bot/my_bot/my_bot/config/nav2/`
- [ ] This is a new directory - won't affect existing configs

### Task 4.2: Create Nav2 Parameters File
- [ ] Create: `my_bot/my_bot/my_bot/config/nav2/nav2_params.yaml`
- [ ] Configure for your robot:
  - Robot radius: ~0.18m (chassis radius)
  - Robot base frame: `base_link`
  - Global planner: NavFn or Smac
  - Local planner: DWB or TEB
  - Costmap parameters

**Key Parameters:**
```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    global_frame_id: "map"

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "/odom"
    default_nav_to_pose_bt_xml: "navigate_to_pose_w_replanning_and_recovery.xml"
    default_nav_through_poses_bt_xml: "navigate_through_poses_w_replanning_and_recovery.xml"

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    progress_checker_plugin: "progress_checker"
    goal_checker_plugins: ["general_goal_checker"]
    controller_plugins: ["FollowPath"]

    # Robot parameters
    base_frame_id: "base_link"
    odom_topic: "/odom"
    cmd_vel_topic: "/cmd_vel"
    
    # Robot dimensions
    footprint: "[[-0.18, -0.18], [-0.18, 0.18], [0.18, 0.18], [0.18, -0.18]]"
    robot_radius: 0.18
    max_vel_x: 0.5
    min_vel_x: 0.0
    max_vel_theta: 1.0
    min_vel_theta: -1.0
    acc_lim_x: 2.5
    acc_lim_theta: 3.2

planner_server:
  ros__parameters:
    use_sim_time: true
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: true
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      robot_radius: 0.18
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: "map"
      robot_base_frame: "base_link"
      use_sim_time: true
      robot_radius: 0.18
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: true
          marking: true
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

- [ ] Test parameters file loads: `ros2 param list` (after launching Nav2)

### Task 4.3: Create Nav2 Launch File (New File)
- [ ] Create: `my_bot/my_bot/my_bot/launch/nav2.launch.py`
- [ ] Include Nav2 bringup with your config
- [ ] Set map file path parameter
- [ ] Configure for simulation (use_sim_time)

**Template:**
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default=os.path.expanduser('~/kitchen_map.yaml'))
    
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('my_bot'),
        'config',
        'nav2',
        'nav2_params.yaml'
    ])
    
    nav2_bringup = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        name='nav2_bringup',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file},
            nav2_params_file
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/kitchen_map.yaml')),
        nav2_bringup,
    ])
```

### Task 4.4: Test Nav2 (Manual Test - Don't Break Existing)
- [ ] Terminal 1: `ros2 launch my_bot launch_sim_with_rviz.launch.py`
- [ ] Terminal 2: `ros2 launch my_bot nav2.launch.py map:=~/kitchen_map.yaml`
- [ ] Terminal 3: Open RViz
- [ ] In RViz: Set 2D Pose Estimate (click where robot is)
- [ ] In RViz: Set 2D Nav Goal (click where you want to go)
- [ ] Verify robot plans path and starts moving
- [ ] **If Nav2 doesn't work, debug before proceeding**

### Task 4.5: Create Master Launch File (Optional - Keep Originals)
- [ ] Create: `my_bot/my_bot/my_bot/launch/launch_full_nav.launch.py`
- [ ] Include: simulation + SLAM + Nav2
- [ ] Make it easy to launch everything together
- [ ] **Keep all original launch files unchanged**

---

## Phase 5: Update Package Dependencies (Safe) 📝

### Task 5.1: Add Dependencies to package.xml
- [ ] Open: `my_bot/my_bot/my_bot/package.xml`
- [ ] Add (in `<depend>` section):
  ```xml
  <depend>slam_toolbox</depend>
  <depend>nav2_bringup</depend>
  <depend>nav2_map_server</depend>
  ```
- [ ] **This won't break existing code** - just declares dependencies

### Task 5.2: Rebuild Package
```bash
cd /home/conor/digital_twins/my_bot
colcon build --symlink-install
source install/setup.bash
```

- [ ] Verify build succeeds
- [ ] Test original launch file still works
- [ ] **If build fails, fix before proceeding**

---

## Phase 6: Testing & Validation ✅

### Task 6.1: Test Complete Workflow
- [ ] **Step 1**: Launch simulation only
  ```bash
  ros2 launch my_bot launch_sim_with_rviz.launch.py
  ```
  - Verify robot works, teleop works

- [ ] **Step 2**: Add SLAM
  ```bash
  # Terminal 2
  ros2 launch my_bot slam.launch.py
  ```
  - Verify map builds

- [ ] **Step 3**: Save map
  ```bash
  ros2 run nav2_map_server map_saver_cli -f ~/kitchen_map
  ```

- [ ] **Step 4**: Launch Nav2
  ```bash
  # Terminal 3
  ros2 launch my_bot nav2.launch.py map:=~/kitchen_map.yaml
  ```

- [ ] **Step 5**: Test Navigation
  - Set initial pose in RViz
  - Set navigation goal
  - Verify robot navigates autonomously

### Task 6.2: Verify Nothing Broke
- [ ] Original launch file still works: `launch_sim_with_rviz.launch.py`
- [ ] Keyboard teleop still works
- [ ] Robot still balanced
- [ ] LiDAR still scanning
- [ ] All original topics still publishing

---

## Phase 7: Optional Enhancements (Extra Credit) 🌟

### Task 7.1: Add Visualization Mesh (STL File)
- [ ] Find or create Roomba STL file
- [ ] Place in: `my_bot/my_bot/my_bot/meshes/roomba.stl`
- [ ] Update `robot_core.xacro` to include mesh:
  ```xml
  <visual>
    <origin xyz="0 0 0.085" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://my_bot/meshes/roomba.stl" scale="1 1 1"/>
    </geometry>
  </visual>
  ```
- [ ] Keep collision geometry as box (for physics)
- [ ] Test visualization in Gazebo

### Task 7.2: Fine-tune Nav2 Parameters
- [ ] Adjust robot radius if needed
- [ ] Tune planner parameters for smoother paths
- [ ] Adjust costmap inflation for better obstacle avoidance
- [ ] Test with different goal locations

### Task 7.3: Add Documentation
- [ ] Update README.md with SLAM/Nav2 instructions
- [ ] Document launch file options
- [ ] Add troubleshooting section

---

## Phase 8: Final Demo Preparation 🎬

### Task 8.1: Prepare Demo Script
- [ ] Create step-by-step demo procedure
- [ ] Test demo flow multiple times
- [ ] Prepare backup plan if something fails

### Task 8.2: Create Submission Package
- [ ] Zip entire `my_bot` directory
- [ ] Include all launch files
- [ ] Include config files
- [ ] Include saved map files
- [ ] Include README with instructions

---

## 🚨 Safety Checklist (Before Each Phase)

Before starting each phase, verify:
- [ ] Previous phase completed successfully
- [ ] All tests passed
- [ ] Original functionality still works
- [ ] No errors in terminal
- [ ] Robot still balanced and working

---

## 📋 Quick Reference: Launch Commands

### Original (Keep Working)
```bash
# Basic simulation
ros2 launch my_bot launch_sim_with_rviz.launch.py

# With teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### New (Add Incrementally)
```bash
# SLAM only
ros2 launch my_bot slam.launch.py

# Nav2 only (requires saved map)
ros2 launch my_bot nav2.launch.py map:=~/kitchen_map.yaml

# Full stack (after everything works)
ros2 launch my_bot launch_full_nav.launch.py
```

---

## 🎯 Success Criteria

Assignment is complete when:
- [x] Robot balanced and working ✅ (Already done)
- [ ] SLAM generates map successfully
- [ ] Map saved and can be loaded
- [ ] Nav2 launches without errors
- [ ] Robot navigates autonomously from start to goal
- [ ] Original functionality still works
- [ ] Demo can be performed reliably

---

## ⏱️ Estimated Time

- Phase 1 (Verification): 15 minutes
- Phase 2 (Install): 10 minutes
- Phase 3 (SLAM): 1-2 hours
- Phase 4 (Nav2): 2-3 hours
- Phase 5 (Dependencies): 5 minutes
- Phase 6 (Testing): 1 hour
- Phase 7 (Optional): 1-2 hours
- Phase 8 (Demo Prep): 30 minutes

**Total: ~6-9 hours**

---

## 🔄 Rollback Plan

If something breaks:
1. **Don't panic** - all original files are unchanged
2. **Use original launch file**: `launch_sim_with_rviz.launch.py`
3. **Check git** (if using): `git status` to see what changed
4. **Remove new files** if needed (they're separate, won't affect originals)
5. **Rebuild**: `colcon build --symlink-install`

**Key Safety**: All new files are separate - original launch files remain untouched!

---

## 📝 Notes

- **Incremental approach**: Each phase builds on previous, tested before moving on
- **No breaking changes**: All new files are separate, originals preserved
- **Test frequently**: Verify at each step before proceeding
- **Keep it simple**: Start with basic configs, tune later if needed

**You've got this! The hard part (robot balance) is done. SLAM and Nav2 are just configuration.** 🚀
