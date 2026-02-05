#!/usr/bin/env python3
"""
Nav2 Launch File for my_bot
Launches Nav2 stack for autonomous navigation
Uses individual nodes approach for better compatibility
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Get map path and expand ~ if present
    map_path_raw = context.launch_configurations.get('map', '/home/ros_user/kitchen_map.yaml')
    map_file_arg = os.path.expanduser(map_path_raw)
    
    nav2_params_file = os.path.join(
        get_package_share_directory('my_bot'),
        'config',
        'nav2',
        'nav2_params.yaml'
    )
    
    # Map Server - use expanded path
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file_arg
        }]
    )
    
    # AMCL (Localization)
    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Planner Server
    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace='',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Controller Server
    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace='',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Behavior Tree Navigator
    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace='',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Waypoint Follower
    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace='',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Smoother Server
    smoother_server = LifecycleNode(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        namespace='',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Behavior Server (provides spin, backup, wait actions for behavior trees)
    behavior_server = LifecycleNode(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace='',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Lifecycle Manager (manages all lifecycle nodes)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 10.0,  # Increase timeout to allow map_server to load map
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'bt_navigator',
                'waypoint_follower',
                'smoother_server',
                'behavior_server'
            ]
        }]
    )
    
    # Initial pose publisher (sets AMCL initial pose automatically)
    # Try installed path first, fallback to source path
    installed_script = os.path.join(
        get_package_prefix('my_bot'),
        'lib', 'my_bot', 'set_initial_pose.py'
    )
    # Source path: from share/my_bot go up to workspace, then to my_bot/my_bot/my_bot/scripts
    source_script = os.path.join(
        get_package_share_directory('my_bot'),
        '..', '..', '..', '..', 'my_bot', 'my_bot', 'my_bot', 'scripts', 'set_initial_pose.py'
    )
    source_script = os.path.abspath(source_script)
    # Use installed path if it exists, otherwise use source path
    script_path = installed_script if os.path.exists(installed_script) else source_script
    initial_pose_publisher = ExecuteProcess(
        cmd=['python3', script_path],
        output='screen'
    )
    
    return [
        DeclareLaunchArgument('use_sim_time', default_value='true',
                             description='Use simulation time'),
        DeclareLaunchArgument('map', default_value='/home/ros_user/kitchen_map.yaml',
                             description='Full path to map yaml file'),
        map_server,
        amcl,
        planner_server,
        controller_server,
        bt_navigator,
        waypoint_follower,
        smoother_server,
        behavior_server,
        # Use event handler to wait for map_server to be ready before starting lifecycle manager
        # Increased delay to allow map_server to fully initialize and register its services
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=map_server,
                on_start=[TimerAction(period=8.0, actions=[lifecycle_manager])]
            )
        ),
        TimerAction(period=10.0, actions=[initial_pose_publisher]),
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
