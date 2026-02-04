#!/usr/bin/env python3
"""
Nav2 Launch File - Using Individual Nodes
Alternative launch file that launches Nav2 components individually
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default=os.path.expanduser('~/kitchen_map.yaml'))
    
    nav2_params_file = os.path.join(
        get_package_share_directory('my_bot'),
        'config',
        'nav2',
        'nav2_params.yaml'
    )
    
    # Map Server
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file
        }]
    )
    
    # AMCL (Localization)
    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Planner Server
    planner_server = LifecycleNode(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Controller Server
    controller_server = LifecycleNode(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Behavior Tree Navigator
    bt_navigator = LifecycleNode(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Waypoint Follower (optional)
    waypoint_follower = LifecycleNode(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params_file]
    )
    
    # Velocity Smoother
    smoother_server = LifecycleNode(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
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
            'node_names': [
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'bt_navigator',
                'waypoint_follower',
                'smoother_server'
            ]
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/kitchen_map.yaml')),
        map_server,
        amcl,
        planner_server,
        controller_server,
        bt_navigator,
        waypoint_follower,
        smoother_server,
        TimerAction(period=2.0, actions=[lifecycle_manager]),
    ])
