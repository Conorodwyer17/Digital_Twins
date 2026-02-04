#!/usr/bin/env python3
"""
Nav2 Launch File for my_bot
Launches Nav2 stack for autonomous navigation
Uses individual nodes approach for better compatibility
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file_arg = LaunchConfiguration('map', default=os.path.expanduser('~/kitchen_map.yaml'))
    
    # Expand user path properly
    import launch.substitutions
    from launch.substitutions import PythonExpression
    
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
        TimerAction(period=2.0, actions=[lifecycle_manager]),
    ])
