#!/usr/bin/env python3
"""
Nav2 Launch File for my_bot
Launches Nav2 stack for autonomous navigation
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    
    # Nav2 Bringup
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
        DeclareLaunchArgument('use_sim_time', default_value='true',
                             description='Use simulation time'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/kitchen_map.yaml'),
                             description='Full path to map yaml file'),
        nav2_bringup,
    ])
