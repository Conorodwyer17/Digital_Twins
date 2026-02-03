#!/usr/bin/env python3
"""
SLAM Launch File for my_bot
Launches SLAM_Toolbox for online mapping
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # SLAM Toolbox node
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
            'minimum_time_interval': 0.5,
            'transform_publish_period': 0.02,
            'map_update_interval': 5.0,
            'max_laser_range': 8.0,
            'minimum_travel_distance': 0.5,
            'minimum_travel_heading': 0.5,
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                             description='Use simulation time'),
        slam_toolbox,
    ])
