#!/usr/bin/env python3
"""
Combined Launch File: Simulation + RViz + SLAM
Launches everything needed for mapping
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'

    # Set GAZEBO_MODEL_PATH
    worlds_path = os.path.join(get_package_share_directory(package_name), 'worlds')
    models_path = os.path.join(worlds_path, 'model')
    models_path = os.path.abspath(models_path)
    
    existing_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_path:
        gazebo_model_path = existing_path + ':' + models_path
    else:
        gazebo_model_path = models_path

    # Robot state publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Gazebo
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    world_file = os.path.join(get_package_share_directory(package_name),'worlds','kitchen_dining_world.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )
    delayed_spawn_entity = TimerAction(period=5.0, actions=[spawn_entity])

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'visualize.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )
    delayed_rviz = TimerAction(period=3.0, actions=[rviz_node])

    # SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'max_laser_range': 8.0,
            'resolution': 0.05,
            'minimum_time_interval': 0.5,
            'transform_publish_period': 0.02,
            'map_update_interval': 5.0,
            'minimum_travel_distance': 0.5,
            'minimum_travel_heading': 0.5,
        }]
    )
    delayed_slam = TimerAction(period=7.0, actions=[slam_toolbox])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        rsp,
        gazebo,
        delayed_spawn_entity,
        delayed_rviz,
        delayed_slam,
    ])
