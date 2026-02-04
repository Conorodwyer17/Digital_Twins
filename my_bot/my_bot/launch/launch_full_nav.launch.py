#!/usr/bin/env python3
"""
Complete Launch File: Simulation + RViz + SLAM + Nav2
Launches everything needed for autonomous navigation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_bot'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default=os.path.expanduser('~/kitchen_map.yaml'))

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
        )]), launch_arguments={'use_sim_time': use_sim_time, 'use_ros2_control': 'true'}.items()
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
        parameters=[{'use_sim_time': use_sim_time}]
    )
    delayed_rviz = TimerAction(period=3.0, actions=[rviz_node])

    # Nav2
    nav2_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'nav2',
        'nav2_params.yaml'
    )
    
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
    delayed_nav2 = TimerAction(period=10.0, actions=[nav2_bringup])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/kitchen_map.yaml')),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        rsp,
        gazebo,
        delayed_spawn_entity,
        delayed_rviz,
        delayed_nav2,
    ])
