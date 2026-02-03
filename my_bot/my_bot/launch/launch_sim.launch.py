import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME

    # Set GAZEBO_MODEL_PATH to include our kitchen_dining model
    # Get the absolute path to the models directory
    worlds_path = os.path.join(get_package_share_directory(package_name), 'worlds')
    models_path = os.path.join(worlds_path, 'model')
    # Convert to absolute path
    models_path = os.path.abspath(models_path)
    
    # Get existing GAZEBO_MODEL_PATH if it exists
    existing_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_path:
        gazebo_model_path = existing_path + ':' + models_path
    else:
        gazebo_model_path = models_path

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    # Use kitchen_dining world with collision geometry enabled
    world_file = os.path.join(get_package_share_directory(package_name),'worlds','kitchen_dining_world.world')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'world': world_file,
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
                    }.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    # Delay spawn_entity to wait for Gazebo to fully start
    delayed_spawn_entity = TimerAction(period=5.0, actions=[spawn_entity])

    # Note: This launch file uses the legacy gazebo_ros_diff_drive plugin,
    # which does NOT require ros2_control controllers. The plugin handles
    # command velocity directly via /cmd_vel topic and publishes odometry.

    # Launch them all!
    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        rsp,
        gazebo,
        delayed_spawn_entity
    ])