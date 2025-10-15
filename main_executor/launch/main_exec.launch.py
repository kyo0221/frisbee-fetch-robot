import os
import subprocess
import yaml
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Parameter file path
    config_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'main_params.yaml'
    )

    # Load launch parameters
    with open(config_file_path, 'r') as file:
        launch_params = yaml.safe_load(file)['launch']['ros__parameters']

    # Log level configuration
    log_level = LaunchConfiguration("log_level")
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    # Main executor node
    main_exec_node = Node(
        package = 'main_executor',
        executable = 'main_exec',
        parameters = [config_file_path],
        arguments=["--ros-args", "--log-level", log_level],
        output='screen'
    )

    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        output='screen'
    )

    # Create launch description
    launch_description = LaunchDescription()

    if(launch_params['joy'] is True):
        launch_description.add_entity(joy_node)

    launch_description.add_action(log_level_arg)
    launch_description.add_entity(main_exec_node)

    return launch_description
