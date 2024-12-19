import os
from os import environ, pathsep

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    ExecuteProcess,
    OpaqueFunction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def start_gzserver(context, *args, **kwargs):
    world_package = LaunchConfiguration('world_package').perform(context)
    world_name = LaunchConfiguration('world_name').perform(context)

    # Dynamically get the package path based on the world_package argument
    pkg_path = get_package_share_directory(world_package)
    world_file = os.path.join(pkg_path, 'worlds', world_name + '.world')

    if not os.path.exists(world_file):
        raise FileNotFoundError(f"World file {world_name}.world not found in package {world_package}")

    params_file = PathJoinSubstitution(
        substitutions=[pkg_path, 'config', 'gazebo_params.yaml'])

    # Command to start the gazebo server.
    gazebo_server_cmd_line = [
        'gzserver', '-s', 'libgazebo_ros_init.so',
        '-s', 'libgazebo_ros_factory.so', world_file,
        '--ros-args', '--params-file', params_file]
    
    # If debugging is required, launch with gdb.
    debug = LaunchConfiguration('debug').perform(context)
    if debug == 'True':
        gazebo_server_cmd_line = (
            ['xterm', '-e', 'gdb', '-ex', 'run', '--args'] +
            gazebo_server_cmd_line
        )

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=gazebo_server_cmd_line, output='screen')

    return [start_gazebo_server_cmd]


def generate_launch_description():
    declare_world_package = DeclareLaunchArgument(
        'world_package', default_value='lab5',
        description="Specify the package containing the world file"
    )
    declare_world_name = DeclareLaunchArgument(
        'world_name', default_value='',
        description="Specify world name, we'll convert to full path"
    )
    declare_debug = DeclareLaunchArgument(
        'debug', default_value='False',
        choices=['True', 'False'],
        description='If debug, start the gazebo world in a gdb session in an xterm terminal'
    )

    start_gazebo_server_cmd = OpaqueFunction(function=start_gzserver)

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'], output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_world_package)
    ld.add_action(declare_world_name)
    ld.add_action(declare_debug)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    return ld
