#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    #
    my_xarm6_dir = get_package_share_directory('my_xarm6')
    spawn_launch = os.path.join(my_xarm6_dir, 'launch', 'spawn_xarm6_gripper_moveit_world.launch.py')

    workspace_dir = os.path.expanduser('~/aliyu_ws/src/my_xarm6_app')
    venv_python = os.path.join(workspace_dir, 'venv', 'bin', 'python3')

    move_to_position = Node(
        package='my_xarm6_app',
        executable='move_to_position_llm',
        name='move_to_position_llm',
        output='screen'
    )

    llm_command_node = ExecuteProcess(
        cmd=[
            venv_python,
            '-m', 'my_xarm6_app.llm.llm_command_node'
        ],
        output='screen',
        cwd=os.path.join(workspace_dir, 'my_xarm6_app')
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch)
    )

    ld = LaunchDescription()
    ld.add_action(spawn_robot)
    ld.add_action(move_to_position)
    ld.add_action(llm_command_node)

    return ld