#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('p3dx_llm_control')
    config_dir = os.path.join(pkg_dir, 'config')

    # Paths to config files
    velocity_limits_config = os.path.join(config_dir, 'velocity_limits.yaml')
    llm_config = os.path.join(config_dir, 'llm_config.yaml')

    # Create the LLM controller node
    llm_controller_node = Node(
        package='p3dx_llm_control',
        executable='llm_controller_node',
        name='p3dx_llm_controller',
        output='screen',
        parameters=[
            velocity_limits_config,
            llm_config
        ],
        emulate_tty=True
    )

    return LaunchDescription([
        llm_controller_node
    ])
