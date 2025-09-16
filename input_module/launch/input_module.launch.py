#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to this package's share folder
    pkg_share = get_package_share_directory('input_module')

    return LaunchDescription([
        Node(
            package='input_module',
            executable='input_module_node',
            name='input_module',
            output='screen',
            parameters=[  # 如需可从 YAML 加载参数
                # os.path.join(pkg_share, 'config', 'config.yaml')
            ],
            # remappings 如有必要也可加入
            # remappings=[('snapshot', '/snapshot')],
        ),
    ])
