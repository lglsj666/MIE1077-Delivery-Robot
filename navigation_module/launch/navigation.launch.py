from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_module',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[
                ('actions', '/actions'),
                ('navigation_feedback', '/navigation_feedback'),
                ('/scan', '/scan')
            ]
        )
    ])
