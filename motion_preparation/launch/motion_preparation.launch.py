from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motion_preparation',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
        ),
        Node(
            package='motion_preparation',
            executable='recorder_node',
            name='recorder_node',
            output='screen',
        ),
        Node(
            package='motion_preparation',
            executable='gui_node',
            name='gui_node',
            output='screen',
        ),
        Node(
            package='motion_preparation',
            executable='grasp_attacher_node',
            name='grasp_attacher_node',
            output='screen',
        ),
    ])
