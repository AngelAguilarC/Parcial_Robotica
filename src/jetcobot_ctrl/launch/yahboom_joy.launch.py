from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='jetcobot_ctrl',
            executable='yahboom_joy',
            name='joy_ctrl',
            output='screen'
        )
    ])