from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tactile_perch_ctrl',
            executable='simple_demo_state_machine',
            name='simple_demo_state_machine',
        ),
        Node(
            package='px4_interface',
            executable='px4_interface_node',
            name='px4_interface_node',
        ),
    ])
