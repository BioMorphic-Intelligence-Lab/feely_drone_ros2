from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tactile_perch_ctrl',
            executable='touch_sensor_driver',
            name='touch_sensor_driver',
            parameters=[{'frequency': 25.0}],
        ),
        Node(
            package='tactile_perch_ctrl',
            executable='feely_drone_state_machine',
            name='feely_drone_state_machine',
        ),
        Node(
            package='px4_interface',
            executable='px4_interface_node',
            name='px4_interface_node',
        )
    ])