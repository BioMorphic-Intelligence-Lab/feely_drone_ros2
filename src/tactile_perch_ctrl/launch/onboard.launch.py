from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tactile_perch_ctrl',
            executable='touch_sensor_driver',
            name='touch_sensor_driver',
            parameters=[{'frequency': 250.0}],
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
        ),
        ExecuteProcess(
            cmd=['docker', 'run', '-it', '--rm', '--privileged',
                 '--net', 'host',  'antbre/uxrce_agent serial',
                  '--dev', '/dev/ttyAMA0', '-b', '921600'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['docker', 'run', '-it', '--rm', '--privileged',
                 '--net', 'host',  'antbre/uxrce_agent serial',
                  '--dev', '/dev/ttyACM0'],
            output='screen'
        )
    ])