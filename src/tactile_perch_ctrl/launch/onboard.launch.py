from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare a launch argument
    yaw_offset_arg = DeclareLaunchArgument(
        'target_yaw_estimate_offset',
        default_value='0.0',
        description='Offset of the Target Yaw (in degree) Ground Truth'
    )

    pos_offset_arg = DeclareLaunchArgument(
        'target_pos_estimate_offset',
        default_value='[0.0, 0.0, 0.0]',
        description='Offset of the Target Position Ground Truth'
    )
    
    return LaunchDescription([
        yaw_offset_arg,
        pos_offset_arg,
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
            parameters=[
                {'target_pos_estimate_offset': LaunchConfiguration('target_pos_estimate_offset'),
                 'target_yaw_estimate_offset': LaunchConfiguration('target_yaw_estimate_offset')}
            ]
        ),
        Node(
            package='px4_interface',
            executable='px4_interface_node',
            name='px4_interface_node',
        )
    ])