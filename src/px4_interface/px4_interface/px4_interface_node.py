import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from custom_msgs.msg import StateMachineState
from px4_msgs.msg import (TrajectorySetpoint, VehicleOdometry,
                          VehicleCommand, OffboardControlMode)

class PX4InterfaceNode(Node):
    def __init__(self):
        super().__init__('px4_interface_node')

        # Publishers
        self._trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile_sensor_data
        )
        self._vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile_sensor_data
        )
        self._vehicle_odometry_publisher = self.create_publisher(
            PoseStamped,
            '/feely_drone/out/pose',
            qos_profile_sensor_data
        )
        self._offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile_sensor_data
        )

        # Subscribers
        self._ref_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/feely_drone/in/ref_pose',
            self.ref_pose_callback,
            qos_profile_sensor_data
        )
        self._odometry_subscriber = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.pose_callback,
            qos_profile_sensor_data
        )
        self._sm_state_subscriber = self.create_subscription(
            StateMachineState,
            '/feely_drone/out/state_machine_state',
            self.sm_state_callback,
            qos_profile_sensor_data
        )

        # Initialize Offboard Control Mode Timer
        self._timer = self.create_timer(0.1, self._publish_offboard_control_mode)

    def _publish_offboard_control_mode(self):
        # Create and publish Offboard Control Mode message
        offboard_control_mode = OffboardControlMode()
        offboard_control_mode.timestamp = int(self.get_clock().now().nanoseconds * 1e-3)
        offboard_control_mode.position = True
        offboard_control_mode.velocity = False
        offboard_control_mode.acceleration = False
        offboard_control_mode.attitude = False
        offboard_control_mode.body_rate = False

        self._offboard_control_mode_publisher.publish(offboard_control_mode)

    def ref_pose_callback(self, msg):
        # Handle incoming reference pose messages
        trajectory_setpoint = TrajectorySetpoint()
        # Timestamp the trajectory setpoint
        trajectory_setpoint.timestamp = int(self.get_clock().now().nanoseconds * 1e-3)

        # Transform from ENU to NED 
        trajectory_setpoint.position[0] =  msg.pose.position.y
        trajectory_setpoint.position[1] =  msg.pose.position.x
        trajectory_setpoint.position[2] = -msg.pose.position.z

        # Extract yaw from quaternion
        yaw = np.arctan2(msg.pose.orientation.z, msg.pose.orientation.w)
        trajectory_setpoint.yaw = - yaw

        # Publish Message
        self._trajectory_setpoint_publisher.publish(trajectory_setpoint)

    def pose_callback(self, msg):
        # Hande vehicle odometry msgs
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "world"
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        # Transform from NED to ENU
        pose_msg.pose.position.x =   float(msg.position[1])
        pose_msg.pose.position.y =   float(msg.position[0])
        pose_msg.pose.position.z = - float(msg.position[2])

        pose_msg.pose.orientation.w =  float(msg.q[0])
        pose_msg.pose.orientation.x =  float(msg.q[2])
        pose_msg.pose.orientation.y =  float(msg.q[1])
        pose_msg.pose.orientation.z = -float(msg.q[3])
    
        # Publish message
        self._vehicle_odometry_publisher.publish(pose_msg)

    def sm_state_callback(self, msg):
        # Handle incoming state machine state messages
        pass

def main(args=None):
    rclpy.init(args=args)

    px4_interface_node = PX4InterfaceNode()

    rclpy.spin(px4_interface_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    px4_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
