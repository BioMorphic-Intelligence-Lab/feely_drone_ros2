import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_default

from .state_machine import State
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import TouchData, StateMachineState

from .simple_demo_state_machine import SimpleDemoStateMachine


class SimpleDemoStateMachineNode(Node):

    def __init__(self):
        super().__init__('feely_drone_state_machine')

        # Declare all parameters
        self.declare_parameter("frequency", 20.0)
        # Get all parameters
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.dt = 1.0 / self.frequency
        # Publishers
        self._ref_pos_publisher = self.create_publisher(PoseStamped, '/feely_drone/in/ref_pose', qos_profile_sensor_data)
        self._ref_twist_publisher = self.create_publisher(TwistStamped, '/feely_drone/in/ref_twist', qos_profile_sensor_data)
        self._ref_joint_state_publisher = self.create_publisher(JointState, '/feely_drone/in/servo_states', qos_profile_default)
        self._sm_state_publisher = self.create_publisher(StateMachineState, '/feely_drone/out/state_machine_state', qos_profile_sensor_data)
        
        # Subscribers
        self._odometry_subscriber = self.create_subscription(PoseStamped, '/feely_drone/out/pose', self.odometry_data_callback, qos_profile_sensor_data)
        self._target_pos_subscriber = self.create_subscription(PoseStamped, '/target/out/pose', self.target_pose_callback, qos_profile_sensor_data)
        
        # Init Timers
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)
        
        # Init the state machine
        self.sm = SimpleDemoStateMachine(dt=1.0 / self.frequency)

        # Pose
        self._position = np.zeros(3, dtype=float)
        self._yaw = 0.0

    def timer_callback(self):
        # Get the reference pose and joint state messages
        pose_msg, twist_msg, joint_state_msg = self.get_references()

        # Get the current state of the state machine
        sm_state_msg = StateMachineState()
        sm_state_msg.state = self.sm.state.value
        sm_state_msg.header.stamp = pose_msg.header.stamp
        
        # Publish the reference pose and joint state messages
        self._ref_pos_publisher.publish(pose_msg)
        self._ref_twist_publisher.publish(twist_msg)
        self._ref_joint_state_publisher.publish(joint_state_msg)
        self._sm_state_publisher.publish(sm_state_msg)


    def get_references(self):
        # Get Timestamp
        timestamp_now = self.get_clock().now().to_msg()

        # Init publish messages
        pose = PoseStamped()
        twist = TwistStamped()
        jointReferenceMsg = JointState()
        
        # Set timestamps and frame ids
        pose.header.stamp = timestamp_now
        pose.header.frame_id = 'world'
        twist.header.stamp = timestamp_now
        twist.header.frame_id = 'world'
        jointReferenceMsg.header.stamp = timestamp_now
        jointReferenceMsg.name = [f'arm{i+1}' for i in range(3)]
        
        # Compute reference pose and arm opening angle
        control = self.sm.control(x=np.concatenate((self._position,
                                                    [self._yaw])),
                                  v=np.zeros(4),
                                  contact=np.zeros(3, dtype=bool)  # Assuming no contact for this example
        )


        # Extract Desired Position
        dist = np.linalg.norm(control['p_des'] - self._position)
        if self.sm.state == State.TAKEOFF:
            if dist < 0.25:
                cmd = np.zeros(3)
            else:
                cmd = 0.5 * (control['p_des'] - self._position) / dist
            
            pose.pose.position.x = control['p_des'][0]
            pose.pose.position.y = control['p_des'][1]
            pose.pose.position.z = control['p_des'][2]
        else:
            if dist < 0.05:
                cmd = np.zeros(3)
            else:
                cmd = 0.2 * (control['p_des'] - self._position) / dist
            pose.pose.position.x = self._position[0] + self.dt * cmd[0]
            pose.pose.position.y = self._position[1] + self.dt * cmd[1]
            pose.pose.position.z = self._position[2] + self.dt * cmd[2]

        # Extract Desired Yaw
        pose.pose.orientation.w = np.cos(control['yaw_des'] / 2.0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = np.sin(control['yaw_des'] / 2.0)
        
        # Extract Desired Velocity
        twist.twist.linear.x = cmd[0]
        twist.twist.linear.y = cmd[1]
        twist.twist.linear.z = cmd[2]
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = control['v_des'][3]
        
        # Extract Desired Arm Opening Angle
        jointReferenceMsg.position = control['alpha']

        return pose, twist, jointReferenceMsg

    def odometry_data_callback(self, msg: PoseStamped):
        self._position = np.array([msg.pose.position.x,
                                  msg.pose.position.y,
                                  msg.pose.position.z], dtype=float)
        self._yaw = np.arctan2(
            2.0 * (msg.pose.orientation.w * msg.pose.orientation.z + msg.pose.orientation.x * msg.pose.orientation.y),
                   msg.pose.orientation.w**2 + msg.pose.orientation.x**2 - msg.pose.orientation.y**2 - msg.pose.orientation.z**2
        )

    def target_pose_callback(self, msg: PoseStamped):
        self.sm.set_target_pos_estimate(
            pos=[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            yaw=np.arctan2(
                2.0 * (msg.pose.orientation.w * msg.pose.orientation.z + msg.pose.orientation.x * msg.pose.orientation.y),
                msg.pose.orientation.w**2 + msg.pose.orientation.x**2 - msg.pose.orientation.y**2 - msg.pose.orientation.z**2
            )
        )

def main(args=None):
    rclpy.init(args=args)

    feely_drone_state_machine = SimpleDemoStateMachineNode()

    rclpy.spin(feely_drone_state_machine)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    feely_drone_state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
