import rclpy
import numpy as np
from collections import deque
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from custom_msgs.msg import TouchData

from .state_machine import StateMachine

class StateMachineNode(Node):

    def __init__(self):
        super().__init__('feely_drone_state_machine')

        # Declare all parameters
        self.declare_parameter("frequency", 20.0)
        self.declare_parameter("touch_window_size", 125) # This assumes touch data is published at 250Hz, so 125 samples corresponds to 0.5 seconds
        self.declare_parameter("touch_threshold", 100)
        # Get all parameters
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.touch_window_size = self.get_parameter("touch_window_size").get_parameter_value().integer_value
        self.TOUCH_THRESHOLD = self.get_parameter("touch_threshold").get_parameter_value().integer_value

        # Init queue for touch data
        self.touch_data_deque = deque(np.zeros([self.touch_window_size, 12]))
       
        # Publishers
        self._ref_pos_publisher = self.create_publisher(PoseStamped, '/feely_drone/ref_pose', qos_profile_sensor_data)
        self._ref_joint_state_publisher = self.create_publisher(JointState, '/feely_drone/ref_joint_state', qos_profile_sensor_data)
        self._bin_touch_data_publisher = self.create_publisher(JointState, '/feely_drone/bin_touch_state', qos_profile_sensor_data)
        # Subscribers
        self._touch_data_subscriber = self.create_subscription(TouchData, '/feely_drone/touch_data', self.touch_data_callback, qos_profile_sensor_data)

        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        # Init the state machine
        self.sm = StateMachine(dt=1.0 / self.frequency,        # Delta T
                               m_arm=np.ones(3),               # Mass of the Arm
                               l_arm=np.ones(3),               # Length of the Arm
                               p0=np.zeros([3, 3]),            # Offset Position of Arms
                               rot0=np.zeros([3, 3, 3]),       # Offset Rotation of Arms
                               K=np.diag(np.ones(3)),          # Stiffness Matrix of the arm
                               A=np.ones(3),                   # Actuation map
                               q0=np.deg2rad(75) * np.ones(3), # Neutral joint states
                               g=np.array([0, 0, -9.81]),      # Gravity Vector
                               target_pos_estimate=np.array([0, 0, 1.5]),
                               target_yaw_estimate=np.zeros([1]))

    def timer_callback(self):
        # Get the reference pose and joint state messages
        pose_msg, joint_state_msg = self.get_references()
        # Publish the reference pose and joint state messages
        self._ref_pos_publisher.publish(pose_msg)
        self._ref_joint_state_publisher.publish(joint_state_msg)

    def get_references(self):
        # Get Timestamp
        timestamp_now = self.get_clock().now().to_msg()

        # Init publish messages
        pose = PoseStamped()
        jointReference = JointState()
        binTouchState = JointState()

        # Set timestamps and frame ids
        pose.header.stamp = timestamp_now
        pose.header.frame_id = 'world'
        jointReference.header.stamp = timestamp_now
        jointReference.name = [f'arm{i+1}' for i in range(3)]
        binTouchState.header.stamp = timestamp_now
        
        # Compute the current touch state and publish for debugging purposes
        touch_state = np.mean(self.touch_data_deque, axis=0) > self.TOUCH_THRESHOLD
        binTouchState.position = touch_state.astype(float)
        self._bin_touch_data_publisher.publish(binTouchState)
        
        # Compute reference pose and arm opening angle
        control = self.sm.control(x=np.zeros(4),
                                  v=np.zeros(4),
                                  contact=np.zeros(9, dtype=bool))
        
        # Extract Desired Position
        pose.pose.position.x = control['p_des'][0]
        pose.pose.position.y = control['p_des'][1]
        pose.pose.position.z = control['p_des'][2]

        # Extract Desired Yaw
        pose.pose.orientation.w = np.cos(control['yaw_des'][0] / 2.0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = np.sin(control['yaw_des'][0] / 2.0)

        # Extract Desired Arm Opening Angle
        jointReference.position = control['alpha']

        return pose, jointReference

    def touch_data_callback(self, msg: TouchData):
        self.touch_data_deque.popleft()
        self.touch_data_deque.append(TouchData.raw_data)

def main(args=None):
    rclpy.init(args=args)

    feely_drone_state_machine = StateMachineNode()

    rclpy.spin(feely_drone_state_machine)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    feely_drone_state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()