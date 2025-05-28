import rclpy
import numpy as np
from collections import deque
from rclpy.node import Node
from rclpy import qos

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from custom_msgs.msg import TouchData

class StateMachine(Node):

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
        self._ref_pos_publisher = self.create_publisher(PoseStamped, '/feely_drone/ref_pose', qos.QoSPresetProfiles.SENSOR_DATA.value)
        self._ref_joint_state_publisher = self.create_publisher(JointState, '/feely_drone/ref_joint_state', qos.QoSPresetProfiles.SENSOR_DATA.value)
        self._bin_touch_data_publisher = self.create_publisher(JointState, '/feely_drone/bin_touch_state', qos.QoSPresetProfiles.SENSOR_DATA.value)
        # Subscribers
        self._touch_data_subscriber = self.create_subscription(TouchData, '/feely_drone/touch_data', self.touch_data_callback, qos.QoSPresetProfiles.SENSOR_DATA.value)

        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

    def timer_callback(self):
        # Get the reference pose and joint state messages
        pose_msg, joint_state_msg = self.get_references()
        # Publish the reference pose and joint state messages
        self._ref_pos_publisher.publish(pose_msg)
        self._ref_joint_state_publisher.publish(joint_state_msg)

    def get_references(self):
        timestamp_now = self.get_clock().now().to_msg()
        pose = PoseStamped()
        jointReference = JointState()
        binTouchState = JointState()

        pose.header.stamp = timestamp_now
        pose.header.frame_id = 'world'
        jointReference.header.stamp = timestamp_now
        jointReference.name = [f'arm{i+1}' for i in range(3)]
        binTouchState.header.stamp = timestamp_now
        
        # Compute the current touch state and publish for debugging purposes
        touch_state = np.mean(self.touch_data_deque, axis=0) > self.TOUCH_THRESHOLD
        binTouchState.position = touch_state.astype(float)
        self._bin_touch_data_publisher.publish(binTouchState)
        
        # ToDo Compute reference pose and 
        jointReference.position = np.zeros(3)


        return pose, jointReference


    def touch_data_callback(self, msg: TouchData):
        self.touch_data_deque.popleft()
        self.touch_data_deque.append(TouchData.raw_data)

def main(args=None):
    rclpy.init(args=args)

    feely_drone_state_machine = StateMachine()

    rclpy.spin(feely_drone_state_machine)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    feely_drone_state_machine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()