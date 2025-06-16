import rclpy
import numpy as np
from collections import deque
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import TouchData

from rclpy.qos import qos_profile_sensor_data, qos_profile_default


class WavyBoyNode(Node):
    def __init__(self):
        super().__init__('wavyboy_node')

        # Declare all parameters
        self.declare_parameter("frequency", 20.0)
        self.declare_parameter("touch_window_size", 10) # This assumes touch data is published at 250Hz, so 10 samples corresponds to 0.025 seconds
        self.declare_parameter("touch_threshold", 50)
        self.declare_parameter("alpha_rate", 0.5)  # Rate at which the alpha value is updated
        # Get all parameters
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.touch_window_size = self.get_parameter("touch_window_size").get_parameter_value().integer_value
        self.TOUCH_THRESHOLD = self.get_parameter("touch_threshold").get_parameter_value().integer_value
        self.ALPHA_RATE = self.get_parameter("alpha_rate").get_parameter_value().double_value
        self.DT = 1.0 / self.frequency
        
        # Init queue for touch data
        self.touch_data_deque = deque(np.zeros([1, 12], dtype=int), maxlen=self.touch_window_size)
        self.touch_data_baseline_deque = deque(np.zeros([1, 12], dtype=int), maxlen=self.touch_window_size)
        self.bin_touch_data = np.zeros([3, 3], dtype=bool)

        # Publishers
        self._arm_state_publisher = self.create_publisher(
            JointState,
            '/feely_drone/in/servo_states',
            qos_profile_default
        )
        self._contact_marker_publisher = self.create_publisher(
            MarkerArray,
            '/feely_drone/out/contact_marker',
            qos_profile_sensor_data
        )


        # Subscribers
        self._touch_state_subscriber = self.create_subscription(
            TouchData,
            '/feely_drone/out/touch_data',
            self.touch_data_callback,
            qos_profile_sensor_data
        )
        
        # Initialize Offboard Control Mode Timer
        self._timer = self.create_timer(0.1, self._publish_arm_ref)
        
        # Init arm state
        self._alpha = 0.5 * np.ones(3, dtype=float)
        self.start = self.get_clock().now().nanoseconds * 1e-9

    def _publish_arm_ref(self):
        # Create a JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        #joint_state_msg.name = ['joint_1', 'joint_2', 'joint_3']

        nocontact = ~self.bin_touch_data.any(axis=0)
        
        self._alpha = np.clip(
            self._alpha + self.ALPHA_RATE * (nocontact.astype(float)) 
                * self.DT * np.cos(2*np.pi / 10.0 * (self.get_clock().now().nanoseconds * 1e-9 - self.start)),
            0.0, 1.0
        )
        joint_state_msg.position = self._alpha.tolist()

        # Publish the JointState message
        self._arm_state_publisher.publish(joint_state_msg)

    def touch_data_callback(self, msg):
        self.touch_data_deque.popleft()
        self.touch_data_deque.append(np.array(msg.raw_data, dtype=int))

        self.touch_data_baseline_deque.popleft()
        self.touch_data_baseline_deque.append(np.array(msg.baseline_data, dtype=int))

        self.bin_touch_data = (np.mean(np.abs(
             np.array(self.touch_data_deque)
           - np.array(self.touch_data_baseline_deque)
           ), axis=0) > self.TOUCH_THRESHOLD)[:9].reshape([3, 3]).transpose()


def main(args=None):
    rclpy.init(args=args)

    wavy_boy_node = WavyBoyNode()

    rclpy.spin(wavy_boy_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wavy_boy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()