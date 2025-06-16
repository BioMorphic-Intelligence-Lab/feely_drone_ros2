import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_default

from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from custom_msgs.msg import TouchData, StateMachineState

from .state_machine import StateMachine
from .search_pattern import SinusoidalSearchPattern

class StateMachineNode(Node):

    def __init__(self):
        super().__init__('feely_drone_state_machine')

        # Declare all parameters
        self.declare_parameter("frequency", 20.0)
        self.declare_parameter("touch_window_size", 10) # This assumes touch data is published at 250Hz, so 10 samples corresponds to 0.025 seconds
        self.declare_parameter("touch_threshold", 50)
        # Get all parameters
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.touch_window_size = self.get_parameter("touch_window_size").get_parameter_value().integer_value
        self.TOUCH_THRESHOLD = self.get_parameter("touch_threshold").get_parameter_value().integer_value

        # Init queue for touch data
        self.touch_data_deque = deque(np.zeros([1, 12], dtype=int), maxlen=self.touch_window_size)
        self.touch_data_baseline_deque = deque(np.zeros([1, 12], dtype=int), maxlen=self.touch_window_size)

        # Publishers
        self._ref_pos_publisher = self.create_publisher(PoseStamped, '/feely_drone/in/ref_pose', qos_profile_sensor_data)
        self._ref_twist_publisher = self.create_publisher(TwistStamped, '/feely_drone/in/ref_twist', qos_profile_sensor_data)
        self._ref_joint_state_publisher = self.create_publisher(JointState, '/feely_drone/in/servo_states', qos_profile_default)
        self._bin_touch_data_publisher = self.create_publisher(JointState, '/feely_drone/out/bin_touch_state', qos_profile_sensor_data)
        self._sm_State_publisher = self.create_publisher(StateMachineState, '/feely_drone/out/state_machine_state', qos_profile_sensor_data)
        self._contact_marker_publisher = self.create_publisher(MarkerArray, '/feely_drone/out/contact_marker', qos_profile_sensor_data)

        # Subscribers
        self._touch_data_subscriber = self.create_subscription(TouchData, '/feely_drone/out/touch_data', self.touch_data_callback, qos_profile_sensor_data)
        self._odometry_subscriber = self.create_subscription(PoseStamped, '/feely_drone/out/pose', self.odometry_data_callback, qos_profile_sensor_data)

        # Init Timers
        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        # Init static offsets for the arms
        #p0 = np.array([[-0.125, 0.125, 0.0],
        #               [ 0.0,  -0.125, 0.0],
        #               [ 0.125, 0.125, 0.0]])
        #rot0 = np.array([
        #    R.from_euler('xyz', [-np.deg2rad(90), 0.0, 0.0]).as_matrix(),
        #    R.from_euler('xyz', [-np.deg2rad(90), 0.0, -np.deg2rad(180)]).as_matrix(),
        #    R.from_euler('xyz', [-np.deg2rad(90), 0.0, 0.0]).as_matrix(),
        #]).reshape(3, 3, 3)  
        p0 = np.array([[ 0.125, 0.125, 0.0],
                       [-0.125, 0.125, 0.0],
                       [ 0.0,  -0.125, 0.0],])
        rot0 = np.array([
            R.from_euler('xyz', [-np.deg2rad(90), 0.0, 0.0]).as_matrix(),
            R.from_euler('xyz', [-np.deg2rad(90), 0.0, 0.0]).as_matrix(),
            R.from_euler('xyz', [-np.deg2rad(90), 0.0, -np.deg2rad(180)]).as_matrix(),
        ]).reshape(3, 3, 3)  


        # Init the state machine
        self.sm = StateMachine(dt=1.0 / self.frequency,            # Delta T
                               m_arm=np.ones(3),                   # Mass of the Arm
                               l_arm=np.array([0.15, 0.25, 0.25]), # Length of the Arm
                               p0=p0,                              # Offset Position of Arms
                               rot0=rot0,                          # Offset Rotation of Arms
                               K=np.diag(100*np.ones(3)),          # Stiffness Matrix of the arm
                               A=-120 * np.ones(3),                # Actuation map
                               q0=np.deg2rad(75) * np.ones(3),     # Neutral joint states
                               g=np.array([0, 0, -9.81]),          # Gravity Vector
                               target_pos_estimate=np.array([0, 0, 1.5]),
                               target_yaw_estimate=np.zeros([1]),
                               searching_pattern=SinusoidalSearchPattern(
                                    params=np.stack([[0.75, 0.75, 0],   # Amplitude
                                                     [2.0, 1.0, 0.0],   # Frequency
                                                     [0.0, 0.0, 0.0],   # Phase Shift
                                                     [0.0, 0.0, 1.5]]), # Offset
                                    dt=1.0 / self.frequency,
                                    vel_norm=0.15)
        )
        
        # Init the binary touch state
        self._bin_touch_state = np.zeros(9, dtype=bool)

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
        self._sm_State_publisher.publish(sm_state_msg)

        # Publish the contact marker for visualization
        contact_locs = self.sm.contact_locs
        marker_array = MarkerArray()
        for i in range(3):
            for j in range(3):
                marker = Marker()
                marker.header.stamp = pose_msg.header.stamp
                marker.header.frame_id = 'world'
                marker.id = i + j * 3 + 1  # Unique ID for each marker
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = contact_locs[i % 2, j, 0]
                marker.pose.position.y = contact_locs[i % 2, j, 1]
                marker.pose.position.z = contact_locs[i % 2, j, 2]
                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                marker.color.a = 1.0
                marker.color.r = 1.0 if self._bin_touch_state[(i % 2)*3 + j] else 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0 if not self._bin_touch_state[i*3 + j] else 0.0
                marker.lifetime = rclpy.duration.Duration(seconds=1.0 / self.frequency).to_msg()
                
                marker_array.markers.append(marker)
        self._contact_marker_publisher.publish(marker_array)

    def get_references(self):
        # Get Timestamp
        timestamp_now = self.get_clock().now().to_msg()

        # Init publish messages
        pose = PoseStamped()
        twist = TwistStamped()
        jointReferenceMsg = JointState()
        binTouchStateMsg = JointState()

        # Set timestamps and frame ids
        pose.header.stamp = timestamp_now
        pose.header.frame_id = 'world'
        twist.header.stamp = timestamp_now
        twist.header.frame_id = 'world'
        jointReferenceMsg.header.stamp = timestamp_now
        jointReferenceMsg.name = [f'arm{i+1}' for i in range(3)]
        binTouchStateMsg.header.stamp = timestamp_now

        # Compute the current touch state and publish for debugging purposes
        self._bin_touch_state = np.mean(np.abs(
             np.array(self.touch_data_deque)
           - np.array(self.touch_data_baseline_deque)
           ), axis=0) > self.TOUCH_THRESHOLD
        binTouchStateMsg.position = self._bin_touch_state.astype(float)
        self._bin_touch_data_publisher.publish(binTouchStateMsg)

        # Compute reference pose and arm opening angle
        control = self.sm.control(x=np.concatenate((self._position,
                                                    [self._yaw])),
                                  v=np.zeros(4),
                                  contact=self._bin_touch_state[:9].reshape([3,3]).transpose()
        )
        # Extract Desired Position
        pose.pose.position.x = control['p_des'][0]
        pose.pose.position.y = control['p_des'][1]
        pose.pose.position.z = control['p_des'][2]

        # Extract Desired Yaw
        pose.pose.orientation.w = np.cos(control['yaw_des'][0] / 2.0)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = np.sin(control['yaw_des'][0] / 2.0)
        
        # Extract Desired Velocity
        twist.twist.linear.x = control['v_des'][0]
        twist.twist.linear.y = control['v_des'][1]
        twist.twist.linear.z = control['v_des'][2]
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = control['v_des'][3]
        
        # Extract Desired Arm Opening Angle
        jointReferenceMsg.position = control['alpha']

        return pose, twist, jointReferenceMsg

    def touch_data_callback(self, msg: TouchData):
        self.touch_data_deque.popleft()
        self.touch_data_deque.append(np.array(msg.raw_data, dtype=int))

        self.touch_data_baseline_deque.popleft()
        self.touch_data_baseline_deque.append(np.array(msg.baseline_data, dtype=int))

    def odometry_data_callback(self, msg: PoseStamped):
        self._position = np.array([msg.pose.position.x,
                                  msg.pose.position.y,
                                  msg.pose.position.z], dtype=float)
        self._yaw = np.arctan2(
            2.0 * (msg.pose.orientation.w * msg.pose.orientation.z + msg.pose.orientation.x * msg.pose.orientation.y),
                   msg.pose.orientation.w**2 + msg.pose.orientation.x**2 - msg.pose.orientation.y**2 - msg.pose.orientation.z**2
        )

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
