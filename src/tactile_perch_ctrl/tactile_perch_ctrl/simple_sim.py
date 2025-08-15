import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped, TwistStamped
import numpy as np


def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return q / norm


def quaternion_slerp(q1, q2, t):
    """Slerp between two quaternions."""
    q1 = normalize_quaternion(q1)
    q2 = normalize_quaternion(q2)
    dot = np.dot(q1, q2)

    # Ensure shortest path
    if dot < 0.0:
        q2 = -q2
        dot = -dot

    DOT_THRESHOLD = 0.9995
    if dot > DOT_THRESHOLD:
        # Very close — use linear interpolation
        result = q1 + t * (q2 - q1)
        return normalize_quaternion(result)

    theta_0 = np.arccos(dot)  # angle between inputs
    theta = theta_0 * t
    q3 = q2 - q1 * dot
    q3 = normalize_quaternion(q3)

    return q1 * np.cos(theta) + q3 * np.sin(theta)


class SimpleSim(Node):
    def __init__(self):
        super().__init__('simple_sim')

        # Internal state
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "world"
        self.last_time = self.get_clock().now()

        # Subscribers
        self.create_subscription(
            PoseStamped,
            '/feely_drone/in/ref_pose',
            self.pose_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            TwistStamped,
            '/feely_drone/in/ref_twist',
            self.twist_callback,
            qos_profile_sensor_data
        )

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/feely_drone/out/pose',
            qos_profile_sensor_data
        )

        # Store latest commands
        self.ref_pose = None
        self.ref_twist = TwistStamped()

        # Convergence gains
        self.pos_gain = 1.0  # 1/s
        self.ori_gain = 1.0  # 1/s

        # Timer
        self.create_timer(0.02, self.update_pose)  # 50 Hz

    def pose_callback(self, msg: PoseStamped):
        self.ref_pose = msg

    def twist_callback(self, msg: TwistStamped):
        self.ref_twist = msg

    def update_pose(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if self.ref_pose is not None:
            # Position convergence
            dx = self.ref_pose.pose.position.x - self.current_pose.pose.position.x
            dy = self.ref_pose.pose.position.y - self.current_pose.pose.position.y
            dz = self.ref_pose.pose.position.z - self.current_pose.pose.position.z
            self.current_pose.pose.position.x += self.pos_gain * dx * dt
            self.current_pose.pose.position.y += self.pos_gain * dy * dt
            self.current_pose.pose.position.z += self.pos_gain * dz * dt

            # Orientation convergence
            q_current = np.array([
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w
            ])
            q_target = np.array([
                self.ref_pose.pose.orientation.x,
                self.ref_pose.pose.orientation.y,
                self.ref_pose.pose.orientation.z,
                self.ref_pose.pose.orientation.w
            ])

            if np.linalg.norm(q_current) < 1e-6:  # init if empty
                q_current = q_target

            q_new = quaternion_slerp(q_current, q_target, self.ori_gain * dt)

            self.current_pose.pose.orientation.x = q_new[0]
            self.current_pose.pose.orientation.y = q_new[1]
            self.current_pose.pose.orientation.z = q_new[2]
            self.current_pose.pose.orientation.w = q_new[3]

        # Twist integration
        self.current_pose.pose.position.x += self.ref_twist.twist.linear.x * dt
        self.current_pose.pose.position.y += self.ref_twist.twist.linear.y * dt
        self.current_pose.pose.position.z += self.ref_twist.twist.linear.z * dt

        self.current_pose.header.stamp = now.to_msg()
        self.pose_pub.publish(self.current_pose)


def main(args=None):
    rclpy.init(args=args)
    feely_drone_state_machine = SimpleSim()
    rclpy.spin(feely_drone_state_machine)
    feely_drone_state_machine.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
