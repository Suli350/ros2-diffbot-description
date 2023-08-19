"""A kinematic differential-drive simulator.

Subscribes: cmd_vel (geometry_msgs/Twist)
Publishes:  odom (nav_msgs/Odometry), joint_states (sensor_msgs/JointState)
Broadcasts: odom -> base_footprint TF

Together with robot_state_publisher this makes the robot drive around in
RViz with spinning wheels, without needing Gazebo.
"""
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from diffbot_description.kinematics import (integrate, saturate_wheels, twist_to_wheels,
                                            wheels_to_twist, yaw_to_quaternion)


class DiffDriveSim(Node):

    def __init__(self):
        super().__init__('diff_drive_sim')
        self.r = self.declare_parameter('wheel_radius', 0.05).value
        self.sep = self.declare_parameter('wheel_separation', 0.30).value
        self.max_wheel = self.declare_parameter('max_wheel_speed', 20.0).value
        self.timeout = self.declare_parameter('cmd_timeout', 0.5).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        rate = self.declare_parameter('rate', 50.0).value

        self.x = self.y = self.theta = 0.0
        self.left_pos = self.right_pos = 0.0
        self.cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_step = self.get_clock().now()

        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(1.0 / rate, self.step)
        self.get_logger().info('diff drive simulator running, send Twist on /cmd_vel')

    def on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_step).nanoseconds * 1e-9
        self.last_step = now
        if dt <= 0.0:
            return

        v, w = self.cmd.linear.x, self.cmd.angular.z
        if (now - self.last_cmd_time).nanoseconds * 1e-9 > self.timeout:
            v = w = 0.0

        left, right = twist_to_wheels(v, w, self.r, self.sep)
        left, right = saturate_wheels(left, right, self.max_wheel)
        v, w = wheels_to_twist(left, right, self.r, self.sep)

        self.x, self.y, self.theta = integrate(self.x, self.y, self.theta, v, w, dt)
        self.left_pos += left * dt
        self.right_pos += right * dt
        qx, qy, qz, qw = yaw_to_quaternion(self.theta)
        stamp = now.to_msg()

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)

        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        js = JointState()
        js.header.stamp = stamp
        js.name = ['left_wheel_joint', 'right_wheel_joint']
        js.position = [self.left_pos, self.right_pos]
        js.velocity = [left, right]
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
