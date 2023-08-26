"""Publish a constant Twist so the robot drives a circle (or figure eight)."""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CircleDriver(Node):

    def __init__(self):
        super().__init__('circle_driver')
        self.v = self.declare_parameter('linear', 0.3).value
        self.w = self.declare_parameter('angular', 0.5).value
        self.figure_eight = self.declare_parameter('figure_eight', False).value
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start = self.get_clock().now()
        self.create_timer(0.05, self.tick)

    def tick(self):
        t = (self.get_clock().now() - self.start).nanoseconds * 1e-9
        cmd = Twist()
        cmd.linear.x = self.v
        cmd.angular.z = self.w
        if self.figure_eight:
            period = 2.0 * math.pi / abs(self.w)
            if int(t // period) % 2:
                cmd.angular.z = -self.w
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CircleDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
