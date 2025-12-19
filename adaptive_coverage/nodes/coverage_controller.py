#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class CoverageController(Node):

    def __init__(self):
        super().__init__("coverage_controller")

        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_pose = None
        self.get_logger().info("Coverage Controller Node Started")

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None:
            return
        
        cmd = Twist()

        cmd.linear.x = 1.5
        cmd.angular.z = 0.8

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()