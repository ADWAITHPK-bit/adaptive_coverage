#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.front_distance = float('inf')

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Obstacle Avoidance Node Started")

    def scan_callback(self, msg):
        
    # Divide laser scan into left, front, right
        center = len(msg.ranges) // 2
        side_angles = 15  # points to consider

        left_ranges = msg.ranges[center+1:center+1+side_angles]
        right_ranges = msg.ranges[center-side_angles:center]

    # Filter invalid measurements
        left = min([r for r in left_ranges if r > 0.0], default=float('inf'))
        right = min([r for r in right_ranges if r > 0.0], default=float('inf'))

        self.front_distance = min([msg.ranges[center], 1.0])  # limit front distance
        self.left_distance = left
        self.right_distance = right

    def control_loop(self):
        cmd = Twist()
        if self.front_distance < 0.5:
            cmd.linear.x = 0.0
        # Turn toward the side with more space
            if self.left_distance > self.right_distance:
                cmd.angular.z = 0.5  # turn left
            else:
                cmd.angular.z = -0.5  # turn right
        else:
            cmd.linear.x = 0.1
            cmd.angular.z = 0.0
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
