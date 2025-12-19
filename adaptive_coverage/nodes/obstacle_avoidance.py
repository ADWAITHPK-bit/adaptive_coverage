#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Subscriber to LiDAR
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher to cmd_vel
        self.pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Initialize distances
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Obstacle Avoidance Node Started")

    def scan_callback(self, msg):
        # Parameters
        window_size = 10  # number of laser points to consider for front/side
        min_range = 0.05  # ignore too close readings (robot bumper)
        max_range = 10.0  # ignore too far readings

        total_points = len(msg.ranges)
        center = total_points // 2

        # Front slice
        front_ranges = msg.ranges[center - window_size : center + window_size + 1]
        front_ranges = [r for r in front_ranges if min_range < r < max_range]
        self.front_distance = min(front_ranges, default=float('inf'))

        # Left slice
        left_ranges = msg.ranges[center + 1 : center + 1 + window_size]
        left_ranges = [r for r in left_ranges if min_range < r < max_range]
        self.left_distance = min(left_ranges, default=float('inf'))

        # Right slice
        right_ranges = msg.ranges[center - window_size : center]
        right_ranges = [r for r in right_ranges if min_range < r < max_range]
        self.right_distance = min(right_ranges, default=float('inf'))

        # Debug info
        self.get_logger().info(
            f"Front: {self.front_distance:.2f} | Left: {self.left_distance:.2f} | Right: {self.right_distance:.2f}"
        )

    def control_loop(self):
        cmd = Twist()
        safe_distance = 0.5  

        if self.front_distance < safe_distance:
            
            cmd.linear.x = 0.0

            
            if self.left_distance > self.right_distance:
                cmd.angular.z = 0.5  
            else:
                cmd.angular.z = -0.5  
        else:
            
            cmd.linear.x = 0.2
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
