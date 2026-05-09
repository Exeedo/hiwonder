#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

from textwrap import dedent
from time import time


class OdomReader(Node):
    def __init__(self, minimum_duration=1):
        super().__init__('odom_reader')
        self.sub1 = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.sub2 = self.create_subscription(Odometry, '/odom_raw', self.odom_raw_callback, 10)
        self.pub = self.create_publisher(Pose2D, '/set_odom', 10)
        self.get_logger().info("Starting ...")
        self.reset_distances()
        self.time1 = time()
        self.time2 = time()
        self.minimum_duration = minimum_duration

    def reset_distances(self):
        msg = Pose2D()
        msg.x = 0.0
        msg.y = 0.0
        msg.theta = 0.0
        self.get_logger().info('Publishing odom reset to (0,0,0)')
        self.pub.publish(msg)

    def odom_callback(self, msg):
        # Limit to a single callback every minimum_duration seconds
        if time() - self.time1 < self.minimum_duration:
            return
        self.time1 = time()

        # Position Info
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        # Twist Info
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.angular.z

        self.get_logger().info(dedent(f"""
            Position:
                x = {x:.3f}, y = {y:.3f}, z = {z:.3f}, w = {w:.3f}
            Twist:
                x = {vx:.3f}, y = {vy:.3f}, z = {vz:.3f}"""))

    def odom_raw_callback(self, msg):
        # Limit to a single callback every minimum_duration seconds
        if time() - self.time2 < self.minimum_duration:
            return
        self.time2 = time()
        self.get_logger().info(f"Odom Raw:\n{msg}")

    def destroy_node(self):
        self.get_logger().info("Exiting ...")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
