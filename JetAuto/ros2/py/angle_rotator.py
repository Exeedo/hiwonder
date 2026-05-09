#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import Future
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi, fabs

from time import time
import argparse
import math

class Rotator(Node):
    def __init__(
        self,
        angle=90,  # in degrees
        ang_speed=0.5,
        minimum_duration=1,
        reverse=False,
        **kwargs
    ):
        super().__init__('rotator')
        angle_rad = angle * pi / 180.0
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.get_logger().info("Starting ...")
        self.initial_distance = (0.0, 0.0, 0.0)
        self.initialized = False
        self.current_x = float(0.0)
        self.current_y = float(0.0)
        self.current_theta = float(0.0)
        self.angle = float(angle_rad)
        self.ang_speed = float(ang_speed)
        self.timer = self.create_timer(0.1, self.rotate_cw if reverse else self.rotate_ccw)
        self.time = time()
        self.minimum_duration = minimum_duration
        self.running = True
        self.twist = None
        self.future = Future()

    def get_theta(self, msg):
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2 * (qz * qw)
        return math.atan2(siny_cosp, 1 - 2 * qz**2)  # Yaw from quaternion


    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = self.get_theta(msg)
        if not self.initialized:
            self.initial_distance = (self.current_x, self.current_y, self.current_theta)
            self.initialized = True
            self.get_logger().info(f"Initialized to {self.initial_distance}")

        # Limit to a single log every minimum_duration seconds
        if time() - self.time < self.minimum_duration:
            return
        self.time = time()
        self.get_logger().info(f"Position: x={self.current_x:.3f}, y={self.current_y:.3f}, theta={self.current_theta:.3f}")

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def angle_diff(self, angle1, angle2):
        return self.normalize_angle(angle1 - angle2)

    def rotate_ccw(self):
        # Forward for 'self.angle'
        twist = Twist()
        if self.angle_diff(self.current_theta, self.initial_distance[2]) < self.angle:
            twist.angular.z = self.ang_speed
            if twist != self.twist:
                self.get_logger().info(f'Rotating with velocity vz = {self.ang_speed}')
                self.twist = twist
        else:
            twist.angular.z = 0.0
            self.get_logger().info(f'Angle of {self.angle * 180.0 / pi} completed!')
            # Exit gracefully
            self.future.set_result(True)
            self.timer.cancel()

        self.pub.publish(twist)

    def rotate_cw(self):
        twist = Twist()
        # Reverse for 'self.angle'
        if self.angle_diff(self.initial_distance[2], self.current_theta) < self.angle:
            twist.angular.z = -self.ang_speed
            if twist != self.twist:
                self.get_logger().info(f'Rotating with velocity vz = {-self.ang_speed}')
                self.twist = twist
        else:
            twist.angular.z = 0.0
            self.get_logger().info(f'Angle of {self.angle * 180.0 / pi} completed!')
            # Exit gracefully
            self.future.set_result(True)
            self.timer.cancel()

        self.pub.publish(twist)


def main(args=None):
    rclpy.init()
    node = Rotator(**args.__dict__)
    try:
        rclpy.spin_until_future_complete(node, node.future)  # Blocks until future done
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def parse_args():
    parser = argparse.ArgumentParser(description="Rotate robot in-place")
    parser.add_argument("-a", "--angle", default=90, type=float, help="Angle to rotate in degrees")
    parser.add_argument("-v", "--ang_speed", default=0.5, type=float, help="Angular speed in rotation/s")
    parser.add_argument("-r", "--reverse", action='store_true', help="Whether to rotate in reverse (CW)")
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_args()
    print(args.__dict__)
    main(args)
