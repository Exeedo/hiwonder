#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import Future
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import signal
from time import time
import argparse
import math

class StraightLineMover(Node):
    def __init__(
        self,
        distance=0.2,
        lin_speed=0.1,
        minimum_duration=1,
        reverse=False,
        **kwargs
    ):
        super().__init__('straight_line_mover')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.get_logger().info("Starting ...")
        self.initial_distance = (0.0, 0.0, 0.0)
        self.initialized = False
        self.current_x = float(0.0)
        self.current_y = float(0.0)
        self.current_theta = float(0.0)
        self.distance = float(distance)
        self.lin_speed = float(lin_speed)
        self.timer = self.create_timer(0.1, self.move_reverse if reverse else self.move_forward)
        self.time = time()
        self.minimum_duration = minimum_duration
        self.twist = None
        self.future = Future()
        signal.signal(signal.SIGINT, self.shutdown)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        siny_cosp = 2 * (qz * qw)
        self.current_theta = math.degrees(math.atan2(siny_cosp, 1 - 2 * qz**2))  # Yaw from quaternion
        if not self.initialized:
            self.initial_distance = (self.current_x, self.current_y, self.current_theta)
            self.initialized = True
            self.get_logger().info(f"Initialized to {self.get_position_string()}")

        # Limit to a single log every minimum_duration seconds
        if time() - self.time < self.minimum_duration:
            return
        self.time = time()
        self.show_current_position()

    def show_current_position(self):
        self.get_logger().info(self.get_position_string())

    def get_position_string(self):
        return f"Position: x={self.current_x:.3f}, y={self.current_y:.3f}, theta = {self.current_theta:.3f}"

    def force_stop(self):
        self.get_logger().info(f'Forcefully stopping the robot')
        self.pub.publish(Twist())
        self.future.set_result(True)
        self.timer.cancel()

    def shutdown(self, signum, frame):
        self.force_stop()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'shutdown')
        rclpy.shutdown()

    def move_forward(self):
        # Forward for 'self.distance'
        twist = Twist()
        if self.current_x - self.initial_distance[0] < self.distance:
            twist.linear.x = self.lin_speed
            if twist != self.twist:
                self.get_logger().info(f'Moving with velocity vx = {self.lin_speed}')
                self.twist = twist
        else:
            self.show_current_position()
            self.get_logger().info(f'Line of {self.distance} complete!')
            # Exit gracefully
            self.future.set_result(True)
            self.timer.cancel()

        self.pub.publish(twist)

    def move_reverse(self):
        twist = Twist()
        # Reverse for 'self.distance'
        if self.current_x - self.initial_distance[0] > -self.distance:
            twist.linear.x = -self.lin_speed
            if twist != self.twist:
                self.get_logger().info(f'Moving with velocity vx = {-self.lin_speed}')
                self.twist = twist
        else:
            self.show_current_position()
            self.get_logger().info(f'Line of {self.distance} complete!')
            # Exit gracefully
            self.future.set_result(True)
            self.timer.cancel()

        self.pub.publish(twist)


def main(args=None):
    rclpy.init()
    node = StraightLineMover(**args.__dict__)
    try:
        rclpy.spin_until_future_complete(node, node.future)  # Blocks until future done
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def parse_args():
    parser = argparse.ArgumentParser(description="Move robot in a straight line")
    parser.add_argument("-d", "--distance", default=0.2, type=float, help="Distance to move in meters")
    parser.add_argument("-v", "--lin_speed", default=0.1, type=float, help="Linear speed in m/s")
    parser.add_argument("-r", "--reverse", action='store_true', help="Whether to run in reverse")
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_args()
    print(args.__dict__)
    main(args)
