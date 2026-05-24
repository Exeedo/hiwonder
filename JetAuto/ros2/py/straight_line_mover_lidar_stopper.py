#!/usr/bin/env python3
import numpy as np
from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.node import Node
from rclpy import Future
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry

import signal
from time import time
import argparse
import math

CAR_WIDTH = 0.4  # meter
CAR_HALF_WIDTH = CAR_WIDTH / 2

class StraightLineMoverLidarStopper(Node):
    def __init__(
        self,
        distance=0.5,
        lin_speed=0.1,
        update_duration=1,
        threshold=0.6,
        **kwargs
    ):
        super().__init__("StraightLineMover_LidarStopper")
        self.get_logger().info("Starting ...")
        self.pub = self.create_publisher(Twist, '/controller/cmd_vel', 1)  # 底盘控制(chassis control)
        self.odom_sub = None  # self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_pub = self.create_publisher(Pose2D, '/set_odom', 10)
        self.initial_distance = (0.0, 0.0, 0.0)
        self.initialized = False
        self.current_x = float(0.0)
        self.current_y = float(0.0)
        self.current_theta = float(0.0)
        self.distance = float(distance)
        self.lin_speed = float(lin_speed)
        self.timer = self.create_timer(0.1, self.move_forward)
        self.update_duration = update_duration
        self.running = True
        self.twist = None
        self.future = Future()
        if 0.3 <= threshold <= 1.5:  # Safety measure
            self.threshold = float(threshold)  # meters
        else:  # Default for safety
            self.threshold = float(0.6)
        self.threshold = round(self.threshold, 2)
        self.get_logger().info(f'Set threshold to {self.threshold:.2f}m')
        self.odom_time = time()
        self.lidar_time = self.odom_time
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)  # 订阅雷达数据(subscribe to Lidar data)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        signal.signal(signal.SIGINT, self.shutdown)

    def reset_pose(self):
        msg = Pose2D()
        msg.x = float(0.0)
        msg.y = float(0.0)
        msg.theta = float(0.0)
        self.odom_pub.publish(msg)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

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

        # Limit to a single log every update_duration seconds
        if time() - self.odom_time < self.update_duration:
            return
        self.odom_time = time()
        self.show_current_position()

    def show_current_position(self):
        self.get_logger().info(self.get_position_string())

    def get_position_string(self):
        return f"Position: x={self.current_x:.3f}, y={self.current_y:.3f}, theta = {self.current_theta:.3f}"

    def force_stop(self):
        self.running = False
        self.show_current_position()
        self.get_logger().info(f'Forcefully stopping the robot')
        self.pub.publish(Twist())
        self.future.set_result(True)
        self.timer.cancel()

    def shutdown(self, signum, frame):
        self.running = False
        self.force_stop()
        self.get_logger().info('\033[1;32m%s\033[0m' % 'shutdown')
        rclpy.shutdown()

    def move_forward(self):
        # Override this function only to stop when reaching required distance, 
        # but not to move it. Movement is controlled by LidarStopper
        if not self.running:
            return
        if self.current_x - self.initial_distance[0] > self.distance:
            twist = Twist()
            twist.linear.x = 0.0
            self.get_logger().info(f'Line of {self.distance}m complete!')
            self.running = False
            self.pub.publish(twist)
            self.future.set_result(True)
            self.timer.cancel()

    def lidar_callback(self, lidar_data):
        if not self.running:
            return
        current_time = time()
        if current_time - self.lidar_time < 0.2:  # update every 0.2 seconds
            return
        self.lidar_time = current_time
        
        # 雷达订阅回调(Lidar subscription callback)
        twist = Twist()
        # 数据大小 = 扫描角度/每扫描一次增加的角度(data size= scanning angle/ the increased angle per scan)
        r_vector = lidar_data.ranges  # radial distance in meters
        angle_vector = np.linspace(0, 2*np.pi, len(r_vector))  # angle in radians
        theta_vector = angle_vector + np.pi / 2  # shifted angle
        x_vector = r_vector * np.cos(theta_vector)
        y_vector = r_vector * np.sin(theta_vector)
        mask_front = -CAR_HALF_WIDTH <= x_vector
        mask_front &= x_vector <= CAR_HALF_WIDTH
        filtered_x = x_vector[mask_front]
        filtered_y = y_vector[mask_front]
        min_dist = filtered_y.min()
        if min_dist <= self.threshold:
            # Stop the robot here
            self.get_logger().info(f"Obstacle found at distance {min_dist:.3f}, stopping the robot")
            twist.linear.x = 0.0
        else:
            twist.linear.x = self.lin_speed
        self.pub.publish(twist)


def main(args=None):
    rclpy.init()
    node = StraightLineMoverLidarStopper(**args.__dict__)
    try:
        node.reset_pose()
        rclpy.spin_until_future_complete(node, node.future)  # Blocks until future done
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

def parse_args():
    parser = argparse.ArgumentParser(description="Move robot in a straight line")
    parser.add_argument("-d", "--distance", default=0.5, type=float, help="Distance to move in meters")
    parser.add_argument("-v", "--lin_speed", default=0.1, type=float, help="Linear speed in m/s")
    parser.add_argument("-t", "--threshold", default=0.6, type=float, help="Threshold for Lidar in m")
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_args()
    print(args.__dict__)
    main(args)
