#!/usr/bin/env python3
import os
import numpy as np
import sdk.pid as pid
from sensor_msgs.msg import LaserScan

import rclpy
from rclpy.node import Node

import argparse
import math
from time import time

CAR_WIDTH = 0.4  # meter
MAX_SCAN_ANGLE = 240  # 激光的扫描角度,去掉总是被遮挡的部分degree(the scanning angle of Lidar. The covered part is always eliminated)
DUMP_FILE = "/home/ubuntu/dump/file.txt"

class LidarReader(Node):
    def __init__(
        self,
        scan_angle = 90,  # degrees
    ):
        super().__init__("Lidar_Reader")
        self.running = False  # ADDED
        self.scan_angle = math.radians(scan_angle)   # radians
        self.angle_data = []
        self.time = time()
        # pid参数
        self.pid_yaw = pid.PID(1.6, 0, 0.16)
        self.pid_dist = pid.PID(1.7, 0, 0.16)
        self.lidar_type = os.environ.get('LIDAR_TYPE')
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 1)  # 订阅雷达数据(subscribe to Lidar data)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')
        self.min_dist_left = 0
        self.min_dist_right = 0

    def lidar_callback(self, lidar_data):
        # if not self.running:  # ADDED
        #     return

        # 雷达订阅回调(Lidar subscription callback)
        # 数据大小 = 扫描角度/每扫描一次增加的角度(data size= scanning angle/ the increased angle per scan)
        max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
        left_ranges = lidar_data.ranges[:max_index]  # 左半边数据 (the left data)
        right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据 (the right data)

        current_time = time()
        if current_time - self.time > 1:  # update every X seconds
            self.time = current_time
            with open(DUMP_FILE, 'a') as fh:
                fh.write(','.join(map(str, lidar_data.ranges)))
                fh.write('\n')
            # 根据设定取数据 (get the data according to the settings)
            half_angle = self.scan_angle / 2
            angle_index = int(half_angle / lidar_data.angle_increment + 0.50)
            left_range = np.array(left_ranges[:angle_index])
            right_range = np.array(right_ranges[:angle_index])
            # Get the minimum distances from left and right
            left_nonzero = left_range.nonzero()
            right_nonzero = right_range.nonzero()
            left_nonan = ~np.isnan(left_range[left_nonzero])
            right_nonan = ~np.isnan(right_range[right_nonzero])
            min_dist_left_ = left_range[left_nonzero][left_nonan]
            min_dist_right_ = right_range[right_nonzero][right_nonan]
            min_dist_left = round(min_dist_left_.min(), 3)
            min_dist_right = round(min_dist_right_.min(), 3)
            if abs(min_dist_left - self.min_dist_left) > 1e-6 or \
                abs(min_dist_right - self.min_dist_right) > 1e-6:
                self.min_dist_left = min_dist_left
                self.min_dist_right = min_dist_right
                print(f"Angle increment: {lidar_data.angle_increment}, {max_index = }, {angle_index = }")
                self.get_logger().info(f"Min distance at LEFT is {min_dist_left:.3f}")
                self.get_logger().info(f"Min distance at RIGHT is {min_dist_right:.3f}")

def main(args):
    rclpy.init()
    node = LidarReader(**args.__dict__)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        node.destroy_node()

def parse_args():
    parser = argparse.ArgumentParser(description="Get the reading of the Lidar")
    parser.add_argument("-a", "--scan_angle", default=90, type=float, help="Scan angle in degrees")
    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_args()
    print(args.__dict__)
    main(args)
