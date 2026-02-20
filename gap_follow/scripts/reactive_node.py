#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveFollowGap(Node):
    def __init__(self):
        super().__init__('reactive_node')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)

        # Tuned parameters
        self.max_range = 5
        self.bubble_radius = 18
        self.smoothing_window = 3

    # ---------------------------------------------------------
    def preprocess_lidar(self, ranges):
        ranges = np.array(ranges)

        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = self.max_range
        ranges = np.clip(ranges, 0, self.max_range)

        kernel = np.ones(self.smoothing_window) / self.smoothing_window
        ranges = np.convolve(ranges, kernel, mode='same')

        return ranges

    # ---------------------------------------------------------
    def find_max_gap(self, free_space):
        max_len = 0
        max_start = 0
        curr_start = 0
        curr_len = 0

        for i in range(len(free_space)):
            if free_space[i] > 0:
                if curr_len == 0:
                    curr_start = i
                curr_len += 1
            else:
                if curr_len > max_len:
                    max_len = curr_len
                    max_start = curr_start
                curr_len = 0

        if curr_len > max_len:
            max_len = curr_len
            max_start = curr_start

        return max_start, max_start + max_len

    # ---------------------------------------------------------
    def find_best_point(self, start_i, end_i, ranges):
        if end_i - start_i <= 0:
            return start_i

        # Pure center of gap
        return (start_i + end_i) // 2
        # return start_i + np.argmax(ranges[start_i:end_i])

    # ---------------------------------------------------------
    def lidar_callback(self, data):

        ranges = self.preprocess_lidar(data.ranges)
        total_points = len(ranges)

        # Use only front 180°
        start_index = total_points // 4
        end_index = 3 * total_points // 4
        ranges[:start_index] = 0
        ranges[end_index:] = 0

        # ---- Find closest obstacle in FRONT ONLY (ignore zeros)
        front = ranges[start_index:end_index]
        nonzero = np.where(front > 0)[0]

        if len(nonzero) == 0:
            return

        closest_front = nonzero[np.argmin(front[nonzero])]
        closest_index = closest_front + start_index

        # ---- Create safety bubble
        bubble_start = max(0, closest_index - self.bubble_radius)
        bubble_end = min(total_points, closest_index + self.bubble_radius)
        ranges[bubble_start:bubble_end] = 0

        # ---- Find largest gap
        gap_start, gap_end = self.find_max_gap(ranges)

        if gap_end - gap_start <= 0:
            return

        # ---- Pick best direction
        best_index = self.find_best_point(gap_start, gap_end, ranges)

        # ---- Center steering properly
        mid_index = total_points // 2
        
        steer_gain = 0.985

        angle = steer_gain*(best_index - mid_index) * data.angle_increment

        # ---- Slower turn speeds
        steering_abs = abs(angle)

        if steering_abs < np.radians(5):
            speed = 2.7
        elif steering_abs < np.radians(15):
            speed = 0.7
        elif steering_abs < np.radians(25):
            speed = 0.55
        else:
            speed = 0.325

        # ---- Publish
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("Reactive Follow Gap Initialized")
    node = ReactiveFollowGap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


