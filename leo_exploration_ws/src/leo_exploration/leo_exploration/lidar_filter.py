#!/usr/bin/env python3
"""
Lidar arc filter for Leo Rover Exploration v3.0

This standalone node subscribes to /scan and publishes /scan_filtered,
masking rays outside the forward ±60° (120° total) FOV to `inf`.
This ensures that SLAM Toolbox only sees forward data and starts
mapping immediately on launch without waiting for the explorer node.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')
        self.declare_parameter('lidar_fov_deg', 120.0)
        
        fov_deg = self.get_parameter('lidar_fov_deg').value
        self.half_fov = math.radians(fov_deg / 2.0)
        
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        
        self.get_logger().info(f"Lidar Filter active: masking out-of-bounds to forward {fov_deg}°")

    def scan_cb(self, scan: LaserScan) -> None:
        n = len(scan.ranges)
        if n == 0:
            return

        angles = (
            np.arange(n, dtype=np.float32) * scan.angle_increment
            + scan.angle_min
        )

        keep = np.abs(angles) <= self.half_fov
        ranges = np.array(scan.ranges, dtype=np.float32)
        filtered = np.where(keep, ranges, float("inf"))

        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = filtered.tolist()
        
        if scan.intensities:
            intensities = np.array(scan.intensities, dtype=np.float32)
            out.intensities = np.where(keep, intensities, 0.0).tolist()
            
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
