#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import sys

class StartupCheckNode(Node):
    def __init__(self):
        super().__init__('startup_check')
        self.scan_received = False
        
        # Subscribe to scan
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.check_status)
        self.get_logger().info("Waiting for /scan and odom -> base_footprint TF...")

    def scan_callback(self, msg):
        self.scan_received = True

    def check_status(self):
        tf_ready = False
        try:
            # Check for transform
            self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            tf_ready = True
        except Exception:
            pass
            
        if self.scan_received and tf_ready:
            self.get_logger().info("\033[92mSystem Ready! /scan and TF are active.\033[0m")
            sys.exit(0)
            
        elif not self.scan_received and tf_ready:
            self.get_logger().info("Waiting for /scan data...")
        elif self.scan_received and not tf_ready:
            self.get_logger().info("Waiting for odom -> base_footprint TF...")

def main(args=None):
    rclpy.init(args=args)
    node = StartupCheckNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
