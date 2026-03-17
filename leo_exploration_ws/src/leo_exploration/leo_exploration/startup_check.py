#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
import sys

class StartupCheckNode(Node):
    def __init__(self):
        super().__init__('startup_check')
        self.scan_received = False
        self.last_status = None
        
        # Subscribe to scan
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(1.0, self.check_status)
        self.get_logger().info(
            "Waiting for /scan, /odometry/filtered, and odom -> base_link TF..."
        )

    def scan_callback(self, msg):
        self.scan_received = True

    def check_status(self):
        scan_ready = self.scan_received
        ekf_ready = self.count_publishers('/odometry/filtered') > 0
        tf_ready = False
        try:
            self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            tf_ready = True
        except Exception:
            pass

        status = (
            f"/scan={'ok' if scan_ready else 'waiting'} | "
            f"EKF={'ok' if ekf_ready else 'waiting'} | "
            f"TF odom->base_link={'ok' if tf_ready else 'waiting'}"
        )
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

        if scan_ready and ekf_ready and tf_ready:
            self.get_logger().info(
                "\033[92mSystem Ready! /scan, EKF, and TF chain are active.\033[0m"
            )
            sys.exit(0)

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
