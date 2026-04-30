#!/usr/bin/env python3
"""Block until /coinft/status publishes READY, then exit 0.

Used as a launch gate: recording starts only after CoinFT offset is done.

Usage:
    ros2 run visionft wait_for_coinft          # blocks until READY
    ros2 run visionft wait_for_coinft --timeout 30  # fail after 30s
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WaitForCoinFT(Node):
    def __init__(self, timeout_s: float):
        super().__init__('wait_for_coinft')
        self._ready = False
        self.create_subscription(String, '/coinft/status', self._on_status, 10)
        if timeout_s > 0:
            self.create_timer(timeout_s, self._on_timeout)
        self.get_logger().info("Waiting for CoinFT READY...")

    def _on_status(self, msg: String):
        if msg.data == 'READY' and not self._ready:
            self._ready = True
            self.get_logger().info("CoinFT is READY")
            raise SystemExit(0)

    def _on_timeout(self):
        if not self._ready:
            self.get_logger().error("Timeout waiting for CoinFT READY")
            raise SystemExit(1)


def main(args=None):
    rclpy.init(args=args)
    timeout = 60.0  # default 60s
    if '--timeout' in sys.argv:
        idx = sys.argv.index('--timeout')
        if idx + 1 < len(sys.argv):
            timeout = float(sys.argv[idx + 1])

    node = WaitForCoinFT(timeout)
    try:
        rclpy.spin(node)
    except SystemExit as e:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(e.code)


if __name__ == '__main__':
    main()
