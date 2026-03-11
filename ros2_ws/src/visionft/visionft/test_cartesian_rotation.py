#!/usr/bin/env python3

"""test_cartesian_rotation.py

Verification test for rdk_cartesian_bridge.

Steps:
  1. Read current TCP pose from /rdk/tcp_pose
  2. Publish a target = current pose + 2 deg around Z
  3. Monitor /rdk/tcp_pose for 8 s, printing position + orientation
  4. Publish original pose to return home
  5. Monitor for another 8 s then exit

Run AFTER rdk_cartesian_bridge is running:
  ros2 run visionft rdk_cartesian_bridge
  ros2 run visionft test_cartesian_rotation
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R


DEGREES = 2.0          # rotation amount
HOLD_SECONDS = 8.0     # time to hold each pose


class RotationTest(Node):
    def __init__(self, degrees: float):
        super().__init__('test_cartesian_rotation')
        self._degrees = degrees
        self._origin: PoseStamped | None = None
        self._state = 'waiting'     # waiting → rotating → returning → done
        self._elapsed = 0.0

        self._pub = self.create_publisher(PoseStamped, '/rdk/cartesian_target', 10)
        self.create_subscription(PoseStamped, '/rdk/tcp_pose', self._pose_cb, 10)

        # 10 Hz monitoring timer
        self.create_timer(0.1, self._monitor)
        self.get_logger().info(f"Waiting for /rdk/tcp_pose ...")

    # ------------------------------------------------------------------
    def _pose_cb(self, msg: PoseStamped):
        if self._origin is None:
            self._origin = msg
            self.get_logger().info(
                f"Current pose: xyz=[{msg.pose.position.x:.4f}, "
                f"{msg.pose.position.y:.4f}, {msg.pose.position.z:.4f}]")
            self._send_rotated(self._degrees)

    # ------------------------------------------------------------------
    def _send_rotated(self, deg: float):
        o = self._origin.pose.orientation
        q_cur = R.from_quat([o.x, o.y, o.z, o.w])
        q_new = q_cur * R.from_euler('z', deg, degrees=True)
        q = q_new.as_quat()  # [x, y, z, w]

        target = PoseStamped()
        target.header.stamp    = self.get_clock().now().to_msg()
        target.header.frame_id = 'world'
        target.pose.position   = self._origin.pose.position
        target.pose.orientation.x = float(q[0])
        target.pose.orientation.y = float(q[1])
        target.pose.orientation.z = float(q[2])
        target.pose.orientation.w = float(q[3])

        self.get_logger().info(f"Publishing target: +{deg:.1f} deg around Z")
        self._pub.publish(target)
        self._state = 'rotating'
        self._elapsed = 0.0

    # ------------------------------------------------------------------
    def _monitor(self):
        if self._origin is None or self._state == 'done':
            return

        self._elapsed += 0.1

        # Print current pose every 0.5 s
        if round(self._elapsed * 10) % 5 == 0:
            pass  # pose printed in _pose_cb on update; sub handles this

        if self._state == 'rotating' and self._elapsed >= HOLD_SECONDS:
            self.get_logger().info("Returning to original pose...")
            self._pub.publish(self._origin)
            self._state = 'returning'
            self._elapsed = 0.0

        elif self._state == 'returning' and self._elapsed >= HOLD_SECONDS:
            self.get_logger().info("Done. Round-trip complete.")
            self._state = 'done'
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    degrees = float(sys.argv[1]) if len(sys.argv) > 1 else DEGREES
    node = RotationTest(degrees)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # normal shutdown
    finally:
        try:
            node.destroy_node()
        except Exception as e:
            print(f"Warning during cleanup: {e}")


if __name__ == '__main__':
    main()
