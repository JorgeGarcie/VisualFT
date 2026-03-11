#!/usr/bin/env python3

"""rdk_cartesian_bridge.py

ROS2 node that owns the exclusive RDK connection and streams Cartesian pose
targets to the robot via NRT_CARTESIAN_MOTION_FORCE mode.

Subscribes:
  /rdk/cartesian_target  (geometry_msgs/PoseStamped)  metres + quaternion

Publishes:
  /rdk/tcp_pose          (geometry_msgs/PoseStamped)
  /rdk/wrench            (geometry_msgs/WrenchStamped)

The control loop runs at `control_rate_hz` (default 50 Hz) and continuously
resends the last received target — the robot holds position if no new command
arrives.  On shutdown, robot.Stop() is called.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
import flexivrdk
from visionft.rdk_utils import init_robot, build_pose_msg, build_wrench_msg


class RDKCartesianBridge(Node):
    def __init__(self):
        super().__init__('rdk_cartesian_bridge')

        # Parameters
        self.declare_parameter('robot_sn',        'Rizon4-062174')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('max_linear_vel',  0.05)   # m/s
        self.declare_parameter('max_angular_vel', 0.5)    # rad/s
        self.declare_parameter('max_linear_acc',  0.5)    # m/s²
        self.declare_parameter('max_angular_acc', 1.0)    # rad/s²

        robot_sn  = self.get_parameter('robot_sn').value
        rate_hz   = self.get_parameter('control_rate_hz').value
        self._mlv = self.get_parameter('max_linear_vel').value
        self._mav = self.get_parameter('max_angular_vel').value
        self._mla = self.get_parameter('max_linear_acc').value
        self._maa = self.get_parameter('max_angular_acc').value

        # Publishers
        self._tcp_pub    = self.create_publisher(PoseStamped,   '/rdk/tcp_pose', 10)
        self._wrench_pub = self.create_publisher(WrenchStamped, '/rdk/wrench',   10)

        # Subscriber — incoming Cartesian targets (metres + quaternion)
        self.create_subscription(
            PoseStamped, '/rdk/cartesian_target', self._target_cb, 10)

        # Connect and configure robot (blocks until operational)
        self._robot = self._init_robot(robot_sn)

        # Hold current pose as initial target  [x, y, z, qw, qx, qy, qz]
        self._target = list(self._robot.states().tcp_pose)
        self.get_logger().info(
            f"Holding initial pose: {[round(v, 4) for v in self._target]}")

        # Start control loop
        self.create_timer(1.0 / rate_hz, self._control_loop)
        self.get_logger().info(f"Control loop running at {rate_hz} Hz")

    # ------------------------------------------------------------------
    def _init_robot(self, robot_sn):
        robot = init_robot(robot_sn, logger=self.get_logger())
        robot.SwitchMode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
        robot.SetForceControlAxis([False] * 6)
        return robot

    # ------------------------------------------------------------------
    def _target_cb(self, msg: PoseStamped):
        """Update internal target from incoming PoseStamped.
        ROS2 quaternion convention: (x, y, z, w)
        RDK pose convention:        [x, y, z, qw, qx, qy, qz]
        """
        p = msg.pose.position
        o = msg.pose.orientation
        self._target = [p.x, p.y, p.z, o.w, o.x, o.y, o.z]

    # ------------------------------------------------------------------
    def _control_loop(self):
        try:
            if self._robot.fault():
                self.get_logger().error("Robot fault detected — holding last target")
                return

            # Send latest target (holds if no new command)
            self._robot.SendCartesianMotionForce(
                self._target, [0.0] * 6,
                self._mlv, self._mav, self._mla, self._maa)

            # Read and publish current state
            states = self._robot.states()
            now = self.get_clock().now().to_msg()
            self._tcp_pub.publish(build_pose_msg(states.tcp_pose, now, 'world'))
            self._wrench_pub.publish(build_wrench_msg(states.ext_wrench_in_tcp, now, 'flexiv_tcp'))

        except Exception as e:
            self.get_logger().error(f"Control loop error: {e}")

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("Stopping robot...")
        try:
            self._robot.Stop()
        except Exception as e:
            self.get_logger().warning(f"Error stopping robot: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RDKCartesianBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # normal shutdown
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
