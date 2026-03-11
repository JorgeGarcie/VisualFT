"""rdk_utils.py — Shared utilities for Flexiv RDK nodes.

Eliminates duplicated robot init, quaternion conversion, and wrench publishing
patterns across rdk_cartesian_bridge, scan_node, and robot_publisher.
"""

import time
import flexivrdk
from geometry_msgs.msg import PoseStamped, WrenchStamped


def init_robot(robot_sn: str, logger=None) -> flexivrdk.Robot:
    """Connect to Flexiv robot, clear faults, enable, wait for operational.

    Args:
        robot_sn: Robot serial number (e.g. 'Rizon4-062174').
        logger: Optional ROS2 logger. Falls back to print() if None.

    Returns:
        Connected and enabled flexivrdk.Robot instance.
    """
    def log(msg):
        if logger:
            logger.info(msg)
        else:
            print(msg)

    def warn(msg):
        if logger:
            logger.warn(msg)
        else:
            print(f"WARN: {msg}")

    robot = flexivrdk.Robot(robot_sn)

    if robot.fault():
        warn("Fault detected — clearing...")
        if not robot.ClearFault():
            raise RuntimeError("Cannot clear robot fault")
        log("Fault cleared")

    log("Enabling robot...")
    robot.Enable()

    log("Waiting for robot to become operational...")
    while not robot.operational():
        time.sleep(1)
    log("Robot operational")

    return robot


def rdk_pose_to_ros(tcp_pose) -> tuple:
    """Convert RDK tcp_pose [x,y,z,qw,qx,qy,qz] to ROS2 position + orientation.

    Returns:
        (pos, ori) where pos=(x,y,z) and ori=(x,y,z,w) in ROS2 convention.
    """
    return (
        (float(tcp_pose[0]), float(tcp_pose[1]), float(tcp_pose[2])),
        (float(tcp_pose[4]), float(tcp_pose[5]), float(tcp_pose[6]), float(tcp_pose[3])),
    )


def ros_orientation_to_rdk(orientation) -> list:
    """Convert ROS2 orientation (x,y,z,w) to RDK quaternion [qw,qx,qy,qz].

    Args:
        orientation: geometry_msgs Quaternion with .x, .y, .z, .w attributes.

    Returns:
        [qw, qx, qy, qz] list.
    """
    return [orientation.w, orientation.x, orientation.y, orientation.z]


def build_pose_msg(tcp_pose, stamp, frame_id='world') -> PoseStamped:
    """Build a PoseStamped from RDK tcp_pose [x,y,z,qw,qx,qy,qz].

    Handles the quaternion reorder (RDK qw-first → ROS2 w-last).
    """
    pos, ori = rdk_pose_to_ros(tcp_pose)

    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = pos[0]
    msg.pose.position.y = pos[1]
    msg.pose.position.z = pos[2]
    msg.pose.orientation.x = ori[0]
    msg.pose.orientation.y = ori[1]
    msg.pose.orientation.z = ori[2]
    msg.pose.orientation.w = ori[3]
    return msg


def build_wrench_msg(wrench_array, stamp, frame_id='flexiv_tcp') -> WrenchStamped:
    """Build a WrenchStamped from RDK wrench [fx,fy,fz,tx,ty,tz]."""
    msg = WrenchStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.wrench.force.x = float(wrench_array[0])
    msg.wrench.force.y = float(wrench_array[1])
    msg.wrench.force.z = float(wrench_array[2])
    msg.wrench.torque.x = float(wrench_array[3])
    msg.wrench.torque.y = float(wrench_array[4])
    msg.wrench.torque.z = float(wrench_array[5])
    return msg


def euler_to_rdk_quat(rx_deg: float, ry_deg: float, rz_deg: float) -> list:
    """Euler XYZ degrees → RDK quaternion [qw, qx, qy, qz].

    Uses scipy extrinsic XYZ convention ('xyz' lowercase).
    """
    from scipy.spatial.transform import Rotation as R
    q = R.from_euler('xyz', [rx_deg, ry_deg, rz_deg], degrees=True).as_quat()
    return [q[3], q[0], q[1], q[2]]  # scipy [x,y,z,w] → RDK [qw,qx,qy,qz]  # noqa
