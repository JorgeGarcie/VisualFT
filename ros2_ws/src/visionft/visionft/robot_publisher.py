#!/usr/bin/env python3

"""robot_publisher.py

Reads Flexiv robot states and publishes ext_wrench_in_tcp as WrenchStamped.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
import spdlog
from visionft.rdk_utils import init_robot, build_pose_msg, build_wrench_msg


class FlexivWrenchPublisher(Node):
    def __init__(self):
        super().__init__('flexiv_wrench_publisher')
        
        # Declare parameters
        self.declare_parameter('robot_sn', 'Rizon4-062174')
        self.declare_parameter('frame_id', 'flexiv_tcp')
        self.declare_parameter('publish_rate', 100.0)  # Hz
        
        robot_sn = self.get_parameter('robot_sn').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Create publisher
        self.wrench_pub = self.create_publisher(WrenchStamped, '/flexiv/wrench', 10)
        self.tcp_pose_pub = self.create_publisher(PoseStamped, '/flexiv/tcp_pose', 10)
        
        # Setup logger
        self.logger = spdlog.ConsoleLogger("FlexivPublisher")
        
        # Initialize robot
        self.robot = None
        self.initialize_robot(robot_sn)
        
        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_wrench)
        
        self.get_logger().info(f"Publishing wrench data to /flexiv/wrench at {publish_rate} Hz")
        self.get_logger().info(f"Publishing TCP pose data to /flexiv/tcp_pose at {publish_rate} Hz")
    
    def initialize_robot(self, robot_sn):
        """Initialize connection to Flexiv robot."""
        self.robot = init_robot(robot_sn, logger=self.get_logger())
    
    def publish_wrench(self):
        """Read robot F/T data and publish as WrenchStamped."""
        try:
            states = self.robot.states()
            now = self.get_clock().now().to_msg()

            self.wrench_pub.publish(build_wrench_msg(states.ext_wrench_in_tcp, now, self.frame_id))
            self.tcp_pose_pub.publish(build_pose_msg(states.tcp_pose, now, self.frame_id))

        except Exception as e:
            self.get_logger().error(f"Error publishing wrench data: {str(e)}")
    
    def shutdown(self):
        """Clean shutdown."""
        self.logger.info("Shutting down Flexiv wrench publisher...")


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        node = FlexivWrenchPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # normal shutdown
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()