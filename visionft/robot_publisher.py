#!/usr/bin/env python3

"""robot_publisher.py

Reads Flexiv robot states and publishes ext_wrench_in_tcp as WrenchStamped.
"""

__copyright__ = "Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import spdlog
import flexivrdk


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
        
        # Setup logger
        self.logger = spdlog.ConsoleLogger("FlexivPublisher")
        
        # Initialize robot
        self.robot = None
        self.initialize_robot(robot_sn)
        
        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_wrench)
        
        self.get_logger().info(f"Publishing wrench data to /flexiv/wrench at {publish_rate} Hz")
    
    def initialize_robot(self, robot_sn):
        """Initialize connection to Flexiv robot."""
        try:
            # Instantiate robot interface
            self.robot = flexivrdk.Robot(robot_sn)
            
            # Clear fault on the connected robot if any
            if self.robot.fault():
                self.logger.warn("Fault occurred on the connected robot, trying to clear...")
                if not self.robot.ClearFault():
                    self.logger.error("Fault cannot be cleared, exiting...")
                    raise RuntimeError("Cannot clear robot fault")
                self.logger.info("Fault on the connected robot is cleared")
            
            # Enable the robot
            self.logger.info("Enabling robot...")
            self.robot.Enable()
            
            # Wait for the robot to become operational
            while not self.robot.operational():
                time.sleep(1)
            
            self.logger.info("Robot is now operational")
            
        except Exception as e:
            self.logger.error(f"Failed to initialize robot: {str(e)}")
            raise
    
    def publish_wrench(self):
        """Read robot F/T data and publish as WrenchStamped."""
        try:
            # Get external wrench in TCP frame [fx, fy, fz, tx, ty, tz]
            ext_wrench = self.robot.states().ext_wrench_in_tcp
            
            # Create and publish WrenchStamped message
            wrench_msg = WrenchStamped()
            wrench_msg.header.stamp = self.get_clock().now().to_msg()
            wrench_msg.header.frame_id = self.frame_id
            
            wrench_msg.wrench.force.x = float(ext_wrench[0])
            wrench_msg.wrench.force.y = float(ext_wrench[1])
            wrench_msg.wrench.force.z = float(ext_wrench[2])
            wrench_msg.wrench.torque.x = float(ext_wrench[3])
            wrench_msg.wrench.torque.y = float(ext_wrench[4])
            wrench_msg.wrench.torque.z = float(ext_wrench[5])
            
            self.wrench_pub.publish(wrench_msg)
            
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
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()