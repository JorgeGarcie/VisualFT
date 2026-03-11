#!/usr/bin/env python3
# DO NOT NEED WAS USED BEFORE TO INTERFACE WITH UDP

"""flexiv_udp_receiver.py

Receives Flexiv F/T data via UDP and publishes as WrenchStamped.

"""

import socket
import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped


class FlexivUDPReceiver(Node):
    def __init__(self):
        super().__init__('flexiv_udp_receiver')
        
        # Declare parameters
        self.declare_parameter('udp_ip', '0.0.0.0')  # Listen on all interfaces
        self.declare_parameter('udp_port', 12345)
        self.declare_parameter('frame_id', 'flexiv_tcp')
        self.declare_parameter('timeout', 1.0)  # seconds
        
        udp_ip = self.get_parameter('udp_ip').value
        udp_port = self.get_parameter('udp_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.timeout = self.get_parameter('timeout').value
        
        # Create publisher
        self.wrench_pub = self.create_publisher(WrenchStamped, '/flexiv/wrench', 10)
        
        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((udp_ip, udp_port))
        self.sock.settimeout(self.timeout)
        
        self.get_logger().info(f"Listening for UDP data on {udp_ip}:{udp_port}")
        self.get_logger().info(f"Publishing to topic: /flexiv/wrench")
        
        # Create timer to check for data
        self.timer = self.create_timer(0.001, self.receive_and_publish)  # 1ms timer
    
    def receive_and_publish(self):
        """Receive UDP data and publish as WrenchStamped."""
        try:
            # Receive data (non-blocking due to timeout)
            data, addr = self.sock.recvfrom(24)  # 6 floats * 4 bytes = 24 bytes
            
            if len(data) == 24:
                # Unpack 6 floats
                wrench_data = struct.unpack('<6f', data)
                
                # Create and publish WrenchStamped message
                wrench_msg = WrenchStamped()
                wrench_msg.header.stamp = self.get_clock().now().to_msg()
                wrench_msg.header.frame_id = self.frame_id
                
                wrench_msg.wrench.force.x = float(wrench_data[0])
                wrench_msg.wrench.force.y = float(wrench_data[1])
                wrench_msg.wrench.force.z = float(wrench_data[2])
                wrench_msg.wrench.torque.x = float(wrench_data[3])
                wrench_msg.wrench.torque.y = float(wrench_data[4])
                wrench_msg.wrench.torque.z = float(wrench_data[5])
                
                self.wrench_pub.publish(wrench_msg)
                
        except socket.timeout:
            # No data received within timeout - this is normal
            pass
        except Exception as e:
            self.get_logger().error(f"Error receiving UDP data: {str(e)}")
    
    def shutdown(self):
        """Clean shutdown."""
        self.get_logger().info("Shutting down UDP receiver...")
        self.sock.close()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = FlexivUDPReceiver()
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