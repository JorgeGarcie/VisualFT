#!/usr/bin/env python3

import numpy as np
import serial
import time
import threading
import scipy.io
import onnxruntime as ort
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

class CoinFTPublisher(Node):
    def __init__(self):
        super().__init__('coinft_wrench_publisher')
        
        # Declare parameters
        self.declare_parameter('com_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('frame_id', 'coinft_sensor')
        self.declare_parameter('data_dir', '/home/li2053/Teo/VisionFT Files/data')
        self.declare_parameter('model_file', 'PFT1-1_MLP_5L_norm_L2.onnx')
        self.declare_parameter('norm_file', 'PFT1-1_norm_constants.mat')
        
        # Get parameters
        self.com_port = self.get_parameter('com_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        data_dir = self.get_parameter('data_dir').value
        model_file = self.get_parameter('model_file').value
        norm_file = self.get_parameter('norm_file').value
        
        # Serial configuration
        self.START_BYTE = 2
        self.END_BYTE = 3
        
        # Load model and normalization constants
        os.chdir(data_dir)
        onnx_model_path = model_file
        self.ort_session = ort.InferenceSession(onnx_model_path)
        norm_data = scipy.io.loadmat(norm_file)
        self.mu_x = norm_data['norm_const']['mu_x'][0,0].flatten()
        self.sd_x = norm_data['norm_const']['sd_x'][0,0].flatten()
        self.mu_y = norm_data['norm_const']['mu_y'][0,0].flatten()
        self.sd_y = norm_data['norm_const']['sd_y'][0,0].flatten()
        
        # Create publisher
        self.wrench_pub = self.create_publisher(WrenchStamped, '/coinft/wrench', 10)
        
        # Global variables
        self.stop_flag = False
        self.offset_CoinFT_list = []
        self.read_count = 0
        self.got_initial_offset = False
        self.offset_CoinFT = None
        
        # Serial setup
        self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=0.1)
        self.ser.write(b'i')
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.write(b'q')
        time.sleep(0.01)
        
        packet_size_excludeStartByte = self.ser.read(1)
        if len(packet_size_excludeStartByte) < 1:
            raise RuntimeError("Failed to read packet size from sensor")
        self.packet_size_excludeStartByte = ord(packet_size_excludeStartByte) - 1
        self.num_Channels = (self.packet_size_excludeStartByte - 2) // 2
        
        # Start streaming
        self.ser.write(b's')
        self.get_logger().info(f"# Channels: {self.num_Channels}")
        self.get_logger().info(f"Publishing to topic: /coinft/wrench")
        
        # Start sensor reading thread
        self.sens_thread = threading.Thread(target=self.read_CoinFT, daemon=True)
        self.sens_thread.start()
    
    def read_CoinFT(self):
        """Thread function to read the CoinFT sensor data via serial port at ~360Hz."""
        initialSampleNum = 1500
        ft_bias_ready = False
        ft_bias = None
        
        while not self.stop_flag and rclpy.ok():
            byte = self.ser.read(1)
            if len(byte) == 0:
                continue
            if byte[0] != self.START_BYTE:
                continue
            
            data = self.ser.read(self.packet_size_excludeStartByte)
            if len(data) < self.packet_size_excludeStartByte:
                continue
            
            end_byte = data[-1]
            if end_byte == self.END_BYTE:
                sensor_data = []
                for byte_num in range(0, self.packet_size_excludeStartByte-3, 2):
                    low = data[byte_num]
                    high = data[byte_num+1]
                    val = low + 256*high
                    sensor_data.append(val)
                sensor_data = np.array(sensor_data, dtype=np.float64)
                self.read_count += 1
                
                # Gather initial offset
                if self.read_count <= initialSampleNum:
                    self.offset_CoinFT_list.append(sensor_data)
                    if self.read_count % 500 == 0:
                        self.get_logger().info(f"Collecting offset: {self.read_count}/{initialSampleNum}")
                    continue
                elif self.read_count == initialSampleNum + 1:
                    self.offset_CoinFT = np.mean(self.offset_CoinFT_list[5:], axis=0)
                    self.got_initial_offset = True
                    self.get_logger().info("Offset computed, streaming calibrated data...")
                    continue
                else:
                    if not self.got_initial_offset:
                        continue
                    
                    # Apply offset
                    SensorData_offsetted = sensor_data - self.offset_CoinFT
                    
                    # Use deep learning model for calibration
                    x_norm = (SensorData_offsetted - self.mu_x) / self.sd_x
                    x_input = x_norm.astype(np.float32).reshape(1, -1)
                    calibratedForceTorque = self.ort_session.run(None, {"input": x_input})[0].flatten()
                    calibratedForceTorque = calibratedForceTorque * self.sd_y + self.mu_y
                    
                    # Zero drift/bias with first calibrated sample
                    if not ft_bias_ready:
                        ft_bias = calibratedForceTorque
                        ft_bias_ready = True
                    else:
                        calibratedForceTorque = calibratedForceTorque - ft_bias
                    
                    # Publish as WrenchStamped
                    wrench_msg = WrenchStamped()
                    wrench_msg.header.stamp = self.get_clock().now().to_msg()
                    wrench_msg.header.frame_id = self.frame_id
                    
                    wrench_msg.wrench.force.x = float(calibratedForceTorque[0])
                    wrench_msg.wrench.force.y = float(calibratedForceTorque[1])
                    wrench_msg.wrench.force.z = float(calibratedForceTorque[2])
                    wrench_msg.wrench.torque.x = float(calibratedForceTorque[3])
                    wrench_msg.wrench.torque.y = float(calibratedForceTorque[4])
                    wrench_msg.wrench.torque.z = float(calibratedForceTorque[5])
                    
                    self.wrench_pub.publish(wrench_msg)
                    
                    # Optional: print occasional updates
                    if self.read_count % 360 == 0:  # Every ~1 second at 360Hz
                        self.get_logger().info(
                            f"F: [{calibratedForceTorque[0]:6.2f}, {calibratedForceTorque[1]:6.2f}, {calibratedForceTorque[2]:6.2f}] N | "
                            f"T: [{calibratedForceTorque[3]:6.3f}, {calibratedForceTorque[4]:6.3f}, {calibratedForceTorque[5]:6.3f}] Nm"
                        )
    
    def shutdown(self):
        """Clean shutdown of the node."""
        self.get_logger().info("Shutting down CoinFT publisher...")
        self.stop_flag = True
        if hasattr(self, 'sens_thread'):
            self.sens_thread.join(timeout=2.0)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        self.get_logger().info("Serial port closed.")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CoinFTPublisher()
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