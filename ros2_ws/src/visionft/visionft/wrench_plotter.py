#!/usr/bin/env python3

"""wrench_plotter.py

Subscribes to CoinFT and Flexiv wrench topics and TCP pose, plotting TCP X and Y positions.
Saves all data to CSV file in real-time.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import csv
from datetime import datetime


class WrenchPlotter(Node):
    def __init__(self):
        super().__init__('wrench_plotter')
        
        # Declare parameters
        self.declare_parameter('plot_duration', 15.0)  # seconds
        self.declare_parameter('update_rate', 50)  # Hz
        
        plot_duration = self.get_parameter('plot_duration').value
        update_rate = self.get_parameter('update_rate').value
        
        # Data storage - NO maxlen to keep ALL historical data
        # CoinFT data
        self.coinft_time = deque()
        self.coinft_fx = deque()
        self.coinft_fy = deque()
        self.coinft_fz = deque()
        self.coinft_tx = deque()
        self.coinft_ty = deque()
        self.coinft_tz = deque()
        
        # Flexiv data
        self.flexiv_time = deque()
        self.flexiv_fx = deque()
        self.flexiv_fy = deque()
        self.flexiv_fz = deque()
        self.flexiv_tx = deque()
        self.flexiv_ty = deque()
        self.flexiv_tz = deque()
        
        # TCP Position 
        self.tcp_time = deque()
        self.tcp_x = deque()
        self.tcp_y = deque()
        self.tcp_z = deque()
        self.tcp_qx = deque()
        self.tcp_qy = deque()
        self.tcp_qz = deque()
        self.tcp_qw = deque()
        
        # Initialize start time as None - will be set on first message
        self.start_time = None
        self.lock = threading.Lock()
        
        # Setup CSV file for real-time writing
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = f"wrench_and_position_data_{timestamp}.csv"
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write CSV header
        self.csv_writer.writerow(['time', 'sensor', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz'])
        self.csv_file.flush()  # Ensure header is written immediately
        
        # Create subscribers
        self.coinft_sub = self.create_subscription(
            WrenchStamped,
            '/coinft/wrench',
            self.coinft_callback,
            10
        )
        
        self.flexiv_sub = self.create_subscription(
            WrenchStamped,
            '/flexiv/wrench',
            self.flexiv_callback,
            10
        )

        self.tcp_sub = self.create_subscription(
            PoseStamped,
            '/flexiv/tcp_pose',
            self.tcp_callback,
            10
        )
        
        self.get_logger().info("Wrench Plotter started")
        self.get_logger().info("Subscribing to /coinft/wrench, /flexiv/wrench, and /flexiv/tcp_pose")
        self.get_logger().info(f"Writing data in real-time to: {self.csv_filename}")
        
        # Setup plot
        self.setup_plot()
        
    def coinft_callback(self, msg):
        """Callback for CoinFT wrench data."""
        with self.lock:
            # Initialize start time on first message
            if self.start_time is None:
                self.start_time = self.get_clock().now().nanoseconds / 1e9
            
            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            
            # Store data
            fx = -msg.wrench.force.x
            fy = -msg.wrench.force.y
            fz = msg.wrench.force.z
            tx = msg.wrench.torque.x
            ty = msg.wrench.torque.y
            tz = msg.wrench.torque.z
            
            self.coinft_time.append(current_time)
            self.coinft_fx.append(fx)
            self.coinft_fy.append(fy)
            self.coinft_fz.append(fz)
            self.coinft_tx.append(tx)
            self.coinft_ty.append(ty)
            self.coinft_tz.append(tz)
            
            # Write to CSV immediately
            self.csv_writer.writerow([current_time, 'coinft', fx, fy, fz, tx, ty, tz])
            self.csv_file.flush()  # Force write to disk
    
    def flexiv_callback(self, msg):
        """Callback for Flexiv wrench data."""
        with self.lock:
            # Initialize start time on first message
            if self.start_time is None:
                self.start_time = self.get_clock().now().nanoseconds / 1e9
            
            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            
            # Store data
            fx = msg.wrench.force.x
            fy = msg.wrench.force.y
            fz = msg.wrench.force.z
            tx = msg.wrench.torque.x
            ty = msg.wrench.torque.y
            tz = msg.wrench.torque.z
            
            self.flexiv_time.append(current_time)
            self.flexiv_fx.append(fx)
            self.flexiv_fy.append(fy)
            self.flexiv_fz.append(fz)
            self.flexiv_tx.append(tx)
            self.flexiv_ty.append(ty)
            self.flexiv_tz.append(tz)
            
            # Write to CSV immediately
            self.csv_writer.writerow([current_time, 'flexiv', fx, fy, fz, tx, ty, tz])
            self.csv_file.flush()  # Force write to disk

    def tcp_callback(self, msg):
        """Callback for TCP Pose data."""
        with self.lock:
            # Initialize start time on first message
            if self.start_time is None:
                self.start_time = self.get_clock().now().nanoseconds / 1e9
            
            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time
            
            # Store data
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w
            
            self.tcp_time.append(current_time)
            self.tcp_x.append(x)
            self.tcp_y.append(y)
            self.tcp_z.append(z)
            self.tcp_qx.append(qx)
            self.tcp_qy.append(qy)
            self.tcp_qz.append(qz)
            self.tcp_qw.append(qw)

            #Write to CSV immediately
            self.csv_writer.writerow([current_time, 'tcp_pose', x, y, z, qx, qy, qz, qw])
            self.csv_file.flush()  # Force write to disk
    
    def setup_plot(self):
        """Setup matplotlib figure with two subplots."""
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(14, 10))
        
        # Top plot: Force/Torque data
        self.ax1.set_title('Force/Torque: CoinFT vs Flexiv')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Value')
        self.ax1.grid(True, alpha=0.3)
        
        # CoinFT lines (solid)
        self.line_coinft_fx, = self.ax1.plot([], [], '-', color='#1f77b4', label='CoinFT Fx', linewidth=1.5)
        self.line_coinft_fy, = self.ax1.plot([], [], '-', color='#ff7f0e', label='CoinFT Fy', linewidth=1.5)
        self.line_coinft_fz, = self.ax1.plot([], [], '-', color='#2ca02c', label='CoinFT Fz', linewidth=1.5)
        self.line_coinft_tx, = self.ax1.plot([], [], '-', color='#d62728', label='CoinFT Tx', linewidth=1.5)
        self.line_coinft_ty, = self.ax1.plot([], [], '-', color='#9467bd', label='CoinFT Ty', linewidth=1.5)
        self.line_coinft_tz, = self.ax1.plot([], [], '-', color='#8c564b', label='CoinFT Tz', linewidth=1.5)
        
        # Flexiv lines (dashed)
        self.line_flexiv_fx, = self.ax1.plot([], [], '--', color='#1f77b4', label='Flexiv Fx', linewidth=1.5)
        self.line_flexiv_fy, = self.ax1.plot([], [], '--', color='#ff7f0e', label='Flexiv Fy', linewidth=1.5)
        self.line_flexiv_fz, = self.ax1.plot([], [], '--', color='#2ca02c', label='Flexiv Fz', linewidth=1.5)
        self.line_flexiv_tx, = self.ax1.plot([], [], '--', color='#d62728', label='Flexiv Tx', linewidth=1.5)
        self.line_flexiv_ty, = self.ax1.plot([], [], '--', color='#9467bd', label='Flexiv Ty', linewidth=1.5)
        self.line_flexiv_tz, = self.ax1.plot([], [], '--', color='#8c564b', label='Flexiv Tz', linewidth=1.5)
        
        self.ax1.legend(loc='upper right', ncol=2, fontsize=8)
        
        # Bottom plot: TCP X and Y positions
        self.ax2.set_title('TCP Position: X and Y')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Position (m)')
        self.ax2.grid(True, alpha=0.3)
        
        # TCP position lines
        self.line_tcp_x, = self.ax2.plot([], [], '-', color='#1f77b4', label='TCP X', linewidth=2)
        self.line_tcp_y, = self.ax2.plot([], [], '-', color='#ff7f0e', label='TCP Y', linewidth=2)
        
        self.ax2.legend(loc='upper right', fontsize=10)
        
        plt.tight_layout()
        
        # Animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
    
    def update_plot(self, frame):
        """Update plot with new data."""
        with self.lock:
            # Define sliding window duration (seconds)
            window_duration = 15.0  # Show last 15 seconds
            
            # Get current max time
            max_time = max(
                max(self.coinft_time) if len(self.coinft_time) > 0 else 0,
                max(self.flexiv_time) if len(self.flexiv_time) > 0 else 0,
                max(self.tcp_time) if len(self.tcp_time) > 0 else 0
            )
            
            min_time = max(0, max_time - window_duration)
            
            # Update CoinFT lines (filter to window)
            if len(self.coinft_time) > 0:
                coinft_times = list(self.coinft_time)
                coinft_mask = [t >= min_time for t in coinft_times]
                
                filtered_times = [t for t, m in zip(coinft_times, coinft_mask) if m]
                
                self.line_coinft_fx.set_data(filtered_times, [v for v, m in zip(self.coinft_fx, coinft_mask) if m])
                self.line_coinft_fy.set_data(filtered_times, [v for v, m in zip(self.coinft_fy, coinft_mask) if m])
                self.line_coinft_fz.set_data(filtered_times, [v for v, m in zip(self.coinft_fz, coinft_mask) if m])
                self.line_coinft_tx.set_data(filtered_times, [v for v, m in zip(self.coinft_tx, coinft_mask) if m])
                self.line_coinft_ty.set_data(filtered_times, [v for v, m in zip(self.coinft_ty, coinft_mask) if m])
                self.line_coinft_tz.set_data(filtered_times, [v for v, m in zip(self.coinft_tz, coinft_mask) if m])
            
            # Update Flexiv lines (filter to window)
            if len(self.flexiv_time) > 0:
                flexiv_times = list(self.flexiv_time)
                flexiv_mask = [t >= min_time for t in flexiv_times]
                
                filtered_times = [t for t, m in zip(flexiv_times, flexiv_mask) if m]
                
                self.line_flexiv_fx.set_data(filtered_times, [v for v, m in zip(self.flexiv_fx, flexiv_mask) if m])
                self.line_flexiv_fy.set_data(filtered_times, [v for v, m in zip(self.flexiv_fy, flexiv_mask) if m])
                self.line_flexiv_fz.set_data(filtered_times, [v for v, m in zip(self.flexiv_fz, flexiv_mask) if m])
                self.line_flexiv_tx.set_data(filtered_times, [v for v, m in zip(self.flexiv_tx, flexiv_mask) if m])
                self.line_flexiv_ty.set_data(filtered_times, [v for v, m in zip(self.flexiv_ty, flexiv_mask) if m])
                self.line_flexiv_tz.set_data(filtered_times, [v for v, m in zip(self.flexiv_tz, flexiv_mask) if m])
            
            # Update TCP lines (filter to window)
            if len(self.tcp_time) > 0:
                tcp_times = list(self.tcp_time)
                tcp_mask = [t >= min_time for t in tcp_times]
                
                filtered_times = [t for t, m in zip(tcp_times, tcp_mask) if m]
                
                self.line_tcp_x.set_data(filtered_times, [v for v, m in zip(self.tcp_x, tcp_mask) if m])
                self.line_tcp_y.set_data(filtered_times, [v for v, m in zip(self.tcp_y, tcp_mask) if m])
            
            # Set fixed x-axis limits for sliding window on both subplots
            self.ax1.set_xlim(min_time, max_time)
            self.ax2.set_xlim(min_time, max_time)
            
            # Auto-scale y-axis only for both subplots
            self.ax1.relim()
            self.ax1.autoscale_view(scalex=False, scaley=True)
            self.ax2.relim()
            self.ax2.autoscale_view(scalex=False, scaley=True)
        
        return (self.line_coinft_fx, self.line_coinft_fy, self.line_coinft_fz,
                self.line_coinft_tx, self.line_coinft_ty, self.line_coinft_tz,
                self.line_flexiv_fx, self.line_flexiv_fy, self.line_flexiv_fz,
                self.line_flexiv_tx, self.line_flexiv_ty, self.line_flexiv_tz,
                self.line_tcp_x, self.line_tcp_y)
    
    def close_csv(self):
        """Close the CSV file."""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f"CSV file closed: {self.csv_filename}")
            print(f"\n✓ Data saved to {self.csv_filename}")
            print(f"  - CoinFT points: {len(self.coinft_time)}")
            print(f"  - Flexiv points: {len(self.flexiv_time)}")
            print(f"  - TCP points: {len(self.tcp_time)}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WrenchPlotter()
        
        # Spin in a separate thread so matplotlib can run in main thread
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()
        
        # Show plot (blocking)
        plt.show()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        if 'node' in locals():
            # Close CSV file before shutting down
            node.close_csv()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()