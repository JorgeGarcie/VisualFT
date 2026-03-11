#!/usr/bin/env python3

"""data_logger.py

Logs CoinFT wrench, Flexiv wrench, TCP pose, and camera frames to separate CSV files.
Saves images at 10Hz and records full video stream at native framerate.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import csv
import os
from datetime import datetime
import threading


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # Declare parameter for image save rate
        self.declare_parameter('image_save_hz', 30.0)
        self.image_save_hz = self.get_parameter('image_save_hz').value
        self.image_save_interval = 1.0 / self.image_save_hz
        self.last_image_save_time = 0.0

        # Create timestamped directory for this trial
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.trial_dir = f"trial_{timestamp}"
        self.frames_dir = os.path.join(self.trial_dir, "frames")
        os.makedirs(self.frames_dir, exist_ok=True)

        # Initialize start time as None - will be set on first message
        self.start_time = None
        self.lock = threading.Lock()

        # Camera setup
        self.bridge = CvBridge()
        self.frame_counter = 0

        # Video writer setup
        self.video_writer = None
        self.video_path = os.path.join(self.trial_dir, "full_stream.mp4")
        self.video_initialized = False

        # Setup CSV files
        self.setup_csv_files()

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

        self.camera_sub = self.create_subscription(
            Image,
            '/image_raw',  # Change to your camera topic
            self.camera_callback,
            10
        )

        self.get_logger().info(f"Data Logger started - saving to: {self.trial_dir}")
        self.get_logger().info(f"Image save rate: {self.image_save_hz} Hz")
        self.get_logger().info("Subscribed to: /coinft/wrench, /flexiv/wrench, /flexiv/tcp_pose, /camera/image_raw")

    def setup_csv_files(self):
        """Setup all CSV files with headers."""
        # Wrench data CSV (both sensors in one file)
        self.wrench_csv_path = os.path.join(self.trial_dir, "wrench_data.csv")
        self.wrench_csv = open(self.wrench_csv_path, 'w', newline='')
        self.wrench_writer = csv.writer(self.wrench_csv)
        self.wrench_writer.writerow(['time', 'sensor', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz'])
        self.wrench_csv.flush()

        # TCP pose CSV
        self.tcp_csv_path = os.path.join(self.trial_dir, "tcp_pose.csv")
        self.tcp_csv = open(self.tcp_csv_path, 'w', newline='')
        self.tcp_writer = csv.writer(self.tcp_csv)
        self.tcp_writer.writerow(['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.tcp_csv.flush()

        # Camera frames CSV
        self.camera_csv_path = os.path.join(self.trial_dir, "camera_frames.csv")
        self.camera_csv = open(self.camera_csv_path, 'w', newline='')
        self.camera_writer = csv.writer(self.camera_csv)
        self.camera_writer.writerow(['time', 'frame_number', 'image_path'])
        self.camera_csv.flush()

        self.get_logger().info(f"CSV files created in {self.trial_dir}")

    def coinft_callback(self, msg):
        """Callback for CoinFT wrench data."""
        with self.lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now().nanoseconds / 1e9

            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time

            fx = -msg.wrench.force.x
            fy = -msg.wrench.force.y
            fz = msg.wrench.force.z
            tx = msg.wrench.torque.x
            ty = msg.wrench.torque.y
            tz = msg.wrench.torque.z

            self.wrench_writer.writerow([current_time, 'coinft', fx, fy, fz, tx, ty, tz])
            self.wrench_csv.flush()

    def flexiv_callback(self, msg):
        """Callback for Flexiv wrench data."""
        with self.lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now().nanoseconds / 1e9

            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time

            fx = msg.wrench.force.x
            fy = msg.wrench.force.y
            fz = msg.wrench.force.z
            tx = msg.wrench.torque.x
            ty = msg.wrench.torque.y
            tz = msg.wrench.torque.z

            self.wrench_writer.writerow([current_time, 'flexiv', fx, fy, fz, tx, ty, tz])
            self.wrench_csv.flush()

    def tcp_callback(self, msg):
        """Callback for TCP Pose data."""
        with self.lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now().nanoseconds / 1e9

            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time

            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w

            self.tcp_writer.writerow([current_time, x, y, z, qx, qy, qz, qw])
            self.tcp_csv.flush()

    def camera_callback(self, msg):
        """Process camera frame: save to video (all frames) and images (10Hz)."""
        with self.lock:
            if self.start_time is None:
                self.start_time = self.get_clock().now().nanoseconds / 1e9

            current_time = self.get_clock().now().nanoseconds / 1e9 - self.start_time

            try:
                # Convert ROS Image to OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

                # Initialize video writer on first frame
                if not self.video_initialized:
                    height, width = cv_image.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    # Assume camera is publishing at ~30Hz
                    self.video_writer = cv2.VideoWriter(
                        self.video_path,
                        fourcc,
                        30.0,  # fps - adjust if your camera is different
                        (width, height)
                    )
                    self.video_initialized = True
                    self.get_logger().info(f"Video recording started: {self.video_path} at 30fps")

                # Write every frame to video
                if self.video_writer is not None:
                    self.video_writer.write(cv_image)

                # Save individual images at specified rate (10Hz)
                if current_time - self.last_image_save_time >= self.image_save_interval:
                    frame_filename = f"frame_{self.frame_counter:06d}.jpg"
                    frame_path = os.path.join(self.frames_dir, frame_filename)

                    # Save image with 95% JPEG quality
                    cv2.imwrite(frame_path, cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95])

                    # Log to CSV (store relative path from trial directory)
                    relative_path = os.path.join("frames", frame_filename)
                    self.camera_writer.writerow([current_time, self.frame_counter, relative_path])
                    self.camera_csv.flush()

                    self.last_image_save_time = current_time
                    self.frame_counter += 1

                    if self.frame_counter % 10 == 0:  # Log every second at 10Hz
                        self.get_logger().info(f"Logged {self.frame_counter} frames")

            except Exception as e:
                self.get_logger().error(f"Error processing camera frame: {e}")

    def close_files(self):
        """Close all CSV files, video writer, and print summary."""
        if hasattr(self, 'wrench_csv') and self.wrench_csv:
            self.wrench_csv.close()
        if hasattr(self, 'tcp_csv') and self.tcp_csv:
            self.tcp_csv.close()
        if hasattr(self, 'camera_csv') and self.camera_csv:
            self.camera_csv.close()

        # Release video writer
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"Video saved: {self.video_path}")

        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Data logging complete!")
        self.get_logger().info(f"{'='*60}")
        self.get_logger().info(f"Data saved to: {self.trial_dir}/")
        self.get_logger().info(f"  - Individual frames saved: {self.frame_counter} (at {self.image_save_hz} Hz)")
        self.get_logger().info(f"  - Full video: full_stream.mp4")
        self.get_logger().info(f"{'='*60}\n")


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = DataLogger()
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nStopping data logger...")
    finally:
        if node is not None:
            node.close_files()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
