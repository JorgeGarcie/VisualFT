#!/usr/bin/env python3
"""Publish USB camera frames to /image_raw.

Simple replacement for gscam2 when using a USB camera instead of
the Raspberry Pi H264 stream.

Usage:
    ros2 run visionft usb_camera
    ros2 run visionft usb_camera --device 0 --fps 30 --width 640 --height 480
"""

import argparse

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class USBCameraNode(Node):
    def __init__(self, device: int, fps: int, width: int, height: int):
        super().__init__('usb_camera')
        self._pub = self.create_publisher(Image, '/image_raw', 1)

        self._cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self.get_logger().error(f'Could not open /dev/video{device}')
            return

        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(
            f'USB camera opened: /dev/video{device} {actual_w}x{actual_h}')

        period = 1.0 / fps
        self._timer = self.create_timer(period, self._publish_frame)
        self._frame_count = 0

    def _publish_frame(self):
        ret, frame = self._cap.read()
        if not ret:
            return

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = 'rgb8'
        msg.step = msg.width * 3
        msg.data = rgb.tobytes()
        self._pub.publish(msg)

        self._frame_count += 1
        if self._frame_count == 1:
            self.get_logger().info('First frame published')

    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='USB camera → /image_raw')
    parser.add_argument('--device', type=int, default=0,
                        help='Video device index (default: 0)')
    parser.add_argument('--fps', type=int, default=30,
                        help='Target frame rate (default: 30)')
    parser.add_argument('--width', type=int, default=640,
                        help='Frame width (default: 640)')
    parser.add_argument('--height', type=int, default=480,
                        help='Frame height (default: 480)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = USBCameraNode(args.device, args.fps, args.width, args.height)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
