#!/usr/bin/env python3
"""Stream tactile/sensor camera to VR headset over ZMQ.

Subscribes to ROS2 /image_raw, overlays force reading from CoinFT,
and publishes JPEG-compressed frames over ZMQ for the Quest headset.

Usage:
    ros2 run visionft tactile_stream
    ros2 run visionft tactile_stream --port 15001 --topic /image_raw
"""

import argparse
import sys

import time

import cv2
import numpy as np
import zmq
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped


class ROS2CamBridge(Node):
    """Subscribes to ROS2 Image topic, overlays force, streams JPEG over ZMQ."""

    def __init__(self, topic: str, port: int, quality: int = 70, force_topic: str = '/coinft/wrench'):
        super().__init__('tactile_stream')
        ctx = zmq.Context()
        self._socket = ctx.socket(zmq.PUB)
        self._socket.bind(f'tcp://*:{port}')
        self._quality = quality
        self._frame_count = 0

        # Force overlay state
        self._last_force = 0.0
        self._last_force_time = 0.0
        self._force_display_sec = 0.5

        self.create_subscription(Image, topic, self._on_image, 1)
        self.create_subscription(WrenchStamped, force_topic, self._on_wrench, 1)
        self.get_logger().info(f'Bridging {topic} -> ZMQ port {port} (force overlay from {force_topic})')

    def _on_image(self, msg: Image):
        # Convert ROS2 Image to numpy BGR
        if msg.encoding == 'rgb8':
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif msg.encoding == 'bgr8':
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
        elif msg.encoding == 'mono8':
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width).copy()
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            self.get_logger().warn(f'Unsupported encoding: {msg.encoding}', throttle_duration_sec=5.0)
            return

        # Overlay force (always show latest)
        if self._last_force_time > 0:
            force_z = self._last_force
            # Color: green < 5N, yellow < 15N, red >= 15N
            if abs(force_z) < 5.0:
                color = (0, 255, 0)
            elif abs(force_z) < 15.0:
                color = (0, 255, 255)
            else:
                color = (0, 0, 255)
            text = f'Fz: {force_z:.1f} N'
            font_scale = max(frame.shape[0], frame.shape[1]) / 500.0
            thickness = max(2, int(font_scale * 2))
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
            cx = (frame.shape[1] - tw) // 2
            cy = th + 20  # top center
            cv2.putText(frame, text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 4)
            cv2.putText(frame, text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), thickness)

        _, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._quality])
        self._socket.send(np.array(buf).tobytes())
        self._frame_count += 1
        if self._frame_count == 1:
            self.get_logger().info(f'Streaming {msg.width}x{msg.height} ({msg.encoding})')

    def _on_wrench(self, msg: WrenchStamped):
        self._last_force = msg.wrench.force.z
        self._last_force_time = time.time()

    def destroy_node(self):
        self._socket.close()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='Bridge ROS2 camera to ZMQ for VR')
    parser.add_argument('--topic', default='/image_raw', help='ROS2 image topic')
    parser.add_argument('--port', type=int, default=15001, help='ZMQ publish port')
    parser.add_argument('--quality', type=int, default=70, help='JPEG quality (0-100)')
    args = parser.parse_args()

    rclpy.init()
    node = ROS2CamBridge(args.topic, args.port, args.quality)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
