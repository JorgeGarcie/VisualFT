"""
Sensor Bridge — ROS2 image + force overlay → ZMQ for Quest headset
==================================================================
Subscribes to:
  /image_raw          (sensor_msgs/Image)     — camera feed from gscam2
  /rdk/wrench         (WrenchStamped)         — external wrench (from scan_node or teleop)
  /coinft/wrench      (WrenchStamped)         — CoinFT sensor (optional)

Publishes:
  ZMQ PUB on port 10505 — JPEG frames with force overlay for FrankaBot VR app

Usage:
  # Requires visionft.launch.py running for /image_raw
  # Requires teleop or scan_node running for /rdk/wrench
  python3 sensor_bridge.py
  python3 sensor_bridge.py --port 10505 --no-ros-wrench  # camera only, no wrench overlay
"""

import sys
import argparse
import threading
import numpy as np
import cv2

# Ensure user site-packages (for pyzmq)
_user_site = '/home/li2053/.local/lib/python3.10/site-packages'
if _user_site not in sys.path:
    sys.path.append(_user_site)

import zmq

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped


class SensorBridge(Node):
    def __init__(self, camera_port=10505, graph_port=15001, show_local=True):
        super().__init__('sensor_bridge')

        self._show_local = show_local

        # ZMQ publishers for VR headset
        ctx = zmq.Context()
        self._zmq_camera = ctx.socket(zmq.PUB)
        self._zmq_camera.bind(f'tcp://*:{camera_port}')
        self._zmq_graph = ctx.socket(zmq.PUB)
        self._zmq_graph.bind(f'tcp://*:{graph_port}')
        self.get_logger().info(f'ZMQ camera on port {camera_port}, force graph on port {graph_port}')

        # State
        self._last_frame = None
        self._frame_lock = threading.Lock()
        self._fz_robot = 0.0       # Z force from robot F/T
        self._fz_coinft = 0.0      # Z force from CoinFT
        self._force_lock = threading.Lock()
        self._force_history = []    # for sparkline
        self._max_history = 120     # ~4s at 30fps

        # ROS2 subscribers
        self.create_subscription(Image, '/image_raw', self._on_image, 10)
        self.create_subscription(WrenchStamped, '/rdk/wrench', self._on_robot_wrench, 10)
        self.create_subscription(WrenchStamped, '/coinft/wrench', self._on_coinft_wrench, 10)

        # Timer for overlay + publish at 30Hz
        self.create_timer(1.0 / 30, self._publish_frame)

        self.get_logger().info('Sensor bridge running. Subscribed to /image_raw, /rdk/wrench, /coinft/wrench')

    def _on_image(self, msg: Image):
        """Convert ROS2 Image to OpenCV BGR."""
        try:
            # Handle common encodings
            h, w = msg.height, msg.width
            if msg.encoding in ('rgb8', 'RGB8'):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding in ('bgr8', 'BGR8'):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            elif msg.encoding in ('mono8', 'MONO8'):
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                # Try as BGR
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)

            with self._frame_lock:
                self._last_frame = frame.copy()
        except Exception as e:
            self.get_logger().warn(f'Image decode error: {e}', throttle_duration_sec=5.0)

    def _on_robot_wrench(self, msg: WrenchStamped):
        with self._force_lock:
            self._fz_robot = msg.wrench.force.z

    def _on_coinft_wrench(self, msg: WrenchStamped):
        with self._force_lock:
            self._fz_coinft = msg.wrench.force.z

    def _draw_force_overlay(self, frame):
        """Draw Z-force bar + sparkline on the frame."""
        h, w = frame.shape[:2]

        with self._force_lock:
            fz = self._fz_robot
            fz_coinft = self._fz_coinft

        # Track history for sparkline
        self._force_history.append(abs(fz))
        if len(self._force_history) > self._max_history:
            self._force_history.pop(0)

        # Semi-transparent overlay background
        overlay = frame.copy()

        # Force bar (right side)
        bar_x = w - 60
        bar_w = 40
        bar_top = 30
        bar_bottom = h - 30
        bar_h = bar_bottom - bar_top
        max_force = 50.0  # N full scale

        # Bar background
        cv2.rectangle(overlay, (bar_x - 5, bar_top - 5),
                      (bar_x + bar_w + 5, bar_bottom + 25), (0, 0, 0), -1)

        # Bar fill — green < 10N, yellow 10-25N, red > 25N
        fill_frac = min(abs(fz) / max_force, 1.0)
        fill_h = int(fill_frac * bar_h)
        fill_top = bar_bottom - fill_h

        if abs(fz) < 10:
            color = (0, 200, 0)       # green
        elif abs(fz) < 25:
            color = (0, 200, 200)     # yellow
        else:
            color = (0, 0, 220)       # red

        cv2.rectangle(overlay, (bar_x, fill_top), (bar_x + bar_w, bar_bottom), color, -1)
        cv2.rectangle(overlay, (bar_x, bar_top), (bar_x + bar_w, bar_bottom), (255, 255, 255), 1)

        # Force text
        cv2.putText(overlay, f'{abs(fz):.1f}N', (bar_x - 10, bar_bottom + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(overlay, 'Fz', (bar_x + 8, bar_top - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Sparkline (bottom of frame)
        spark_y0 = h - 60
        spark_h = 40
        spark_x0 = 20
        spark_w = w - 100

        # Sparkline background
        cv2.rectangle(overlay, (spark_x0 - 5, spark_y0 - 5),
                      (spark_x0 + spark_w + 5, spark_y0 + spark_h + 5), (0, 0, 0), -1)

        # Draw sparkline
        if len(self._force_history) > 1:
            pts = []
            for i, val in enumerate(self._force_history):
                x = spark_x0 + int(i * spark_w / self._max_history)
                y = spark_y0 + spark_h - int(min(val / max_force, 1.0) * spark_h)
                pts.append((x, y))
            for i in range(1, len(pts)):
                cv2.line(overlay, pts[i - 1], pts[i], (0, 255, 0), 1)

        # Threshold line at 25N
        thresh_y = spark_y0 + spark_h - int(25.0 / max_force * spark_h)
        cv2.line(overlay, (spark_x0, thresh_y), (spark_x0 + spark_w, thresh_y),
                 (0, 0, 200), 1)
        cv2.putText(overlay, '25N', (spark_x0 + spark_w + 2, thresh_y + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 200), 1)

        # CoinFT readout (top-left)
        if abs(fz_coinft) > 0.01:
            cv2.putText(overlay, f'CoinFT Fz: {fz_coinft:.1f}N', (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

        # Robot Fz readout (top-left, below CoinFT)
        cv2.putText(overlay, f'Robot Fz: {fz:.1f}N', (20, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Blend overlay
        cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, frame)
        return frame

    def _render_force_graph(self):
        """Render a standalone force graph image."""
        graph = np.zeros((360, 640, 3), dtype=np.uint8)
        h, w = graph.shape[:2]

        with self._force_lock:
            fz = self._fz_robot
            fz_coinft = self._fz_coinft

        self._force_history.append(abs(fz))
        if len(self._force_history) > self._max_history:
            self._force_history.pop(0)

        max_force = 50.0

        # Title
        cv2.putText(graph, 'Z-Force Monitor', (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        # Big force readout
        if abs(fz) < 10:
            color = (0, 200, 0)
        elif abs(fz) < 25:
            color = (0, 200, 200)
        else:
            color = (0, 0, 220)
        cv2.putText(graph, f'{abs(fz):.1f} N', (w // 2 - 80, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, color, 3)

        if abs(fz_coinft) > 0.01:
            cv2.putText(graph, f'CoinFT: {fz_coinft:.1f} N', (20, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

        # Force bar (right side)
        bar_x = w - 80
        bar_w = 50
        bar_top = 50
        bar_bottom = h - 40
        bar_h = bar_bottom - bar_top

        fill_frac = min(abs(fz) / max_force, 1.0)
        fill_h = int(fill_frac * bar_h)
        fill_top = bar_bottom - fill_h

        cv2.rectangle(graph, (bar_x, fill_top), (bar_x + bar_w, bar_bottom), color, -1)
        cv2.rectangle(graph, (bar_x, bar_top), (bar_x + bar_w, bar_bottom), (255, 255, 255), 1)

        # Threshold lines
        for thresh, label in [(10, '10'), (25, '25'), (40, '40')]:
            ty = bar_bottom - int(thresh / max_force * bar_h)
            cv2.line(graph, (bar_x - 5, ty), (bar_x + bar_w + 5, ty), (100, 100, 100), 1)
            cv2.putText(graph, label, (bar_x + bar_w + 8, ty + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        # Sparkline (main area)
        spark_x0 = 30
        spark_w = w - 140
        spark_y0 = 140
        spark_h = h - 190

        # Grid
        cv2.rectangle(graph, (spark_x0, spark_y0),
                      (spark_x0 + spark_w, spark_y0 + spark_h), (50, 50, 50), 1)
        for thresh in [10, 25, 40]:
            ty = spark_y0 + spark_h - int(thresh / max_force * spark_h)
            cv2.line(graph, (spark_x0, ty), (spark_x0 + spark_w, ty), (50, 50, 50), 1)

        # 25N threshold in red
        ty25 = spark_y0 + spark_h - int(25.0 / max_force * spark_h)
        cv2.line(graph, (spark_x0, ty25), (spark_x0 + spark_w, ty25), (0, 0, 180), 1)

        # Draw history
        if len(self._force_history) > 1:
            pts = []
            for i, val in enumerate(self._force_history):
                x = spark_x0 + int(i * spark_w / self._max_history)
                y = spark_y0 + spark_h - int(min(val / max_force, 1.0) * spark_h)
                pts.append((x, y))
            for i in range(1, len(pts)):
                cv2.line(graph, pts[i - 1], pts[i], (0, 255, 0), 2)

        return graph

    def _publish_frame(self):
        """Publish camera (clean) on camera port, force graph on graph port."""
        with self._frame_lock:
            frame = self._last_frame.copy() if self._last_frame is not None else None

        # Camera stream (clean, no overlay)
        if frame is not None:
            _, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            self._zmq_camera.send(buf.tobytes())

        # Force graph (separate stream)
        graph = self._render_force_graph()
        _, buf = cv2.imencode('.jpg', graph, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        self._zmq_graph.send(buf.tobytes())

        # Show locally — camera with small force overlay
        if self._show_local:
            if frame is not None:
                display = self._draw_force_overlay(frame)
            else:
                display = graph
            cv2.imshow('Sensor Bridge', display)
            cv2.waitKey(1)


def main():
    parser = argparse.ArgumentParser(description='ROS2 sensor → ZMQ bridge for Quest VR')
    parser.add_argument('--camera-port', type=int, default=10005, help='ZMQ port for camera stream')
    parser.add_argument('--graph-port', type=int, default=15001, help='ZMQ port for force graph')
    parser.add_argument('--no-display', action='store_true', help='Disable local display window')
    args = parser.parse_args()

    rclpy.init()
    node = SensorBridge(camera_port=args.camera_port, graph_port=args.graph_port,
                        show_local=not args.no_display)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # normal shutdown
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
