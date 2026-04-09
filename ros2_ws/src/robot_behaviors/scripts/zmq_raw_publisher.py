#!/usr/bin/env python3
"""
ZMQ Raw Bytes Publisher — drop-in replacement for leapft's pickle-based publisher.

The leapft pipeline uses pickle to serialize numpy arrays over ZMQ.
C++ cannot read pickle. This module provides a ZMQRawPublisher that sends
raw bytes instead, which C++ teleop can read with a simple memcpy.

Wire format:
  - Hand frame: topic prefix b'transformed_hand_frame ' + 12 doubles as raw bytes (96 bytes)
    Layout: [origin(3), x_axis(3), y_axis(3), z_axis(3)]
  - Pause state: topic prefix b'pause ' + 1 byte (uint8: 0=STOP, 1=CONT)

Usage:
  Option A — Drop-in replacement for leapft publisher:
    Replace ZMQKeypointPublisher with ZMQRawPublisher in TransformHandPositionCoords.

  Option B — Standalone bridge (subscribe pickle, republish raw):
    python zmq_raw_publisher.py [--host localhost] [--sub-port 8089] [--pub-port 8090]

  Option C — Modify leapft directly:
    In leapft/utils/network.py, change ZMQKeypointPublisher.pub_keypoints():
      OLD: buffer = pickle.dumps(keypoint_array, protocol=-1)
      NEW: buffer = np.asarray(keypoint_array, dtype=np.float64).tobytes()
    And for pause (single int):
      OLD: buffer = pickle.dumps(pause_status, protocol=-1)
      NEW: buffer = struct.pack('B', int(pause_status))
"""

import argparse
import struct
import time

import numpy as np
import zmq


class ZMQRawPublisher:
    """Publishes numpy arrays as raw bytes over ZMQ PUB socket.

    Drop-in API replacement for leapft's ZMQKeypointPublisher.
    """

    def __init__(self, host: str, port: int):
        self._host = host
        self._port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        print(f'[ZMQRawPublisher] Binding PUB socket to tcp://{host}:{port}')
        self.socket.bind(f'tcp://{host}:{port}')

    def pub_keypoints(self, data, topic_name: str):
        """Publish data as raw bytes with topic prefix.

        For numpy arrays: sends raw float64 bytes.
        For scalars (int/bool): sends single uint8 byte.
        """
        prefix = f'{topic_name} '.encode('utf-8')

        if isinstance(data, (int, bool, np.integer)):
            # Pause state: single byte
            buffer = struct.pack('B', int(data))
        else:
            # Numpy array: flatten and send as raw doubles
            arr = np.asarray(data, dtype=np.float64).ravel()
            buffer = arr.tobytes()

        self.socket.send(prefix + buffer)

    def stop(self):
        print(f'[ZMQRawPublisher] Closing publisher {self._host}:{self._port}')
        self.socket.close()
        self.context.term()


def run_bridge(host: str, sub_port: int, pub_port: int,
               pause_sub_port: int = 8102, pause_pub_port: int = 8103):
    """Bridge: subscribe to pickle-based leapft publishers, republish as raw bytes.

    This allows running the existing leapft pipeline unchanged while the C++
    teleop subscribes to the raw-bytes republished ports.
    """
    import pickle

    ctx = zmq.Context()

    # Subscribe to pickle-based frame publisher
    frame_sub = ctx.socket(zmq.SUB)
    frame_sub.setsockopt(zmq.CONFLATE, 1)
    frame_sub.connect(f'tcp://{host}:{sub_port}')
    frame_sub.setsockopt(zmq.SUBSCRIBE, b'transformed_hand_frame')
    print(f'[Bridge] Subscribed to pickle frames: tcp://{host}:{sub_port}')

    # Subscribe to pickle-based pause publisher
    pause_sub = ctx.socket(zmq.SUB)
    pause_sub.setsockopt(zmq.CONFLATE, 1)
    pause_sub.connect(f'tcp://{host}:{pause_sub_port}')
    pause_sub.setsockopt(zmq.SUBSCRIBE, b'pause')
    print(f'[Bridge] Subscribed to pickle pause: tcp://{host}:{pause_sub_port}')

    # Republish as raw bytes
    frame_pub = ZMQRawPublisher(host, pub_port)
    pause_pub = ZMQRawPublisher(host, pause_pub_port)
    print(f'[Bridge] Republishing raw frames on port {pub_port}, pause on {pause_pub_port}')

    poller = zmq.Poller()
    poller.register(frame_sub, zmq.POLLIN)
    poller.register(pause_sub, zmq.POLLIN)

    count = 0
    try:
        while True:
            events = dict(poller.poll(timeout=100))

            if frame_sub in events:
                raw = frame_sub.recv()
                topic_prefix = b'transformed_hand_frame '
                if raw.startswith(topic_prefix):
                    payload = raw[len(topic_prefix):]
                    frame = pickle.loads(payload)
                    if frame is not None and len(frame) >= 4:
                        # frame = [origin, x_axis, y_axis, z_axis]
                        flat = np.array(frame, dtype=np.float64).ravel()  # 12 doubles
                        frame_pub.pub_keypoints(flat, 'transformed_hand_frame')
                        count += 1

            if pause_sub in events:
                raw = pause_sub.recv()
                topic_prefix = b'pause '
                if raw.startswith(topic_prefix):
                    payload = raw[len(topic_prefix):]
                    data = pickle.loads(payload)
                    val = int(data) if isinstance(data, (int, np.integer)) else int(data[0])
                    pause_pub.pub_keypoints(val, 'pause')

            if count > 0 and count % 500 == 0:
                print(f'[Bridge] Forwarded {count} frames')

    except KeyboardInterrupt:
        print('\n[Bridge] Shutting down...')
    finally:
        frame_pub.stop()
        pause_pub.stop()
        frame_sub.close()
        pause_sub.close()
        ctx.term()
        print(f'[Bridge] Done. Forwarded {count} frames total.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Bridge: pickle ZMQ -> raw bytes ZMQ for C++ teleop')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--sub-port', type=int, default=8089,
                        help='Port to subscribe (pickle frames from leapft)')
    parser.add_argument('--pub-port', type=int, default=8090,
                        help='Port to republish (raw bytes for C++ teleop)')
    parser.add_argument('--pause-sub-port', type=int, default=8102,
                        help='Port to subscribe (pickle pause from leapft)')
    parser.add_argument('--pause-pub-port', type=int, default=8103,
                        help='Port to republish (raw bytes pause for C++ teleop)')
    args = parser.parse_args()

    run_bridge(args.host, args.sub_port, args.pub_port,
               args.pause_sub_port, args.pause_pub_port)
