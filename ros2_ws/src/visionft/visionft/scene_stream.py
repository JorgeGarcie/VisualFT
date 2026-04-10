#!/usr/bin/env python3
"""Stream scene camera (3rd person view) to VR headset over ZMQ.

Opens a USB webcam directly and publishes JPEG-compressed frames
over ZMQ for the Quest headset. No ROS2 dependency.

Usage:
    ros2 run visionft scene_stream
    ros2 run visionft scene_stream --device 0 --port 10005
"""

import argparse
import time

import cv2
import numpy as np
import zmq


def main():
    parser = argparse.ArgumentParser(description='Stream USB camera to ZMQ for VR')
    parser.add_argument('--device', type=int, default=0, help='Video device index')
    parser.add_argument('--port', type=int, default=10005, help='ZMQ publish port')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=360)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--quality', type=int, default=70, help='JPEG quality (0-100)')
    args = parser.parse_args()

    # Open camera
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'[CamBridge] ERROR: Could not open /dev/video{args.device}')
        return 1

    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'[CamBridge] Camera opened: /dev/video{args.device} {actual_w}x{actual_h}')

    # ZMQ publisher
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.bind(f'tcp://*:{args.port}')
    print(f'[CamBridge] Streaming on ZMQ port {args.port}')

    frame_count = 0
    target_delay = 1.0 / args.fps

    try:
        while True:
            t0 = time.time()
            ret, frame = cap.read()
            if ret and frame is not None:
                _, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
                sock.send(np.array(buf).tobytes())
                frame_count += 1
                if frame_count == 1:
                    print(f'[CamBridge] First frame sent')
            else:
                time.sleep(0.1)

            elapsed = time.time() - t0
            if elapsed < target_delay:
                time.sleep(target_delay - elapsed)

    except KeyboardInterrupt:
        print(f'\n[CamBridge] Stopped. {frame_count} frames sent.')
    finally:
        cap.release()
        sock.close()
        ctx.term()


if __name__ == '__main__':
    main()
