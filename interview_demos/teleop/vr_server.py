#!/usr/bin/env python3
"""
Standalone VR server — receives Quest 3S hand tracking and publishes
transformed hand frames as raw bytes over ZMQ.

No leapft dependency. All logic inlined.

Data flow:
    Quest APK (PUSH text on 8087/8095/8100)
        → this server (PULL, parse, transform)
        → PUB raw bytes on 8089 (hand frame) and 8102 (pause)

Wire format (output):
    - 'transformed_hand_frame ' + 12 doubles (96 bytes)
      Layout: [origin(3), x_axis(3), y_axis(3), z_axis(3)]
    - 'pause ' + 1 byte (uint8: 0=STOP, 1=CONT)

Usage:
    python vr_server.py
    python vr_server.py --host 0.0.0.0
"""

import argparse
import multiprocessing
import os
import signal
import struct
import sys
import time
from collections import deque

# Force unbuffered stdout so subprocess prints show up
os.environ['PYTHONUNBUFFERED'] = '1'

import numpy as np
import zmq

# ── Constants ────────────────────────────────────────────────────────────────

VR_FREQ = 60  # Hz
NUM_KEYPOINTS = 24

# Oculus joint indices
INDEX_KNUCKLE = 6
PINKY_KNUCKLE = 16

# Teleop state flags
TELEOP_CONT = 1
TELEOP_STOP = 0

# Ports (matching Quest APK config)
OCULUS_PORT = 8087
KEYPOINT_PORT = 8088
TRANSFORMED_PORT = 8089
BUTTON_PORT = 8095
BUTTON_PUB_PORT = 8093
RESET_PORT = 8100
RESET_PUB_PORT = 8102

# ── ZMQ helpers ──────────────────────────────────────────────────────────────


def create_pull_socket(ctx, host, port):
    sock = ctx.socket(zmq.PULL)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind(f'tcp://{host}:{port}')
    return sock


def create_pub_socket(ctx, host, port):
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind(f'tcp://{host}:{port}')
    return sock


def create_sub_socket(ctx, host, port, topic):
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.RCVTIMEO, 5000)
    sock.connect(f'tcp://{host}:{port}')
    sock.setsockopt(zmq.SUBSCRIBE, topic.encode('utf-8'))
    return sock


def pub_raw(sock, topic, data):
    """Publish data as raw bytes with topic prefix."""
    prefix = f'{topic} '.encode('utf-8')
    if isinstance(data, (int, bool, np.integer)):
        buffer = struct.pack('B', int(data))
    else:
        buffer = np.asarray(data, dtype=np.float64).ravel().tobytes()
    sock.send(prefix + buffer)


def recv_raw(sock):
    """Receive raw bytes, return numpy array, scalar, or None on timeout."""
    try:
        raw = sock.recv()
    except zmq.Again:
        return None
    # Strip topic prefix (everything up to and including first space)
    idx = raw.index(b' ') + 1
    payload = raw[idx:]
    if len(payload) == 1:
        return struct.unpack('B', payload)[0]
    return np.frombuffer(payload, dtype=np.float64)


# ── Vector ops ───────────────────────────────────────────────────────────────


def normalize(v):
    n = np.linalg.norm(v)
    return v / n if n > 1e-10 else v


def moving_average(value, queue, limit):
    queue.append(value)
    if len(queue) > limit:
        queue.popleft()
    return np.mean(queue, axis=0)


# ── Quest data parsing ───────────────────────────────────────────────────────


def parse_quest_token(raw_bytes):
    """Parse Quest APK text format into keypoint array.

    Format: 'absolute:x,y,z,_|x,y,z,_|...' or 'relative:...'
    Returns: (is_absolute, keypoints (24x3))
    """
    text = raw_bytes.decode().strip()
    is_absolute = text.startswith('absolute')
    vectors_str = text.split(':')[1].strip().split('|')

    keypoints = []
    for vec_str in vectors_str:
        vals = vec_str.split(',')
        keypoints.append([float(vals[0]), float(vals[1]), float(vals[2])])

    return is_absolute, np.array(keypoints[:NUM_KEYPOINTS])


# ── Hand frame transform ────────────────────────────────────────────────────


def vr_to_robot(coord):
    """VR [x,y,z] (Y-up) → Robot [x,z,y] (Z-up)"""
    return np.array([coord[0], coord[2], coord[1]])


def compute_hand_frame(origin, index_knuckle, pinky_knuckle):
    """Compute hand coordinate frame from wrist + knuckle positions.

    Returns [origin(3), x_axis(3), y_axis(3), z_axis(3)] in robot coords.
    """
    origin_r = vr_to_robot(origin)
    index_r = vr_to_robot(index_knuckle)
    pinky_r = vr_to_robot(pinky_knuckle)

    z_axis = normalize(np.cross(index_r, pinky_r))
    x_temp = (index_r + pinky_r) - np.dot(index_r + pinky_r, z_axis) * z_axis
    x_axis = normalize(x_temp)
    y_axis = normalize(np.cross(z_axis, x_axis))

    return np.array([origin_r, x_axis, y_axis, z_axis])


# ── Detector process ────────────────────────────────────────────────────────


def run_detector(host):
    """PULL raw Quest data, parse, PUB as raw bytes."""
    ctx = zmq.Context()

    raw_sock = create_pull_socket(ctx, host, OCULUS_PORT)
    button_sock = create_pull_socket(ctx, host, BUTTON_PORT)
    reset_sock = create_pull_socket(ctx, host, RESET_PORT)

    keypoint_pub = create_pub_socket(ctx, host, KEYPOINT_PORT)
    button_pub = create_pub_socket(ctx, host, BUTTON_PUB_PORT)
    pause_pub = create_pub_socket(ctx, host, RESET_PUB_PORT)

    print(f'[Detector] PULL on {host}:{OCULUS_PORT}, PUB keypoints on {KEYPOINT_PORT}')
    first = False
    dt = 1.0 / VR_FREQ

    try:
        while True:
            t0 = time.monotonic()

            raw_keypoints = raw_sock.recv()
            if not first:
                print('[Detector] Receiving VR data from Quest!')
                first = True

            button_feedback = button_sock.recv()
            pause_status = reset_sock.recv()

            # Parse
            is_abs, keypoints = parse_quest_token(raw_keypoints)
            type_flag = 0 if is_abs else 1
            flat = np.concatenate([[type_flag], keypoints.ravel()])

            pause_val = TELEOP_STOP if pause_status == b'Low' else TELEOP_CONT

            # Publish raw bytes
            pub_raw(keypoint_pub, 'right', flat)
            pub_raw(pause_pub, 'pause', pause_val)

            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        raw_sock.close()
        button_sock.close()
        reset_sock.close()
        keypoint_pub.close()
        button_pub.close()
        pause_pub.close()
        ctx.term()
        print('[Detector] Stopped.')


# ── Transform process ───────────────────────────────────────────────────────


def run_transform(host):
    """SUB raw keypoints, compute hand frame, PUB transformed frame."""
    ctx = zmq.Context()

    sub = create_sub_socket(ctx, host, KEYPOINT_PORT, 'right')
    pub = create_pub_socket(ctx, host, TRANSFORMED_PORT)

    print(f'[Transform] SUB on {host}:{KEYPOINT_PORT}, PUB frames on {TRANSFORMED_PORT}')
    frame_avg_queue = deque()
    avg_limit = 4
    first = False
    dt = 1.0 / VR_FREQ

    try:
        while True:
            t0 = time.monotonic()

            data = recv_raw(sub)
            if data is None:
                continue

            if not first:
                print('[Transform] Receiving keypoints, transforming...')
                first = True

            # data[0] = type flag, data[1:] = 24*3 = 72 floats
            keypoints = data[1:].reshape(NUM_KEYPOINTS, 3)

            # Translate to wrist origin
            translated = keypoints - keypoints[0]

            # Compute hand direction frame
            hand_frame = compute_hand_frame(
                keypoints[0],
                translated[INDEX_KNUCKLE],
                translated[PINKY_KNUCKLE],
            )

            # Moving average for smoothing
            averaged = moving_average(hand_frame, frame_avg_queue, avg_limit)

            # Publish: 12 doubles = [origin, x_axis, y_axis, z_axis]
            pub_raw(pub, 'transformed_hand_frame', averaged)

            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        sub.close()
        pub.close()
        ctx.term()
        print('[Transform] Stopped.')


# ── Main ─────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(description='VR server for Quest 3S hand tracking')
    parser.add_argument('--host', default=os.environ.get('VR_HOST', '0.0.0.0'),
                        help='Host IP to bind (default: 0.0.0.0)')
    args = parser.parse_args()

    host = args.host
    print(f'=== VR Server on {host} ===')
    print(f'  Quest APK → PULL port {OCULUS_PORT}')
    print(f'  Hand frames → PUB port {TRANSFORMED_PORT}')
    print(f'  Pause state → PUB port {RESET_PUB_PORT}')
    print(f'  Ctrl+C to stop\n')

    procs = [
        multiprocessing.Process(target=run_detector, args=(host,), name='Detector'),
        multiprocessing.Process(target=run_transform, args=(host,), name='Transform'),
    ]

    for p in procs:
        p.daemon = True
        p.start()

    def shutdown(sig, frame):
        print('\nShutting down VR server...')
        for p in procs:
            p.terminate()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    try:
        while True:
            for p in procs:
                if not p.is_alive():
                    print(f'Process {p.name} died — shutting down.')
                    shutdown(None, None)
            time.sleep(1)
    except KeyboardInterrupt:
        shutdown(None, None)


if __name__ == '__main__':
    main()
