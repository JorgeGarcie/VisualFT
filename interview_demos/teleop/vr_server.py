"""
Minimal VR server — receives Quest hand tracking and republishes via ZMQ.
Equivalent to LeapFT's start_vr_server.sh but standalone.

Runs three components:
  1. OculusVRHandDetector: PULL from Quest APK (port 8087), PUB keypoints (port 8088)
  2. TransformHandPositionCoords: SUB keypoints, PUB transformed frames (port 8089)
  3. Pause relay: PULL pause button (port 8100), PUB pause state (port 8102)

Usage:
  python vr_server.py                          # default host from env or 0.0.0.0
  python vr_server.py --host 192.168.0.62      # explicit host
  LEAPFT_HOST=172.20.10.9 python vr_server.py  # env var
"""

import sys
import os
import multiprocessing
import argparse
import signal
import time

# Ensure user site-packages (for pyzmq)
_user_site = '/home/li2053/.local/lib/python3.10/site-packages'
if _user_site not in sys.path:
    sys.path.append(_user_site)

# Add leapft to path
_leapft_root = os.path.join(os.path.dirname(__file__), '..', 'check')
sys.path.insert(0, _leapft_root)

from leapft.teleop.server.detector.oculus import OculusVRHandDetector
from leapft.teleop.server.detector.keypoint_transform import TransformHandPositionCoords


# Ports from network.yaml
OCULUS_PORT = 8087
KEYPOINT_PORT = 8088
TRANSFORMED_PORT = 8089
BUTTON_PORT = 8095
BUTTON_PUB_PORT = 8093
RESET_PORT = 8100
RESET_PUB_PORT = 8102


def run_detector(host):
    detector = OculusVRHandDetector(
        host, OCULUS_PORT, KEYPOINT_PORT,
        BUTTON_PORT, BUTTON_PUB_PORT,
        RESET_PORT, RESET_PUB_PORT
    )
    detector.stream()


def run_transform(host):
    transformer = TransformHandPositionCoords(
        host, KEYPOINT_PORT, TRANSFORMED_PORT,
        moving_average_limit=4
    )
    transformer.stream()


def main():
    parser = argparse.ArgumentParser(description='Minimal VR server for Quest hand tracking')
    parser.add_argument('--host', default=os.environ.get('LEAPFT_HOST', '0.0.0.0'),
                        help='Host IP to bind (default: 0.0.0.0 = all interfaces)')
    args = parser.parse_args()

    host = args.host
    print(f"=== VR Server starting on {host} ===")
    print(f"  Quest APK sends to port {OCULUS_PORT}")
    print(f"  Transformed keypoints on port {TRANSFORMED_PORT}")
    print(f"  Pause/resume on port {RESET_PUB_PORT}")
    print(f"  Waiting for Quest hand data...")
    print(f"  Ctrl+C to stop\n")

    procs = [
        multiprocessing.Process(target=run_detector, args=(host,), name='VRDetector'),
        multiprocessing.Process(target=run_transform, args=(host,), name='KeypointTransform'),
    ]

    for p in procs:
        p.daemon = True
        p.start()

    def shutdown(sig, frame):
        print("\nShutting down VR server...")
        for p in procs:
            p.terminate()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    # Keep alive
    try:
        while True:
            for p in procs:
                if not p.is_alive():
                    print(f"Process {p.name} died — restarting...")
                    p = multiprocessing.Process(target=p._target, args=p._args, name=p.name)
                    p.daemon = True
                    p.start()
            time.sleep(1)
    except KeyboardInterrupt:
        shutdown(None, None)


if __name__ == '__main__':
    main()
