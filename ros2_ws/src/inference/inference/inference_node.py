#!/usr/bin/env python3

"""inference_node.py

Runs the image-only spatial tendon-classification model at camera framerate.
Subscribes to /image_raw only (no force input).
Publishes /tendon_class (Int32) and /tendon_class_name (String).

Background subtraction:
  Call the service ~/set_reference to capture the current frame as the
  reference (background).  Subsequent frames have the reference subtracted
  in raw pixel space (after ToTensor, before Normalize) so the model sees
  only the change from contact.  Call ~/clear_reference to disable.

Inference runs in a dedicated background thread so the ROS executor
is never blocked by the GPU/CPU forward pass.

Classes:
  0 = none
  1 = single
  2 = crossed
  3 = double
"""

import os
import threading

import torch
import torchvision.transforms as T
from cv_bridge import CvBridge
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from ament_index_python.packages import get_package_share_directory

from .config import load_config
from .models_v2 import get_model_v2


CLASS_NAMES = {0: 'none', 1: 'single', 2: 'crossed', 3: 'double'}


class TendonInferenceNode(Node):
    def __init__(self):
        super().__init__('tendon_inference')

        # Parameters
        self.declare_parameter('model_dir', os.path.expanduser('~/model'))
        self.declare_parameter('config_file', 'spatial_image_only.yaml')
        self.declare_parameter('use_cuda', True)

        model_dir = self.get_parameter('model_dir').value
        config_file = self.get_parameter('config_file').value
        use_cuda = self.get_parameter('use_cuda').value

        # Device
        if use_cuda and torch.cuda.is_available():
            self.device = torch.device('cuda')
        else:
            self.device = torch.device('cpu')
        self.get_logger().info(f'Using device: {self.device}')

        # Resolve config path: absolute → use as-is, relative → look in installed share
        if os.path.isabs(config_file):
            config_path = config_file
        else:
            share_dir = get_package_share_directory('inference')
            config_path = os.path.join(share_dir, 'config', config_file)

        config = load_config(config_path)
        model = get_model_v2(config.model)

        # Load weights from model_dir
        weights_name = 'best_image_only.pth'
        weights_path = os.path.join(model_dir, weights_name)
        ckpt = torch.load(weights_path, map_location=self.device, weights_only=True)
        model.load_state_dict(ckpt['model_state_dict'])
        model.eval()
        self.model = model.to(self.device)
        self.get_logger().info(
            f'Model loaded (config={config_path}, weights={weights_path})'
        )

        # Split transform: pre_norm → optional subtraction → normalize
        self.pre_norm = T.Compose([
            T.CenterCrop(1080),
            T.Resize(224, antialias=True),
            T.ToTensor(),   # → [0, 1] float32
        ])
        self.normalize = T.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225],
        )

        self.bridge = CvBridge()

        # Background subtraction reference frame  [3, 224, 224] or None
        self._ref_frame: torch.Tensor | None = None

        # Shared state
        self._lock = threading.Lock()
        self._pending_frame: torch.Tensor | None = None   # preprocessed img tensor
        self._capture_ref: bool = False   # flag: grab next frame as reference

        # Worker thread
        self._frame_event = threading.Event()
        self._stop_event = threading.Event()

        # Publications
        self.pub_class = self.create_publisher(Int32, '/tendon_class', 10)
        self.pub_name = self.create_publisher(String, '/tendon_class_name', 10)

        # Subscription
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # Services
        self.create_service(Trigger, '~/set_reference', self._svc_set_reference)
        self.create_service(Trigger, '~/clear_reference', self._svc_clear_reference)

        # Start inference worker
        self._worker = threading.Thread(target=self._inference_worker, daemon=True)
        self._worker.start()

        self.get_logger().info(
            'TendonInferenceNode ready  '
            '(call ~/set_reference to capture background)'
        )

    # ── image callback (fast — CPU only) ────────────────────────────

    def image_callback(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        pil_img = PILImage.fromarray(cv_img)
        img = self.pre_norm(pil_img)   # [3, 224, 224], range [0, 1]

        with self._lock:
            # If a reference capture was requested, store this frame
            if self._capture_ref:
                self._ref_frame = img.clone()
                self._capture_ref = False
                self.get_logger().info('Reference frame captured')

            ref = self._ref_frame

        # Background subtraction in raw pixel space (before Normalize)
        if ref is not None:
            img = (img - ref).clamp(0.0, 1.0)

        img = self.normalize(img).unsqueeze(0)   # [1, 3, 224, 224]

        with self._lock:
            self._pending_frame = img

        self._frame_event.set()

    # ── inference worker ─────────────────────────────────────────────

    def _inference_worker(self):
        while not self._stop_event.is_set():
            self._frame_event.wait()
            self._frame_event.clear()

            with self._lock:
                frame = self._pending_frame
                self._pending_frame = None

            if frame is None:
                continue

            with torch.no_grad():
                out = self.model(frame.to(self.device))
                pred = int(out.argmax(dim=1).item())

            class_msg = Int32()
            class_msg.data = pred
            self.pub_class.publish(class_msg)

            name_msg = String()
            name_msg.data = CLASS_NAMES.get(pred, 'unknown')
            self.pub_name.publish(name_msg)

            self.get_logger().debug(f'class: {pred} ({name_msg.data})')

    # ── services ─────────────────────────────────────────────────────

    def _svc_set_reference(self, _req, response):
        with self._lock:
            self._capture_ref = True
        response.success = True
        response.message = 'Will capture next frame as reference'
        return response

    def _svc_clear_reference(self, _req, response):
        with self._lock:
            self._ref_frame = None
            self._capture_ref = False
        response.success = True
        response.message = 'Reference frame cleared'
        self.get_logger().info('Reference frame cleared')
        return response

    def destroy_node(self):
        self._stop_event.set()
        self._frame_event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TendonInferenceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
