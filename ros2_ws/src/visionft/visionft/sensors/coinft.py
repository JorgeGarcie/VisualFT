#!/usr/bin/env python3

import numpy as np
import serial
import time
import threading
import scipy.io
import onnxruntime as ort
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger


class CoinFTPublisher(Node):
    """CoinFT sensor node with status reporting, watchdog, and recalibration.

    States: INIT → COLLECTING_OFFSET → READY → (recalibrate) → COLLECTING_OFFSET
                                              → (watchdog timeout) → ERROR
    """

    # Sensor status constants
    STATUS_INIT = 'INIT'
    STATUS_COLLECTING_OFFSET = 'COLLECTING_OFFSET'
    STATUS_READY = 'READY'
    STATUS_ERROR = 'ERROR'

    OFFSET_SAMPLE_COUNT = 1500
    WATCHDOG_TIMEOUT_S = 1.0  # no valid packet for 1s → ERROR
    STALE_IDENTICAL_LIMIT = 50  # 50 identical readings (~167ms at 300Hz) → frozen. TODO: untested on live hardware

    def __init__(self):
        super().__init__('coinft_wrench_publisher')

        # Declare parameters
        self.declare_parameter('com_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('frame_id', 'coinft_sensor')
        self.declare_parameter('data_dir', os.path.join(
            os.path.expanduser('~'), 'Teo', 'VisionFT Files', 'data'))
        self.declare_parameter('model_file', 'PFT5-1_MLP_5L_norm_L2.onnx')
        self.declare_parameter('norm_file', 'PFT5-1_norm_constants.mat')

        # Get parameters
        self.com_port = self.get_parameter('com_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        data_dir = self.get_parameter('data_dir').value
        model_file = self.get_parameter('model_file').value
        norm_file = self.get_parameter('norm_file').value

        # Serial protocol constants
        self.START_BYTE = 2
        self.END_BYTE = 3

        # Load ONNX model and normalization constants
        os.chdir(data_dir)
        self.ort_session = ort.InferenceSession(model_file)
        norm_data = scipy.io.loadmat(norm_file)
        self.mu_x = norm_data['norm_const']['mu_x'][0, 0].flatten()
        self.sd_x = norm_data['norm_const']['sd_x'][0, 0].flatten()
        self.mu_y = norm_data['norm_const']['mu_y'][0, 0].flatten()
        self.sd_y = norm_data['norm_const']['sd_y'][0, 0].flatten()

        # Publishers
        self.wrench_pub = self.create_publisher(WrenchStamped, '/coinft/wrench', 10)
        self.status_pub = self.create_publisher(String, '/coinft/status', 10)

        # Recalibrate service
        self.recal_srv = self.create_service(
            Trigger, '/coinft/recalibrate', self._recalibrate_callback)

        # State (protected by lock for cross-thread access)
        self._lock = threading.Lock()
        self._status = self.STATUS_INIT
        self._stop_flag = False
        self._reset_calibration_state()

        # Watchdog tracking
        self._last_valid_packet_time = time.monotonic()
        self._prev_sensor_data = None
        self._identical_count = 0

        # Serial setup
        self.ser = serial.Serial(self.com_port, self.baud_rate, timeout=0.1)
        self._init_sensor()

        # Status publish timer (10 Hz)
        self.create_timer(0.1, self._publish_status)

        # Watchdog timer (5 Hz check) — disabled for debugging
        # self.create_timer(0.2, self._watchdog_check)

        # Start sensor reading thread
        self._set_status(self.STATUS_COLLECTING_OFFSET)
        self.sens_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.sens_thread.start()

    def _reset_calibration_state(self):
        """Reset all calibration state for a fresh offset collection."""
        self._read_count = 0
        self._offset_list = []
        self._offset = None
        self._got_offset = False
        self._ft_bias = None
        self._ft_bias_ready = False
        self._prev_sensor_data = None
        self._identical_count = 0

    def _init_sensor(self):
        """Send init sequence: 'i' → wait → flush → 'q' → read packet size → 's'."""
        self.ser.write(b'i')
        time.sleep(0.2)
        self.ser.reset_input_buffer()

        self.ser.write(b'q')
        time.sleep(0.01)

        packet_size_byte = self.ser.read(1)
        if len(packet_size_byte) < 1:
            self._set_status(self.STATUS_ERROR)
            raise RuntimeError("Failed to read packet size from sensor")
        self.packet_size_excludeStartByte = ord(packet_size_byte) - 1
        self.num_channels = self.packet_size_excludeStartByte // 2

        self.ser.write(b's')
        self.get_logger().info(
            f"Sensor initialized: {self.num_channels} channels, streaming at ~300Hz")

    def _set_status(self, new_status: str):
        """Thread-safe status update with logging on transitions."""
        with self._lock:
            old = self._status
            self._status = new_status
        if old != new_status:
            self.get_logger().info(f"Status: {old} → {new_status}")

    def _get_status(self) -> str:
        with self._lock:
            return self._status

    def _publish_status(self):
        """Timer callback: publish current status at 10 Hz."""
        msg = String()
        msg.data = self._get_status()
        self.status_pub.publish(msg)

    def _watchdog_check(self):
        """Timer callback: check for serial stall."""
        if self._get_status() == self.STATUS_ERROR:
            return
        elapsed = time.monotonic() - self._last_valid_packet_time
        if elapsed > self.WATCHDOG_TIMEOUT_S:
            self.get_logger().error(
                f"Watchdog: no valid packet for {elapsed:.1f}s")
            self._set_status(self.STATUS_ERROR)

    def _recalibrate_callback(self, _request, response):
        """Service callback: reset calibration and re-collect offset."""
        current = self._get_status()
        if current == self.STATUS_COLLECTING_OFFSET:
            response.success = False
            response.message = "Already collecting offset"
            return response

        self.get_logger().info("Recalibration requested — resetting offset collection")
        with self._lock:
            self._reset_calibration_state()
            self._status = self.STATUS_COLLECTING_OFFSET

        response.success = True
        response.message = "Recalibrating — collecting new offset (~4s)"
        return response

    def _read_loop(self):
        """Main sensor read thread: serial → offset → calibrate → publish at ~300Hz."""
        self.get_logger().info("Read loop started")

        while not self._stop_flag and rclpy.ok():
            try:
                byte = self.ser.read(1)
            except serial.SerialException:
                self.get_logger().warn("Serial read failed (disconnected?)")
                self._set_status(self.STATUS_ERROR)
                time.sleep(0.5)
                continue

            if len(byte) == 0:
                continue
            if byte[0] != self.START_BYTE:
                continue

            try:
                data = self.ser.read(self.packet_size_excludeStartByte)
            except serial.SerialException:
                self.get_logger().warn("Serial read failed (disconnected?)")
                self._set_status(self.STATUS_ERROR)
                time.sleep(0.5)
                continue

            if len(data) < self.packet_size_excludeStartByte:
                continue

            if data[-1] != self.END_BYTE:
                continue

            # Valid packet received — feed watchdog
            self._last_valid_packet_time = time.monotonic()

            # If we were in ERROR and packets resume, go back to offset collection
            if self._get_status() == self.STATUS_ERROR:
                self.get_logger().info("Packets resumed after error — re-collecting offset")
                with self._lock:
                    self._reset_calibration_state()
                    self._status = self.STATUS_COLLECTING_OFFSET

            # Parse raw channel data
            sensor_data = np.empty(self.num_channels, dtype=np.float64)
            for i in range(self.num_channels):
                low = data[2 * i]
                high = data[2 * i + 1]
                sensor_data[i] = low + 256 * high

            # Stale data check: identical readings → frozen sensor
            if self._prev_sensor_data is not None and np.array_equal(sensor_data, self._prev_sensor_data):
                self._identical_count += 1
                if self._identical_count == self.STALE_IDENTICAL_LIMIT:
                    self.get_logger().error(
                        f"Stale data: {self._identical_count} identical readings — sensor frozen")
                    self._set_status(self.STATUS_ERROR)
            else:
                self._identical_count = 0
            self._prev_sensor_data = sensor_data.copy()

            # Take lock snapshot of calibration state
            with self._lock:
                read_count = self._read_count
                self._read_count += 1
                got_offset = self._got_offset
                offset = self._offset
                ft_bias_ready = self._ft_bias_ready
                ft_bias = self._ft_bias

            # Phase 1: Collect offset samples
            if read_count < self.OFFSET_SAMPLE_COUNT:
                with self._lock:
                    self._offset_list.append(sensor_data)
                if (read_count + 1) % 500 == 0:
                    self.get_logger().info(
                        f"Collecting offset: {read_count + 1}/{self.OFFSET_SAMPLE_COUNT}")
                continue

            # Phase 2: Compute offset (once)
            if not got_offset:
                with self._lock:
                    self._offset = np.mean(self._offset_list[5:], axis=0)
                    self._got_offset = True
                    offset = self._offset
                    self._offset_list = []  # free memory
                self.get_logger().info("Offset computed, streaming calibrated data")
                self._set_status(self.STATUS_READY)
                continue

            # Phase 3: Calibrate and publish
            offsetted = sensor_data - offset
            x_norm = (offsetted - self.mu_x) / self.sd_x
            x_input = x_norm.astype(np.float32).reshape(1, -1)
            wrench = self.ort_session.run(None, {"input": x_input})[0].flatten()
            wrench = wrench * self.sd_y + self.mu_y

            # Bias zero: first calibrated sample becomes bias
            if not ft_bias_ready:
                with self._lock:
                    self._ft_bias = wrench.copy()
                    self._ft_bias_ready = True
                continue
            else:
                wrench = wrench - ft_bias

            # Publish WrenchStamped
            msg = WrenchStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.wrench.force.x = float(wrench[0])
            msg.wrench.force.y = float(wrench[1])
            msg.wrench.force.z = float(wrench[2])
            msg.wrench.torque.x = float(wrench[3])
            msg.wrench.torque.y = float(wrench[4])
            msg.wrench.torque.z = float(wrench[5])
            self.wrench_pub.publish(msg)

            if self._read_count % 360 == 0:
                self.get_logger().info(
                    f"F: [{wrench[0]:6.2f}, {wrench[1]:6.2f}, {wrench[2]:6.2f}] N | "
                    f"T: [{wrench[3]:6.3f}, {wrench[4]:6.3f}, {wrench[5]:6.3f}] Nm")

    def shutdown(self):
        """Clean shutdown of the node."""
        self.get_logger().info("Shutting down CoinFT publisher...")
        self._stop_flag = True
        if hasattr(self, 'sens_thread'):
            self.sens_thread.join(timeout=2.0)
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        self.get_logger().info("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = CoinFTPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        print(f"Error: {e}\n{traceback.format_exc()}")
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
