#!/usr/bin/env python3
"""LED + CoinFT + Camera dashboard for sensor control and data collection.

PyQt5 GUI with:
- LED control panel (6 NeoPixel LEDs via Arduino serial)
- CoinFT force/torque display (subscribes to ROS2 /coinft/wrench)
- Camera feed from ROS2 /image_raw with 5x2 vision analytics grid
- Synchronized capture and recording across all streams

Requires visionft.launch.py running for sensor data (camera + CoinFT).

Original code by Teo, integrated into visionft package.

Usage:
    ros2 run visionft led_dashboard
    ros2 run visionft led_dashboard --led-port /dev/ttyACM0 --device 2
"""

import argparse
import csv
import os
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import pyqtgraph as pg
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QSlider, QLabel, QColorDialog, QFrame,
                             QGridLayout, QGroupBox)
from PyQt5.QtGui import QColor, QImage, QPixmap, QFont
from PyQt5.QtCore import Qt, QThread, pyqtSignal, pyqtSlot, QObject


# ==========================================
# ROS2 BRIDGE: subscribes to topics, emits Qt signals
# ==========================================
class ROS2Bridge(QObject):
    """Bridges ROS2 callbacks to Qt signals (thread-safe)."""
    wrench_received = pyqtSignal(float, float, float, float, float, float)
    image_received = pyqtSignal(np.ndarray, float)
    tare_result = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._last_image_time = time.time()
        self._node = None  # set after node creation


class DashboardNode(Node):
    """ROS2 node that subscribes to /coinft/wrench and /image_raw."""

    def __init__(self, bridge: ROS2Bridge, image_topic: str, wrench_topic: str):
        super().__init__('led_dashboard')
        self._bridge = bridge
        bridge._node = self

        self.create_subscription(
            WrenchStamped, wrench_topic, self._on_wrench, 1)
        self.create_subscription(
            Image, image_topic, self._on_image, 1)

        self._tare_client = self.create_client(Trigger, '/coinft/recalibrate')

        self.get_logger().info(
            f'Subscribed to {wrench_topic} and {image_topic}')

    def call_tare(self):
        """Call /coinft/recalibrate service asynchronously."""
        if not self._tare_client.service_is_ready():
            self._bridge.tare_result.emit('Recalibrate service not available')
            return
        future = self._tare_client.call_async(Trigger.Request())
        future.add_done_callback(self._on_tare_done)

    def _on_tare_done(self, future):
        try:
            result = future.result()
            self._bridge.tare_result.emit(
                result.message if result.message else 'Sensor recalibrated')
        except Exception as e:
            self._bridge.tare_result.emit(f'Tare failed: {e}')

    def _on_wrench(self, msg: WrenchStamped):
        f = msg.wrench.force
        t = msg.wrench.torque
        self._bridge.wrench_received.emit(f.x, f.y, f.z, t.x, t.y, t.z)

    def _on_image(self, msg: Image):
        if msg.encoding == 'rgb8':
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3).copy()
        elif msg.encoding == 'bgr8':
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3).copy()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif msg.encoding == 'mono8':
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width).copy()
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)
        else:
            return

        now = time.time()
        fps = 1.0 / (now - self._bridge._last_image_time) \
            if (now - self._bridge._last_image_time) > 0 else 0
        self._bridge._last_image_time = now
        self._bridge.image_received.emit(frame, fps)


# ==========================================
# VIDEO PROCESSING (runs on Qt signals from ROS2)
# ==========================================
class VideoProcessor(QObject):
    """Processes frames received from ROS2 and emits display signals."""
    # Row 1: Raw + Crop
    update_raw_frame = pyqtSignal(np.ndarray, float)
    update_crop_frame = pyqtSignal(np.ndarray)
    # Row 2: BGS masks
    update_bgs_mask_raw_frame = pyqtSignal(np.ndarray)
    update_bgs_mask_crop_frame = pyqtSignal(np.ndarray)
    # Row 3: BGS contours on color
    update_bgs_contour_raw_frame = pyqtSignal(np.ndarray)
    update_bgs_contour_crop_frame = pyqtSignal(np.ndarray)
    # Row 4: Gel contours B&W
    update_gel_contour_raw_frame = pyqtSignal(np.ndarray)
    update_gel_contour_crop_frame = pyqtSignal(np.ndarray)
    # Row 5: Tactile diff
    update_diff_frame = pyqtSignal(np.ndarray)
    update_crop_diff_frame = pyqtSignal(np.ndarray)

    status_message = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.take_ref_flag = False
        self.reference_frame = None
        self.reference_crop_frame = None

        self.back_sub_raw = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=25, detectShadows=True)
        self.back_sub_crop = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=16, detectShadows=True)

        self.single_capture_flag = False
        self.is_recording = False
        self.video_writers = None

    def compute_tactile_diff(self, img_current, img_bg, offset=0.5):
        diff = np.int32(img_current) - np.int32(img_bg)
        diff = diff / 255.0 + offset
        diff = np.clip(diff, 0.0, 1.0)
        return np.uint8(diff * 255.0)

    @pyqtSlot(np.ndarray, float)
    def on_frame(self, rgb_frame, fps):
        """Process a frame received from ROS2."""
        # Row 1: Raw & Crop
        self.update_raw_frame.emit(rgb_frame, fps)

        h, w, _ = rgb_frame.shape
        crop_size = min(h, w)
        start_x = (w - crop_size) // 2
        start_y = (h - crop_size) // 2
        cropped = rgb_frame[start_y:start_y + crop_size,
                            start_x:start_x + crop_size]
        resized_224 = cv2.resize(cropped, (224, 224),
                                 interpolation=cv2.INTER_AREA)
        self.update_crop_frame.emit(resized_224)

        # MOG2 masks
        bgs_raw_mask = self.back_sub_raw.apply(rgb_frame)
        bgs_crop_mask = self.back_sub_crop.apply(resized_224)

        # Row 2: Pure MOG2 masks
        self.update_bgs_mask_raw_frame.emit(
            cv2.cvtColor(bgs_raw_mask, cv2.COLOR_GRAY2RGB))
        self.update_bgs_mask_crop_frame.emit(
            cv2.cvtColor(bgs_crop_mask, cv2.COLOR_GRAY2RGB))

        # Contour extraction
        _, thresh_raw = cv2.threshold(bgs_raw_mask, 200, 255,
                                      cv2.THRESH_BINARY)
        contours_raw, _ = cv2.findContours(
            thresh_raw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        _, thresh_crop = cv2.threshold(bgs_crop_mask, 200, 255,
                                       cv2.THRESH_BINARY)
        contours_crop, _ = cv2.findContours(
            thresh_crop, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Row 3: BGS + green contours on color
        bgs_contour_raw = rgb_frame.copy()
        cv2.drawContours(bgs_contour_raw, contours_raw, -1, (0, 255, 0), 2)
        bgs_contour_crop = resized_224.copy()
        cv2.drawContours(bgs_contour_crop, contours_crop, -1, (0, 255, 0), 2)
        self.update_bgs_contour_raw_frame.emit(bgs_contour_raw)
        self.update_bgs_contour_crop_frame.emit(bgs_contour_crop)

        # Row 4: Gel contour (B&W)
        gel_contour_raw = np.zeros_like(rgb_frame)
        cv2.drawContours(gel_contour_raw, contours_raw, -1,
                         (255, 255, 255), 2)
        gel_contour_crop = np.zeros_like(resized_224)
        cv2.drawContours(gel_contour_crop, contours_crop, -1,
                         (255, 255, 255), 2)
        self.update_gel_contour_raw_frame.emit(gel_contour_raw)
        self.update_gel_contour_crop_frame.emit(gel_contour_crop)

        # Row 5: Tactile diff
        if self.take_ref_flag:
            self.reference_frame = rgb_frame.copy()
            self.reference_crop_frame = resized_224.copy()
            self.take_ref_flag = False
            self.status_message.emit('Reference frame captured.')

        diff_frame = None
        diff_crop_frame = None

        if self.reference_frame is not None:
            diff_frame = self.compute_tactile_diff(
                rgb_frame, self.reference_frame)
            self.update_diff_frame.emit(diff_frame)

        if self.reference_crop_frame is not None:
            diff_crop_frame = self.compute_tactile_diff(
                resized_224, self.reference_crop_frame)
            self.update_crop_diff_frame.emit(diff_crop_frame)

        # Single frame capture (all 10 views)
        if self.single_capture_flag:
            self._save_single_capture(
                cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR), resized_224,
                cv2.cvtColor(bgs_raw_mask, cv2.COLOR_GRAY2RGB),
                cv2.cvtColor(bgs_crop_mask, cv2.COLOR_GRAY2RGB),
                bgs_contour_raw, bgs_contour_crop,
                gel_contour_raw, gel_contour_crop,
                diff_frame, diff_crop_frame)
            self.single_capture_flag = False

        # Video recording
        if self.is_recording:
            if self.video_writers is None:
                os.makedirs('recordings', exist_ok=True)
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                fourcc = cv2.VideoWriter_fourcc(*'XVID')
                self.video_writers = {
                    'raw': cv2.VideoWriter(
                        f'recordings/raw_{timestamp}.avi', fourcc, 20.0,
                        (w, h)),
                    'crop': cv2.VideoWriter(
                        f'recordings/crop_{timestamp}.avi', fourcc, 20.0,
                        (224, 224)),
                }
                self.status_message.emit(f'Recording started: {timestamp}')

            self.video_writers['raw'].write(
                cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR))
            self.video_writers['crop'].write(
                cv2.cvtColor(resized_224, cv2.COLOR_RGB2BGR))
        else:
            if self.video_writers is not None:
                self.video_writers['raw'].release()
                self.video_writers['crop'].release()
                self.video_writers = None
                self.status_message.emit('Recording stopped and saved.')

    def _save_single_capture(self, bgr_raw, rgb_crop, mask_raw, mask_crop,
                             cont_raw, cont_crop, gel_raw, gel_crop,
                             diff_raw, diff_crop):
        os.makedirs('captures', exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]

        cv2.imwrite(f'captures/1_raw_{timestamp}.jpg', bgr_raw)
        cv2.imwrite(f'captures/1_crop_{timestamp}.jpg',
                    cv2.cvtColor(rgb_crop, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f'captures/2_mask_raw_{timestamp}.jpg',
                    cv2.cvtColor(mask_raw, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f'captures/2_mask_crop_{timestamp}.jpg',
                    cv2.cvtColor(mask_crop, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f'captures/3_bgs_contour_raw_{timestamp}.jpg',
                    cv2.cvtColor(cont_raw, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f'captures/3_bgs_contour_crop_{timestamp}.jpg',
                    cv2.cvtColor(cont_crop, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f'captures/4_gel_contour_raw_{timestamp}.jpg',
                    cv2.cvtColor(gel_raw, cv2.COLOR_RGB2BGR))
        cv2.imwrite(f'captures/4_gel_contour_crop_{timestamp}.jpg',
                    cv2.cvtColor(gel_crop, cv2.COLOR_RGB2BGR))

        if diff_raw is not None:
            cv2.imwrite(f'captures/5_diff_gray_raw_{timestamp}.jpg',
                        cv2.cvtColor(diff_raw, cv2.COLOR_RGB2BGR))
        if diff_crop is not None:
            cv2.imwrite(f'captures/5_diff_gray_crop_{timestamp}.jpg',
                        cv2.cvtColor(diff_crop, cv2.COLOR_RGB2BGR))

        self.status_message.emit(f'Captured all 10 frames at {timestamp}')

    def capture_reference(self):
        self.take_ref_flag = True

    def capture_single_frame(self):
        self.single_capture_flag = True

    def toggle_recording(self, state):
        self.is_recording = state

    def cleanup(self):
        if self.video_writers is not None:
            self.video_writers['raw'].release()
            self.video_writers['crop'].release()
            self.video_writers = None


# ==========================================
# UI: LED PANEL
# ==========================================
class LEDControlPanel(QWidget):
    def __init__(self, serial_port: str, baud_rate: int):
        super().__init__()
        self.num_leds = 6
        self.led_colors = [(0, 0, 0) for _ in range(self.num_leds)]
        self.serial_conn = None

        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            print(f'Connected to {serial_port}. Waiting for boot...')
            time.sleep(2)
        except Exception:
            print(f'Arduino not found on {serial_port}. '
                  f'Running in simulation mode.')

        self.init_ui()
        self.send_serial_command(99, 20, 0, 0)
        self.set_default_rgbrgb()

    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel('LED Controller')
        title.setFont(QFont('Arial', 14, QFont.Bold))
        layout.addWidget(title)

        brightness_layout = QVBoxLayout()
        self.brightness_label = QLabel('Global Brightness: 20')
        self.brightness_slider = QSlider(Qt.Horizontal)
        self.brightness_slider.setRange(0, 255)
        self.brightness_slider.setValue(20)
        self.brightness_slider.valueChanged.connect(self.on_brightness_changed)
        brightness_layout.addWidget(self.brightness_label)
        brightness_layout.addWidget(self.brightness_slider)
        layout.addLayout(brightness_layout)

        layout.addSpacing(10)

        led_layout = QHBoxLayout()
        self.led_buttons = []
        for i in range(self.num_leds):
            btn = QPushButton(f'LED {i}')
            btn.setFixedSize(60, 60)
            btn.clicked.connect(
                lambda checked, idx=i: self.choose_color(idx))
            self.led_buttons.append(btn)
            led_layout.addWidget(btn)
        layout.addLayout(led_layout)

        layout.addSpacing(10)

        quick_layout = QHBoxLayout()
        btn_red = QPushButton('All Red')
        btn_red.clicked.connect(lambda: self.set_all_color(255, 0, 0))
        btn_green = QPushButton('All Green')
        btn_green.clicked.connect(lambda: self.set_all_color(0, 255, 0))
        btn_blue = QPushButton('All Blue')
        btn_blue.clicked.connect(lambda: self.set_all_color(0, 0, 255))
        btn_default = QPushButton('Default')
        btn_default.clicked.connect(self.set_default_rgbrgb)
        btn_off = QPushButton('Turn Off All')
        btn_off.clicked.connect(lambda: self.set_all_color(0, 0, 0))
        for b in (btn_red, btn_green, btn_blue, btn_default, btn_off):
            quick_layout.addWidget(b)
        layout.addLayout(quick_layout)

        layout.addStretch()
        self.setLayout(layout)

    def send_serial_command(self, index, r, g, b):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(f'{index},{r},{g},{b}\n'.encode('utf-8'))
            time.sleep(0.01)

    def update_button_color(self, idx, r, g, b):
        luminance = 0.299 * r + 0.587 * g + 0.114 * b
        text_color = 'black' if luminance > 128 else 'white'
        self.led_buttons[idx].setStyleSheet(
            f'background-color: #{r:02x}{g:02x}{b:02x}; color: {text_color}; '
            f'font-weight: bold; border-radius: 5px;')

    def on_brightness_changed(self):
        val = self.brightness_slider.value()
        self.brightness_label.setText(f'Global Brightness: {val}')
        self.send_serial_command(99, val, 0, 0)

    def choose_color(self, idx):
        color = QColorDialog.getColor(
            QColor(*self.led_colors[idx]), self, f'LED {idx}')
        if color.isValid():
            self.set_single_led(idx, color.red(), color.green(), color.blue())

    def set_single_led(self, idx, r, g, b):
        self.led_colors[idx] = (r, g, b)
        self.update_button_color(idx, r, g, b)
        self.send_serial_command(idx, r, g, b)

    def set_all_color(self, r, g, b):
        for i in range(self.num_leds):
            self.set_single_led(i, r, g, b)

    def set_default_rgbrgb(self):
        pattern = [(255, 0, 0), (255, 0, 0), (0, 255, 0),
                   (0, 255, 0), (0, 0, 255), (0, 0, 255)]
        for i in range(self.num_leds):
            self.set_single_led(i, *pattern[i])

    def closeEvent(self, event):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        event.accept()


# ==========================================
# UI: FT SENSOR PANEL (live plots from ROS2)
# ==========================================
class FTSensorPanel(QWidget):
    def __init__(self, bridge: ROS2Bridge):
        super().__init__()
        self._bridge = bridge
        self.history_size = 150
        self.data_fx = np.zeros(self.history_size)
        self.data_fy = np.zeros(self.history_size)
        self.data_fz = np.zeros(self.history_size)
        self.data_tx = np.zeros(self.history_size)
        self.data_ty = np.zeros(self.history_size)
        self.data_tz = np.zeros(self.history_size)

        # Recording state
        self.is_recording = False
        self.csv_file = None
        self.csv_writer = None
        self.single_capture_flag = False
        self._latest_wrench = None

        self.init_ui()
        bridge.tare_result.connect(self._on_tare_result)

    def init_ui(self):
        layout = QVBoxLayout()

        header_layout = QHBoxLayout()
        title = QLabel('CoinFT Sensor')
        title.setFont(QFont('Arial', 14, QFont.Bold))
        header_layout.addWidget(title)
        self.btn_tare = QPushButton('Tare (Zero) Sensor')
        self.btn_tare.setStyleSheet(
            'background-color: orange; font-weight: bold;')
        self.btn_tare.clicked.connect(self._on_tare)
        header_layout.addWidget(self.btn_tare)
        layout.addLayout(header_layout)

        self.status_label = QLabel('Status: Waiting for /coinft/wrench...')
        self.status_label.setStyleSheet('color: blue; font-style: italic;')
        layout.addWidget(self.status_label)

        # Numeric readouts
        readout_layout = QHBoxLayout()

        force_group = QGroupBox('Forces (N)')
        force_layout = QHBoxLayout()
        self.lbl_fx = QLabel('Fx: 0.00')
        self.lbl_fy = QLabel('Fy: 0.00')
        self.lbl_fz = QLabel('Fz: 0.00')
        for lbl in (self.lbl_fx, self.lbl_fy, self.lbl_fz):
            lbl.setFont(QFont('Consolas', 12, QFont.Bold))
            force_layout.addWidget(lbl)
        force_group.setLayout(force_layout)

        torque_group = QGroupBox('Torques (Nm)')
        torque_layout = QHBoxLayout()
        self.lbl_tx = QLabel('Tx: 0.000')
        self.lbl_ty = QLabel('Ty: 0.000')
        self.lbl_tz = QLabel('Tz: 0.000')
        for lbl in (self.lbl_tx, self.lbl_ty, self.lbl_tz):
            lbl.setFont(QFont('Consolas', 12, QFont.Bold))
            torque_layout.addWidget(lbl)
        torque_group.setLayout(torque_layout)

        readout_layout.addWidget(force_group)
        readout_layout.addWidget(torque_group)
        layout.addLayout(readout_layout)

        # Live plots
        pg.setConfigOptions(antialias=True)

        self.plot_f = pg.PlotWidget(title='Force (N)')
        self.plot_f.setFixedHeight(180)
        self.plot_f.addLegend()
        self.plot_f.showGrid(x=False, y=True)
        self.curve_fx = self.plot_f.plot(
            pen=pg.mkPen('r', width=2), name='Fx')
        self.curve_fy = self.plot_f.plot(
            pen=pg.mkPen('g', width=2), name='Fy')
        self.curve_fz = self.plot_f.plot(
            pen=pg.mkPen('c', width=2), name='Fz')

        self.plot_t = pg.PlotWidget(title='Torque (Nm)')
        self.plot_t.setFixedHeight(180)
        self.plot_t.addLegend()
        self.plot_t.showGrid(x=False, y=True)
        self.curve_tx = self.plot_t.plot(
            pen=pg.mkPen('r', width=2), name='Tx')
        self.curve_ty = self.plot_t.plot(
            pen=pg.mkPen('g', width=2), name='Ty')
        self.curve_tz = self.plot_t.plot(
            pen=pg.mkPen('c', width=2), name='Tz')

        layout.addWidget(self.plot_f)
        layout.addWidget(self.plot_t)
        self.setLayout(layout)

    @pyqtSlot(float, float, float, float, float, float)
    def update_display(self, fx, fy, fz, tx, ty, tz):
        self._latest_wrench = (fx, fy, fz, tx, ty, tz)

        # Update status on first message
        if self.status_label.text().startswith('Status: Waiting'):
            self.status_label.setText('Status: Receiving wrench data.')

        self.lbl_fx.setText(f'Fx: {fx:6.2f}')
        self.lbl_fy.setText(f'Fy: {fy:6.2f}')
        self.lbl_fz.setText(f'Fz: {fz:6.2f}')
        self.lbl_tx.setText(f'Tx: {tx:6.3f}')
        self.lbl_ty.setText(f'Ty: {ty:6.3f}')
        self.lbl_tz.setText(f'Tz: {tz:6.3f}')

        for data, val in [(self.data_fx, fx), (self.data_fy, fy),
                          (self.data_fz, fz), (self.data_tx, tx),
                          (self.data_ty, ty), (self.data_tz, tz)]:
            data[:-1] = data[1:]
            data[-1] = val

        self.curve_fx.setData(self.data_fx)
        self.curve_fy.setData(self.data_fy)
        self.curve_fz.setData(self.data_fz)
        self.curve_tx.setData(self.data_tx)
        self.curve_ty.setData(self.data_ty)
        self.curve_tz.setData(self.data_tz)

        # Single capture
        if self.single_capture_flag:
            os.makedirs('captures', exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
            with open(f'captures/ft_data_{timestamp}.txt', 'w') as f:
                f.write(f'Fx: {fx:.4f}\nFy: {fy:.4f}\nFz: {fz:.4f}\n')
                f.write(f'Tx: {tx:.4f}\nTy: {ty:.4f}\nTz: {tz:.4f}\n')
            self.single_capture_flag = False

        # CSV recording
        if self.is_recording and self.csv_writer:
            self.csv_writer.writerow(
                [time.time(), fx, fy, fz, tx, ty, tz])

    def _on_tare(self):
        node = self._bridge._node
        if node:
            node.call_tare()
        else:
            self.status_label.setText('Status: ROS2 node not ready')

    @pyqtSlot(str)
    def _on_tare_result(self, msg):
        self.status_label.setText(f'Status: {msg}')

    def capture_single_frame(self):
        self.single_capture_flag = True

    def toggle_recording(self, state):
        self.is_recording = state
        if state:
            os.makedirs('recordings', exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.csv_file = open(
                f'recordings/ft_data_{timestamp}.csv', 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ['timestamp', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz'])
        else:
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
                self.csv_writer = None

    def cleanup(self):
        if self.csv_file:
            self.csv_file.close()


# ==========================================
# UI: CAMERA PANEL (5x2 vision grid)
# ==========================================
class CameraMonitorPanel(QWidget):
    req_capture = pyqtSignal()
    req_record = pyqtSignal(bool)

    def __init__(self, video_processor: VideoProcessor):
        super().__init__()
        self.vp = video_processor
        self.init_ui()

        # Row 1
        self.vp.update_raw_frame.connect(self.update_raw_image)
        self.vp.update_crop_frame.connect(self.update_crop_image)
        # Row 2
        self.vp.update_bgs_mask_raw_frame.connect(
            self.update_bgs_mask_raw_image)
        self.vp.update_bgs_mask_crop_frame.connect(
            self.update_bgs_mask_crop_image)
        # Row 3
        self.vp.update_bgs_contour_raw_frame.connect(
            self.update_bgs_contour_raw_image)
        self.vp.update_bgs_contour_crop_frame.connect(
            self.update_bgs_contour_crop_image)
        # Row 4
        self.vp.update_gel_contour_raw_frame.connect(
            self.update_gel_contour_raw_image)
        self.vp.update_gel_contour_crop_frame.connect(
            self.update_gel_contour_crop_image)
        # Row 5
        self.vp.update_diff_frame.connect(self.update_diff_image)
        self.vp.update_crop_diff_frame.connect(self.update_crop_diff_image)

        self.vp.status_message.connect(self.update_status)

    def init_ui(self):
        layout = QVBoxLayout()
        title = QLabel('Vision Analytics (5x2)')
        title.setFont(QFont('Arial', 14, QFont.Bold))
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setSpacing(5)

        def make_label():
            lbl = QLabel()
            lbl.setMinimumSize(160, 120)
            lbl.setStyleSheet(
                'background-color: black; border: 1px solid gray;')
            lbl.setAlignment(Qt.AlignCenter)
            return lbl

        def make_title(text):
            lbl = QLabel(text)
            lbl.setFont(QFont('Arial', 9, QFont.Bold))
            lbl.setAlignment(Qt.AlignCenter)
            return lbl

        # Row 1
        self.label_raw_title = make_title('1. RAW Input (FPS: --)')
        self.label_raw_video = make_label()
        self.label_crop_title = make_title('ML Crop (224x224)')
        self.label_crop_video = make_label()
        grid.addWidget(self.label_raw_title, 0, 0)
        grid.addWidget(self.label_raw_video, 1, 0)
        grid.addWidget(self.label_crop_title, 0, 1)
        grid.addWidget(self.label_crop_video, 1, 1)

        # Row 2
        self.label_bgs_mask_raw_title = make_title('2. BGS Mask (MOG2 RAW)')
        self.label_bgs_mask_raw_video = make_label()
        self.label_bgs_mask_crop_title = make_title('BGS Mask (MOG2 Crop)')
        self.label_bgs_mask_crop_video = make_label()
        grid.addWidget(self.label_bgs_mask_raw_title, 2, 0)
        grid.addWidget(self.label_bgs_mask_raw_video, 3, 0)
        grid.addWidget(self.label_bgs_mask_crop_title, 2, 1)
        grid.addWidget(self.label_bgs_mask_crop_video, 3, 1)

        # Row 3
        self.label_bgs_cont_raw_title = make_title('3. BGS + Green Contours')
        self.label_bgs_cont_raw_video = make_label()
        self.label_bgs_cont_crop_title = make_title('BGS + Green Contours')
        self.label_bgs_cont_crop_video = make_label()
        grid.addWidget(self.label_bgs_cont_raw_title, 4, 0)
        grid.addWidget(self.label_bgs_cont_raw_video, 5, 0)
        grid.addWidget(self.label_bgs_cont_crop_title, 4, 1)
        grid.addWidget(self.label_bgs_cont_crop_video, 5, 1)

        # Row 4
        self.label_gel_cont_raw_title = make_title('4. Gel Contour (B&W RAW)')
        self.label_gel_cont_raw_video = make_label()
        self.label_gel_cont_crop_title = make_title('Gel Contour (B&W Crop)')
        self.label_gel_cont_crop_video = make_label()
        grid.addWidget(self.label_gel_cont_raw_title, 6, 0)
        grid.addWidget(self.label_gel_cont_raw_video, 7, 0)
        grid.addWidget(self.label_gel_cont_crop_title, 6, 1)
        grid.addWidget(self.label_gel_cont_crop_video, 7, 1)

        # Row 5
        self.label_diff_title = make_title('5. Tactile Diff (Gray Ref)')
        self.label_diff_video = make_label()
        self.label_crop_diff_title = make_title('Tactile Diff (Gray Ref)')
        self.label_crop_diff_video = make_label()
        grid.addWidget(self.label_diff_title, 8, 0)
        grid.addWidget(self.label_diff_video, 9, 0)
        grid.addWidget(self.label_crop_diff_title, 8, 1)
        grid.addWidget(self.label_crop_diff_video, 9, 1)

        layout.addLayout(grid)
        layout.addSpacing(10)

        # Controls
        controls_layout = QHBoxLayout()

        self.btn_take_ref = QPushButton('Take Static Ref')
        self.btn_take_ref.setMinimumHeight(40)
        self.btn_take_ref.clicked.connect(self.vp.capture_reference)

        self.btn_capture = QPushButton('Capture Snap (10 Frames + FT)')
        self.btn_capture.setStyleSheet(
            'background-color: #4CAF50; color: white; font-weight: bold;')
        self.btn_capture.setMinimumHeight(40)
        self.btn_capture.clicked.connect(self.on_capture)

        self.btn_record = QPushButton('Start Recording')
        self.btn_record.setMinimumHeight(40)
        self.btn_record.setCheckable(True)
        self.btn_record.setStyleSheet(
            'QPushButton:checked { background-color: red; color: white; '
            'font-weight: bold; }')
        self.btn_record.toggled.connect(self.on_record)

        controls_layout.addWidget(self.btn_take_ref)
        controls_layout.addWidget(self.btn_capture)
        controls_layout.addWidget(self.btn_record)
        layout.addLayout(controls_layout)

        self.status_label = QLabel('Status: Waiting for /image_raw...')
        self.status_label.setStyleSheet('color: blue; font-style: italic;')
        layout.addWidget(self.status_label)

        layout.addStretch()
        self.setLayout(layout)

    def on_capture(self):
        self.vp.capture_single_frame()
        self.req_capture.emit()

    def on_record(self, checked):
        self.btn_record.setText(
            'Stop Recording' if checked else 'Start Recording')
        self.vp.toggle_recording(checked)
        self.req_record.emit(checked)

    @pyqtSlot(str)
    def update_status(self, msg):
        self.status_label.setText(f'Status: {msg}')

    def convert_cv_qt(self, cv_img, target_width=320, target_height=240):
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        q_img = QImage(cv_img.data, w, h, bytes_per_line,
                       QImage.Format_RGB888).copy()
        pixmap = QPixmap.fromImage(q_img)
        return pixmap.scaled(target_width, target_height, Qt.KeepAspectRatio)

    @pyqtSlot(np.ndarray, float)
    def update_raw_image(self, cv_img, fps):
        self.label_raw_title.setText(f'1. RAW Input (FPS: {fps:.1f})')
        self.label_raw_video.setPixmap(self.convert_cv_qt(cv_img))

    @pyqtSlot(np.ndarray)
    def update_crop_image(self, img):
        self.label_crop_video.setPixmap(self.convert_cv_qt(img, 224, 224))

    @pyqtSlot(np.ndarray)
    def update_bgs_mask_raw_image(self, img):
        self.label_bgs_mask_raw_video.setPixmap(self.convert_cv_qt(img))

    @pyqtSlot(np.ndarray)
    def update_bgs_mask_crop_image(self, img):
        self.label_bgs_mask_crop_video.setPixmap(
            self.convert_cv_qt(img, 224, 224))

    @pyqtSlot(np.ndarray)
    def update_bgs_contour_raw_image(self, img):
        self.label_bgs_cont_raw_video.setPixmap(self.convert_cv_qt(img))

    @pyqtSlot(np.ndarray)
    def update_bgs_contour_crop_image(self, img):
        self.label_bgs_cont_crop_video.setPixmap(
            self.convert_cv_qt(img, 224, 224))

    @pyqtSlot(np.ndarray)
    def update_gel_contour_raw_image(self, img):
        self.label_gel_cont_raw_video.setPixmap(self.convert_cv_qt(img))

    @pyqtSlot(np.ndarray)
    def update_gel_contour_crop_image(self, img):
        self.label_gel_cont_crop_video.setPixmap(
            self.convert_cv_qt(img, 224, 224))

    @pyqtSlot(np.ndarray)
    def update_diff_image(self, img):
        self.label_diff_video.setPixmap(self.convert_cv_qt(img))

    @pyqtSlot(np.ndarray)
    def update_crop_diff_image(self, img):
        self.label_crop_diff_video.setPixmap(
            self.convert_cv_qt(img, 224, 224))


# ==========================================
# MAIN DASHBOARD
# ==========================================
class MainApp(QWidget):
    def __init__(self, led_port: str, led_baud: int,
                 bridge: ROS2Bridge, video_processor: VideoProcessor):
        super().__init__()
        self.setWindowTitle('LED + CoinFT + Vision Dashboard')
        self._bridge = bridge
        self._vp = video_processor

        main_layout = QHBoxLayout()

        left_layout = QVBoxLayout()
        self.led_panel = LEDControlPanel(led_port, led_baud)
        self.ft_panel = FTSensorPanel(bridge)
        left_layout.addWidget(self.led_panel)
        left_layout.addWidget(self.ft_panel)

        self.camera_panel = CameraMonitorPanel(video_processor)

        # Wire ROS2 bridge → panels
        bridge.wrench_received.connect(self.ft_panel.update_display)
        bridge.image_received.connect(video_processor.on_frame)

        # Sync capture/record buttons to FT panel
        self.camera_panel.req_capture.connect(
            self.ft_panel.capture_single_frame)
        self.camera_panel.req_record.connect(
            self.ft_panel.toggle_recording)

        line = QFrame()
        line.setFrameShape(QFrame.VLine)
        line.setFrameShadow(QFrame.Sunken)

        main_layout.addLayout(left_layout)
        main_layout.addWidget(line)
        main_layout.addWidget(self.camera_panel)

        self.setLayout(main_layout)

    def closeEvent(self, event):
        self.led_panel.closeEvent(event)
        self.ft_panel.cleanup()
        self._vp.cleanup()
        event.accept()


def main():
    parser = argparse.ArgumentParser(
        description='LED + CoinFT + Camera dashboard (ROS2)')
    # LED args
    parser.add_argument('--led-port', default='/dev/ttyACM0',
                        help='Arduino LED serial port (default: /dev/ttyACM0)')
    parser.add_argument('--led-baud', type=int, default=115200,
                        help='Arduino baud rate (default: 115200)')
    # ROS2 topic args
    parser.add_argument('--image-topic', default='/image_raw',
                        help='ROS2 image topic (default: /image_raw)')
    parser.add_argument('--wrench-topic', default='/coinft/wrench',
                        help='ROS2 wrench topic (default: /coinft/wrench)')
    args = parser.parse_args()

    # Init ROS2
    rclpy.init()
    bridge = ROS2Bridge()
    video_processor = VideoProcessor()
    node = DashboardNode(bridge, args.image_topic, args.wrench_topic)

    # Spin ROS2 in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Run Qt app in main thread
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = MainApp(
        led_port=args.led_port,
        led_baud=args.led_baud,
        bridge=bridge,
        video_processor=video_processor,
    )
    window.showMaximized()

    exit_code = app.exec_()

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
