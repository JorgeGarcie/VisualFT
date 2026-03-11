#!/usr/bin/env python3

"""grid_visualizer.py

Paints a live 2D classification grid as the Flexiv TCP moves through
the defined workspace.  Updated at 10 Hz via matplotlib FuncAnimation.

Grid spec:
  X: 0.370 – 0.570 m  (200 mm) → 100 columns at 2 mm resolution
  Y: 0.245 – 0.300 m  ( 55 mm) →  28 rows   at 2 mm resolution

Subscribes:
  /tendon_class       std_msgs/Int32
  /flexiv/tcp_pose    geometry_msgs/PoseStamped

Color mapping:
  -1  unvisited  #cccccc (grey)
   0  none       #f5f5f5 (near white)
   1  single     #2196F3 (blue)
   2  crossed    #FF9800 (orange)
   3  double     #F44336 (red)
"""

import threading

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

# ── workspace constants ──────────────────────────────────────────────
X_MIN, X_MAX = 0.370, 0.570   # 200 mm
Y_MIN, Y_MAX = 0.245, 0.300   # 55 mm
CELL_SIZE = 0.002              # 2 mm

nx = int(round((X_MAX - X_MIN) / CELL_SIZE))   # 100
ny = int(round((Y_MAX - Y_MIN) / CELL_SIZE))   # 28

# ── color table (RGBA, 0–1 range) ────────────────────────────────────
_HEX_COLORS = {
    -1: ('#cccccc', 0.45),   # unvisited – semi-transparent
     0: ('#f5f5f5', 1.0),    # none
     1: ('#2196F3', 1.0),    # single   – blue
     2: ('#FF9800', 1.0),    # crossed  – orange
     3: ('#F44336', 1.0),    # double   – red
}


def _hex_to_rgba(hex_color: str, alpha: float) -> tuple:
    h = hex_color.lstrip('#')
    r, g, b = (int(h[i:i+2], 16) / 255.0 for i in (0, 2, 4))
    return (r, g, b, alpha)


COLOR_MAP = {k: _hex_to_rgba(col, alpha) for k, (col, alpha) in _HEX_COLORS.items()}
CLASS_NAMES = {-1: 'unvisited', 0: 'none', 1: 'single', 2: 'crossed', 3: 'double'}


class GridVisualizer(Node):
    def __init__(self):
        super().__init__('grid_visualizer')

        # Grid: -1 = unvisited
        self.grid = np.full((ny, nx), -1, dtype=np.int8)

        self.lock = threading.Lock()
        self.latest_class: int = -1
        self.tcp_x: float | None = None
        self.tcp_y: float | None = None

        # Subscriptions
        self.create_subscription(Int32, '/tendon_class', self.class_callback, 10)
        self.create_subscription(PoseStamped, '/flexiv/tcp_pose', self.pose_callback, 10)

        self.get_logger().info(
            f'GridVisualizer ready — grid {nx}×{ny} cells '
            f'({(X_MAX-X_MIN)*1000:.0f} mm × {(Y_MAX-Y_MIN)*1000:.0f} mm)'
        )

        self._setup_plot()

    # ── callbacks ────────────────────────────────────────────────────

    def class_callback(self, msg: Int32):
        with self.lock:
            self.latest_class = msg.data

    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y

        with self.lock:
            self.tcp_x = x
            self.tcp_y = y

            if X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX:
                ix = min(int((x - X_MIN) / CELL_SIZE), nx - 1)
                iy = min(int((y - Y_MIN) / CELL_SIZE), ny - 1)
                if self.latest_class >= 0:
                    self.grid[iy, ix] = self.latest_class

    # ── plot setup ───────────────────────────────────────────────────

    def _setup_plot(self):
        bg = '#1a1a2e'
        fg = '#e0e0e0'

        self.fig, self.ax = plt.subplots(figsize=(12, 5))
        self.fig.patch.set_facecolor(bg)
        self.ax.set_facecolor(bg)

        # Initial RGBA image (all unvisited)
        rgba_init = np.full((ny, nx, 4), COLOR_MAP[-1], dtype=float)
        self.im = self.ax.imshow(
            rgba_init,
            origin='lower',
            extent=[X_MIN, X_MAX, Y_MIN, Y_MAX],
            aspect='auto',
            interpolation='nearest',
        )

        # TCP marker (starts off-screen)
        self.tcp_marker, = self.ax.plot([], [], 'w^', markersize=10, zorder=5)

        # Labels / ticks
        self.ax.set_xlabel('TCP X (m)', color=fg)
        self.ax.set_ylabel('TCP Y (m)', color=fg)
        self.ax.set_title('Tendon Classification Grid', color=fg, fontsize=13)
        self.ax.tick_params(colors=fg)
        for spine in self.ax.spines.values():
            spine.set_edgecolor(fg)

        # Grid lines every 10 mm
        x_ticks = np.arange(X_MIN, X_MAX + 0.001, 0.010)
        y_ticks = np.arange(Y_MIN, Y_MAX + 0.001, 0.010)
        self.ax.set_xticks(x_ticks, minor=False)
        self.ax.set_yticks(y_ticks, minor=False)
        self.ax.xaxis.set_tick_params(labelsize=7, colors=fg)
        self.ax.yaxis.set_tick_params(labelsize=7, colors=fg)
        self.ax.grid(True, color='#444466', linewidth=0.5, alpha=0.6)

        # Status text (top-left inside axes)
        self.status_text = self.ax.text(
            0.01, 0.97, '',
            transform=self.ax.transAxes,
            color=fg, fontsize=9, va='top',
            fontfamily='monospace',
        )

        # Legend
        legend_patches = [
            mpatches.Patch(facecolor=_hex_to_rgba('#cccccc', 0.45), label='Unvisited'),
            mpatches.Patch(facecolor=_hex_to_rgba('#f5f5f5', 1.0),  label='None'),
            mpatches.Patch(facecolor=_hex_to_rgba('#2196F3', 1.0),  label='Single'),
            mpatches.Patch(facecolor=_hex_to_rgba('#FF9800', 1.0),  label='Crossed'),
            mpatches.Patch(facecolor=_hex_to_rgba('#F44336', 1.0),  label='Double'),
        ]
        leg = self.ax.legend(
            handles=legend_patches,
            loc='lower right',
            fontsize=8,
            facecolor='#2a2a4e',
            edgecolor=fg,
            labelcolor=fg,
        )

        plt.tight_layout()

        # Animation at 100 ms (10 Hz)
        self.ani = FuncAnimation(self.fig, self._update_frame, interval=100, blit=False)

    # ── animation frame ──────────────────────────────────────────────

    def _update_frame(self, _frame):
        with self.lock:
            grid_copy = self.grid.copy()
            tcp_x = self.tcp_x
            tcp_y = self.tcp_y
            current_class = self.latest_class

        # Rebuild RGBA array from grid values
        rgba = np.empty((ny, nx, 4), dtype=float)
        for val, color in COLOR_MAP.items():
            mask = grid_copy == val
            rgba[mask] = color

        self.im.set_data(rgba)

        # TCP marker
        if tcp_x is not None and tcp_y is not None:
            self.tcp_marker.set_data([tcp_x], [tcp_y])
        else:
            self.tcp_marker.set_data([], [])

        # Status text
        if tcp_x is not None:
            class_label = CLASS_NAMES.get(current_class, 'unknown')
            self.status_text.set_text(
                f'TCP ({tcp_x:.4f}, {tcp_y:.4f})   class: {class_label}'
            )
        else:
            self.status_text.set_text('Waiting for TCP pose…')

        return self.im, self.tcp_marker, self.status_text


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GridVisualizer()

        # Spin ROS in daemon thread; matplotlib runs in main thread
        spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        plt.show()  # blocking

    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
