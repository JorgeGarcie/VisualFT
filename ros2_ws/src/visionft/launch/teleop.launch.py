"""teleop.launch.py

Launches the VR server (Quest → ZMQ) and C++ teleop executable.
Optionally starts MCAP bag recording.

Data flow:
    Quest 3S (PUSH 8087) → vr_server.py (PUB 8089/8102) → teleop (C++)

Prerequisites:
    - Quest 3S running FrankaBot APK, connected to same network
    - visionft.launch.py running (camera + CoinFT sensors) if recording

Usage:
    ros2 launch visionft teleop.launch.py \
        teleop_config:=/path/to/teleop.yaml \
        robot_config:=/path/to/robot.yaml

    # With MCAP recording:
    ros2 launch visionft teleop.launch.py \
        teleop_config:=/path/to/teleop.yaml \
        robot_config:=/path/to/robot.yaml \
        record:=true
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration


RECORD_TOPICS = [
    '/coinft/wrench',
    '/image_raw',
]

VR_SERVER_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..',
    'interview_demos', 'teleop', 'vr_server.py')

DEFAULT_OUTPUT_DIR = os.path.join(os.path.expanduser('~'), 'VisualFT', 'data')


def launch_setup(context, *args, **kwargs):
    teleop_config = LaunchConfiguration('teleop_config').perform(context)
    robot_config = LaunchConfiguration('robot_config').perform(context)
    record = LaunchConfiguration('record').perform(context).lower() == 'true'

    actions = []

    # VR server: Quest PULL → keypoint transform → ZMQ PUB (raw bytes)
    actions.append(ExecuteProcess(
        cmd=['python3', VR_SERVER_PATH],
        output='screen',
        name='vr_server',
    ))

    # C++ teleop executable (subscribes to vr_server on ports 8089/8102)
    actions.append(ExecuteProcess(
        cmd=[
            'ros2', 'run', 'floating_scan', 'teleop',
            teleop_config,
            robot_config,
        ],
        output='screen',
        name='teleop',
    ))

    # Optional MCAP recording
    if record:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        output_dir = LaunchConfiguration('output_dir').perform(context)
        bag_path = os.path.join(output_dir, f'teleop_{timestamp}')
        os.makedirs(output_dir, exist_ok=True)

        actions.append(ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--output', bag_path,
                '--storage', 'mcap',
                '--max-cache-size', '524288000',
            ] + RECORD_TOPICS,
            output='screen',
            name='bag_recorder',
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'teleop_config',
            description='Path to teleop.yaml config'),
        DeclareLaunchArgument(
            'robot_config',
            description='Path to robot.yaml config'),
        DeclareLaunchArgument(
            'record',
            default_value='false',
            description='Enable MCAP bag recording (true/false)'),
        DeclareLaunchArgument(
            'output_dir',
            default_value=DEFAULT_OUTPUT_DIR,
            description='Parent directory for MCAP bags'),
        OpaqueFunction(function=launch_setup),
    ])
