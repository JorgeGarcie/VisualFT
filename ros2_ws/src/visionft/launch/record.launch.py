"""record.launch.py

Launches a ros2 bag recording for a VisionFT trial.
Records all topics uncompressed for maximum frame completeness:
    /coinft/wrench, /flexiv/wrench, /flexiv/tcp_pose, /image_raw

By default, waits for CoinFT sensor to be READY before recording.
Disable with wait_for_coinft:=false.

Usage:
    ros2 launch visionft record.launch.py
    ros2 launch visionft record.launch.py trial_name:=my_trial
    ros2 launch visionft record.launch.py wait_for_coinft:=false
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TOPICS = [
    '/coinft/wrench',
    '/coinft/status',
    '/flexiv/wrench',
    '/flexiv/tcp_pose',
    '/image_raw',
]

DEFAULT_OUTPUT_DIR = os.path.join(os.path.expanduser('~'), 'VisualFT', 'data')


def launch_gated_record(context, *args, **kwargs):
    trial_name = LaunchConfiguration('trial_name').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    wait = LaunchConfiguration('wait_for_coinft').perform(context)

    bag_path = os.path.join(output_dir, trial_name)
    os.makedirs(output_dir, exist_ok=True)

    record_cmd = [
        'ros2', 'bag', 'record',
        '--output', bag_path,
        '--storage', 'mcap',
        '--max-cache-size', '524288000',
    ] + TOPICS

    if wait.lower() == 'true':
        # Gate node blocks until /coinft/status == READY, then exits 0
        gate_node = Node(
            package='visionft',
            executable='wait_for_coinft',
            name='coinft_gate',
            output='screen',
        )
        # Start recording only after gate exits successfully
        start_record = RegisterEventHandler(
            OnProcessExit(
                target_action=gate_node,
                on_exit=[ExecuteProcess(cmd=record_cmd, output='screen')],
            )
        )
        return [gate_node, start_record]
    else:
        return [ExecuteProcess(cmd=record_cmd, output='screen')]


def generate_launch_description():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    return LaunchDescription([
        DeclareLaunchArgument(
            'trial_name',
            default_value=f'trial_{timestamp}',
            description='Name for this recording trial (used as bag directory name)',
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value=DEFAULT_OUTPUT_DIR,
            description='Parent directory where the bag will be saved',
        ),
        DeclareLaunchArgument(
            'wait_for_coinft',
            default_value='true',
            description='Wait for CoinFT READY before recording (true/false)',
        ),
        OpaqueFunction(function=launch_gated_record),
    ])
