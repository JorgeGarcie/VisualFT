"""record.launch.py

Launches a ros2 bag recording for a VisionFT trial.
Records all topics uncompressed for maximum frame completeness:
    /coinft/wrench, /flexiv/wrench, /flexiv/tcp_pose, /image_raw

Usage:
    ros2 launch visionft record.launch.py
    ros2 launch visionft record.launch.py trial_name:=my_trial
    ros2 launch visionft record.launch.py output_dir:=/data/trials
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction


TOPICS = [
    '/coinft/wrench',
    '/flexiv/wrench',
    '/flexiv/tcp_pose',
    '/image_raw',
]

DEFAULT_OUTPUT_DIR = os.path.join(os.path.expanduser('~'), 'VisualFT', 'data')


def launch_bag_record(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration

    trial_name = LaunchConfiguration('trial_name').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)

    bag_path = os.path.join(output_dir, trial_name)
    os.makedirs(output_dir, exist_ok=True)

    cmd = [
        'ros2', 'bag', 'record',
        '--output', bag_path,
        '--storage', 'mcap',
        '--max-cache-size', '524288000',  # 500 MB - absorbs image bursts without blocking callbacks
    ] + TOPICS

    return [ExecuteProcess(cmd=cmd, output='screen')]


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
        OpaqueFunction(function=launch_bag_record),
    ])
