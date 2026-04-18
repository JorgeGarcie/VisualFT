"""scan.launch.py

Launches the C++ scan_controller executable from the robot_behaviors package.
The executable manages its own MCAP bag recording internally — one bag per
scan in the session.  Exits automatically when all scans are done.

Usage:
    ros2 launch visionft scan.launch.py \
        scan_config:=/path/to/scan.yaml \
        robot_config:=/path/to/robot.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scan_config',
            description='Path to scan session YAML config'),
        DeclareLaunchArgument(
            'robot_config',
            description='Path to robot.yaml config'),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'robot_behaviors', 'scan_controller',
                LaunchConfiguration('scan_config'),
                LaunchConfiguration('robot_config'),
            ],
            output='screen',
        ),
    ])
