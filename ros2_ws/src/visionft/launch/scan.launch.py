"""scan.launch.py

Launches the automated phantom scan node.  The node manages its own MCAP bag
recording internally — one bag per scan in the session.  Exits automatically
when all scans are done.

Usage:
    # Single scan with defaults (no YAML needed)
    ros2 launch visionft scan.launch.py

    # Single scan, override params
    ros2 launch visionft scan.launch.py y_scan_range:=0.05 rz_end:=0.0 rx_end:=0.0

    # Session from YAML config (multiple scans back-to-back)
    ros2 launch visionft scan.launch.py session_config:=/path/to/session.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration

    session_config = LaunchConfiguration('session_config').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    record = LaunchConfiguration('record').perform(context)

    # Build parameters dict — only include scan params if no YAML config
    params = {
        'session_config': session_config,
        'output_dir': output_dir,
        'record': record.lower() in ('true', '1', 'yes'),
    }

    if not session_config:
        # Pass through ROS scan params for single-scan mode
        for key in ['y_scan_range', 'scan_speed', 'contact_force',
                     'search_velocity',
                     'rz_start', 'rz_end', 'rz_step',
                     'rx_start', 'rx_end', 'rx_step']:
            params[key] = float(LaunchConfiguration(key).perform(context))

    scan_node = Node(
        package='visionft',
        executable='scan_node',
        name='scan_node',
        output='screen',
        parameters=[params],
    )

    return [scan_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session_config', default_value='',
                              description='Path to session YAML config'),
        DeclareLaunchArgument('output_dir', default_value='',
                              description='Override output directory'),
        DeclareLaunchArgument('record', default_value='true',
                              description='Enable MCAP bag recording'),
        # Single-scan params (ignored when session_config is set)
        DeclareLaunchArgument('y_scan_range', default_value='0.03'),
        DeclareLaunchArgument('scan_speed', default_value='0.04'),
        DeclareLaunchArgument('contact_force', default_value='20.0'),
        DeclareLaunchArgument('search_velocity', default_value='0.02'),
        DeclareLaunchArgument('rz_start', default_value='0.0'),
        DeclareLaunchArgument('rz_end', default_value='175.0'),
        DeclareLaunchArgument('rz_step', default_value='5.0'),
        DeclareLaunchArgument('rx_start', default_value='0.0'),
        DeclareLaunchArgument('rx_end', default_value='0.0'),
        DeclareLaunchArgument('rx_step', default_value='5.0'),
        OpaqueFunction(function=launch_setup),
    ])
