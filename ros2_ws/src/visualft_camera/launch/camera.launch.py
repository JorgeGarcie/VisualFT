from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gscam2',
            executable='gscam_main',
            name='gscam_main',
            output='screen',
            parameters=[{
                'gscam_config': 'udpsrc port=5000 caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000 ! rtpjitterbuffer latency=0 ! rtph264depay ! avdec_h264 ! videoconvert',
            }]
        ),
    ])