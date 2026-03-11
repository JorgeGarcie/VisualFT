## Launch file for VisionFT nodes

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    coinft_data_dir = os.path.join(
        os.path.expanduser('~'), 'Teo', 'VisionFT Files', 'data')

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

        Node(
            package='visionft',
            executable='visualft_coinft',
            name='visualft_coinft',
            output='screen',
            parameters=[{
                'com_port': '/dev/ttyACM1',
                'baud_rate': 1000000,
                'frame_id': 'coinft_sensor',
                'data_dir': coinft_data_dir,
                'model_file': 'PFT5-1_MLP_5L_norm_L2.onnx',
                'norm_file': 'PFT5-1_norm_constants.mat',
            }]
        ),
    ])