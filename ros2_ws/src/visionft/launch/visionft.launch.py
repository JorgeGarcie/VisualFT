## Launch file for VisionFT nodes

import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    coinft_data_dir = os.path.join(
        os.path.expanduser('~'), 'Teo', 'VisionFT Files', 'data')

    return LaunchDescription([

        Node(
            package='visionft',
            executable='usb_camera',
            name='usb_camera',
            output='screen',
            arguments=['--device', '2'],
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