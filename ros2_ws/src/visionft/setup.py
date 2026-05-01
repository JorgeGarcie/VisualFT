from setuptools import find_packages, setup

package_name = 'visionft'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/visionft.launch.py',
            'launch/record.launch.py',
            'launch/scan.launch.py',
            'launch/teleop.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/example_session.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li2053',
    maintainer_email='georgealex29@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # sensors
            'coinft           = visionft.sensors.coinft:main',
            'usb_camera       = visionft.sensors.usb_camera:main',
            'wait_for_coinft  = visionft.sensors.wait_for_coinft:main',
            # VR streaming
            'tactile_stream   = visionft.streams.tactile_stream:main',
            'scene_stream     = visionft.streams.scene_stream:main',
            # visualization / dashboards
            'led_dashboard    = visionft.viz.led_dashboard:main',
            'grid_visualizer  = visionft.viz.grid_visualizer:main',
            'plot_csv         = visionft.viz.plot_csv:main',
            # tendon classification lives in the 'tendon_classifier' package
        ],
    },
)
