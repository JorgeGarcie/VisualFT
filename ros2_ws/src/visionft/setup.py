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
            'wrench_plotter = visionft.wrench_plotter:main',
            'plot_csv = visionft.plot_csv:main',
            'visualft_coinft = visionft.coinft:main',
            # tendon_inference moved to the 'inference' package
            'grid_visualizer  = visionft.grid_visualizer:main',
            'wait_for_coinft  = visionft.wait_for_coinft:main',
            'tactile_stream   = visionft.tactile_stream:main',
            'scene_stream     = visionft.scene_stream:main',
        ],
    },
)
