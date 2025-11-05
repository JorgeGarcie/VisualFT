from setuptools import find_packages, setup

package_name = 'visualft_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li2053',
    maintainer_email='georgealex29@gmail.com',
    description='Send Camera Info and images from Camera',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gst_udp_camera_node = visualft_camera.gstream_camera_udp_node:main',
        ],
        
    },
)
