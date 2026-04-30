from setuptools import find_packages, setup

package_name = 'tendon_classifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/spatial_combined.yaml',
            'config/spatial_image_only.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li2053',
    maintainer_email='georgealex29@gmail.com',
    description='Tendon classification inference (image-only spatial model)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tendon_inference = tendon_classifier.inference_node:main',
        ],
    },
)
