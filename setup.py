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
        ('share/' + package_name + '/launch', ['launch/visionft.launch.py']),
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
            #'flexiv_publisher = visionft.flexiv:main', # NOT USED 
            'wrench_plotter = visionft.wrench_plotter:main', 
            'plot_csv = visionft.plot_csv:main', # USED FOR PLOTTING CSV
            'flexiv_wrench_publisher = visionft.robot_publisher:main',
            'visualft_coinft = visionft.coinft:main',
        ],
    },
)
