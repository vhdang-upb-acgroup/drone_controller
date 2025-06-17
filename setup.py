from setuptools import find_packages, setup

package_name = 'drone_controller'

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
    maintainer='huyen_rat_admin',
    maintainer_email='vhuyendang@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sensor_node = drone_controller.sensor:main",
            "control_node = drone_controller.control_commands:main",
            "pid_z_controller_node = drone_controller.pid_z_controller:main",
            "pid_pos_controller_node = drone_controller.pid_pos_controller:main",
            "pid_8figure_controller_node = drone_controller.pid_8figure_tracking_controller:main"
        ],
    },
)
