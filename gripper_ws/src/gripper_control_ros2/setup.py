from setuptools import find_packages, setup

package_name = 'gripper_control_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gripper_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@example.com',
    description='ROS 2 port of a ROS 1 stepper-gripper controller (serial).',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_control_node = gripper_control_ros2.gripper_control_node:main',
        ],
    },
)
