from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gripper_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Gripper control package for stepper motor',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_control = gripper_control.gripper_control:main',
            'gripper_utils = gripper_control.gripper_utils_node:main',
            'gripper_commander = gripper_control.gripper_commander:main',
            'gripper_service_node = gripper_control.gripper_service_node:main',
            'gripper_force_control_node = gripper_control.gripper_force_control:main',
            'gripper_gelsight_control_node = gripper_control.gripper_gelsight_control:main',
            'gripper_close_until_contact = gripper_control.gripper_close_until_contact:main',
        ],
    },

)
