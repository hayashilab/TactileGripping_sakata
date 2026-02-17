from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'clip_cam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hayashi',
    maintainer_email='hayashi@example.com',
    description='OpenCLIPを使って物体の柔らかさ/硬さを判定するROS2ノード',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'softness_classifier = clip_cam.softness_classifier_node:main',
            'single_shot = clip_cam.single_shot_classifier_node:main',
        ],
    },
)
