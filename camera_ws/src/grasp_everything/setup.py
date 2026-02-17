from setuptools import find_packages, setup
from glob import glob

package_name = 'grasp_everything'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/weights', glob('weights/*')),
        ('share/grasp_everything/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hayashi',
    maintainer_email='sakatayoshitaka1031@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stream_ros2 = grasp_everything.get_stream_from_url_ros2:main',
            'data_collector = grasp_everything.data_collector_node:main',
            'data_collection_cli = grasp_everything.data_collection_cli:main',
            'data_collection_test = grasp_everything.data_collection_test:main',
            'data_collection_orchestrator = grasp_everything.data_collection_orchestrator:main',
            'manual_collection = grasp_everything.manual_collection:main',
            'crush_collection = grasp_everything.crush_detection_collection:main',
            'crush_detector = grasp_everything.crush_detector_node:main',
        ],
    },
)
