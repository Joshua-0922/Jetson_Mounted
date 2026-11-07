from glob import glob

from setuptools import find_packages, setup

package_name = 'yolo_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua-0922',
    maintainer_email='nayoonho0922@snu.ac.kr',
    description='YOLO detector node for the IMX219 camera stream',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detector_node = yolo_detector.detector_node:main',
        ],
    },
)
