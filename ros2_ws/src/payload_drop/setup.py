from setuptools import find_packages, setup

package_name = 'payload_drop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', ['config/logger_topics.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua-0922',
    maintainer_email='nachoigpt@gmail.com',
    description='Servo payload release over MAVLink and drop-latency measurement',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'drop = payload_drop.drop_node:main',
            'drop_latency = payload_drop.analyze:main',
        ],
    },
)
