from setuptools import find_packages, setup

package_name = 'system_identification'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Joshua-0922',
    maintainer_email='nachoigpt@gmail.com',
    description='Offboard system-identification flights over MAVLink',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sysid = system_identification.sysid_node:main',
            'sysid_plan = system_identification.maneuvers:main',
        ],
    },
)
