from glob import glob
from setuptools import find_packages, setup

package_name = 'scout'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/behavior_trees',
         glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joystick_teleop = scout.joystick_teleop:main',
            'led_node = scout.led_node:main',
            'gyro_calibrator = scout.gyro_calibrator:main',
            'battery_monitor = scout.battery_monitor:main',
            'wheel_joint_relay = scout.wheel_joint_relay:main',
            'tilt_monitor = scout.tilt_monitor:main',
        ],
    },
)