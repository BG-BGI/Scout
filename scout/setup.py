from glob import glob

from setuptools import find_packages, setup

package_name = 'scout'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        # glob('config/*.yaml') does not recurse — install the profile overlays too.
        ('share/' + package_name + '/config/overlays/tight_tunnel',
         glob('config/overlays/tight_tunnel/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/behavior_trees',
         glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cdrew',
    maintainer_email='cdrew@brasfieldgorrie.com',
    description='Skid-steer robot nodes: teleop, e-stop, LED, battery, '
                'follow, patrol, tricks.',
    license='MIT',
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
            'trick_player = scout.trick_player:main',
            'led_status = scout.led_status:main',
            'follow_me = scout.follow_me:main',
            'clutter_mapper = scout.clutter_mapper:main',
            'patrol_capture = scout.patrol_capture:main',
            'link_watchdog = scout.link_watchdog:main',
            'estop = scout.estop:main',
        ],
    },
)
