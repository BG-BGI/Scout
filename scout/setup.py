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
        ('share/' + package_name + '/urdf', ['urdf/robot.urdf']),
        ('share/' + package_name + '/meshes', glob('meshes/*.STL')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
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
            'joint_state_controller = scout.joint_state_controller:main',
            'motor_driver = scout.motor_driver:main',
            'joystick_teleop = scout.joystick_teleop:main',
            'gyro_calibrator = scout.gyro_calibrator:main',
        ],
    },
)