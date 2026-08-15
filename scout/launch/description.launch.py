import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory('scout_description'),
        'urdf',
        'scout_description.urdf',
    )
    with open(urdf, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # The wheel joints are continuous, so without joint states
        # robot_state_publisher emits no transform for the wheel links and they
        # disappear from the render. This translates the driver's two encoder
        # joints into the four the URDF names.
        Node(
            package='scout',
            executable='wheel_joint_relay',
            output='screen',
        ),
    ])
