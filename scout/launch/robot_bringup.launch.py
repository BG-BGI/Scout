import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('scout')

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_description.launch.py')
        )
    )

    return LaunchDescription([
        robot_description,
        Node(
            package='scout',
            executable='motor_driver',
        ),
        Node(
            package='scout',
            executable='joystick_teleop',
        ),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 256000,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),
        # Mount the scan's 'laser' frame onto the URDF lidar link
        # so the scan plane is corrected without touching the model.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf',
            arguments=['--frame-id', 'lidar1_link', '--child-frame-id', 'laser',
                        '--roll', '1.5707963267949',
                        '--pitch', '1.5707963267949',
                        '--yaw', '0.0'
                       ],
        ),
    ])
