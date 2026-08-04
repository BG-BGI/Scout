"""Core robot stack: drivetrain, sensors, odom fusion, LED.

slam / nav2 / foxglove_bridge stay as separate compose services.

    ros2 launch scout robot.launch.py
    ros2 launch scout robot.launch.py enable_joystick:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scout_share = get_package_share_directory('scout')
    config = os.path.join(scout_share, 'config')
    # Bind-mounted repo wins for live edits; share path is the fallback after install.
    src_config = '/ros_ws/src/scout/config'
    if os.path.isdir(src_config):
        config = src_config

    enable_joystick = LaunchConfiguration('enable_joystick')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_joystick',
            default_value='true',
            description='Start joystick_teleop (set false when Nav2 owns /cmd_vel)',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scout_share, 'launch', 'description.launch.py'),
            ),
        ),

        # Bare node: roboclaw_driver.launch.py cannot remap, and /odom belongs to the EKF.
        Node(
            package='roboclaw_driver',
            executable='roboclaw_driver_node',
            name='roboclaw_driver',
            output='screen',
            parameters=[os.path.join(config, 'roboclaw.yaml')],
            remappings=[('/odom', '/wheel_odom')],
        ),

        Node(
            package='scout',
            executable='battery_monitor',
            output='screen',
        ),

        Node(
            package='scout',
            executable='led_node',
            output='screen',
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[os.path.join(config, 'rplidar.yaml')],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py',
                ),
            ),
            launch_arguments={
                'config_file': os.path.join(config, 'realsense.yaml'),
            }.items(),
        ),

        Node(
            package='scout',
            executable='gyro_calibrator',
            output='screen',
            remappings=[
                ('imu_in', '/camera/camera/imu'),
                ('imu_out', '/imu/data'),
            ],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(config, 'ekf.yaml')],
            remappings=[('/odometry/filtered', '/odom')],
        ),

        Node(
            package='scout',
            executable='joystick_teleop',
            output='screen',
            condition=IfCondition(enable_joystick),
        ),
    ])
