import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ),
        launch_arguments={
            'camera_name': 'd455',
            'enable_color': 'true',
            'enable_depth': 'true',
            'rgb_camera.color_profile': '640x480x15',
            'depth_module.depth_profile': '640x480x15',
            # RGBD bundle (color + aligned depth) — off; flip all three on to use it
            'enable_sync': 'false',
            'align_depth.enable': 'false',
            'enable_rgbd': 'false',
            'pointcloud.enable': 'false',
            # IMU (D455): combine accel+gyro into one /imu topic
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
        }.items(),
    )

    return LaunchDescription([
        # Mute librealsense's USB-init WARNING spam; keeps ROS warnings.
        SetEnvironmentVariable('LRS_LOG_LEVEL', 'error'),
        realsense,
        # Bridge the RealSense frame tree (rooted at d455_link) onto the URDF
        # camera mount, correcting the SolidWorks axes to the camera convention.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf',
            arguments=['--frame-id', 'camera_link', '--child-frame-id', 'd455_link',
                       '--roll', '-1.5707963267949',
                       '--pitch', '-1.5707963267949'],
        ),
        # Remove residual gyro bias at boot (hold the robot still ~5s) -> /imu/data_raw
        Node(
            package='scout',
            executable='gyro_calibrator',
            name='gyro_calibrator',
            output='screen',
            remappings=[('imu_in', '/camera/d455/imu'),
                        ('imu_out', '/imu/data_raw')],
        ),
        # Fuse the de-biased accel+gyro into an orientation quaternion -> /imu/data.
        # Reads /imu/data_raw (from gyro_calibrator) by default.
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }],
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
                        '--roll', '-1.5707963267949',
                        '--pitch', '1.5707963267949',
                        '--yaw', '0.0'
                       ],
        ),
    ])
