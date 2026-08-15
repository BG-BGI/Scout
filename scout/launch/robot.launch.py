"""Core robot stack: drivetrain, sensors, odom fusion, LED, tilt monitor.

slam / nav2 / foxglove_bridge stay as separate compose services.

    ros2 launch scout robot.launch.py
    ros2 launch scout robot.launch.py enable_joystick:=false
    ros2 launch scout robot.launch.py profile:=tight_tunnel
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from scout.robot_profile import merged_params, resolve_config_dir


def _camera_setup(context, *args, **kwargs):
    # RealSense config carries the profile overlay (tight_tunnel turns depth
    # off); resolved here because the profile is only known at launch time.
    profile = LaunchConfiguration('profile').perform(context)
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py')),
            launch_arguments={
                'config_file': merged_params('realsense.yaml', profile),
            }.items(),
        ),
    ]


def generate_launch_description():
    scout_share = get_package_share_directory('scout')
    # Bind-mounted repo wins for live edits; share path is the fallback
    # after install (one policy, owned by scout.robot_profile — ADR-0013).
    config = resolve_config_dir()

    enable_joystick = LaunchConfiguration('enable_joystick')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_joystick',
            default_value='true',
            description='Start joystick_teleop (set false when Nav2 owns /cmd_vel)',
        ),
        DeclareLaunchArgument(
            'profile',
            default_value='default',
            description='Config profile (default | tight_tunnel) — applies the '
                        'realsense overlay here; pass the same to slam/nav2',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(scout_share, 'launch', 'description.launch.py'),
            ),
        ),

        # cmd_vel arbiter: every motion source publishes its own /cmd_vel_* topic;
        # twist_mux forwards the highest-priority fresh one to /cmd_vel_out. Lives
        # in this container so the whole motion chain (mux + driver) dies together.
        # See scout/config/twist_mux.yaml and ADR-0001.
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[os.path.join(config, 'twist_mux.yaml')],
            remappings=[('cmd_vel_out', '/cmd_vel_out')],
        ),

        # Software e-stop: publishes the twist_mux /estop lock heartbeat (5 Hz,
        # fail-safe) and active-brakes on /cmd_vel_stop when engaged.
        # /estop/engage | /estop/release (Trigger).
        Node(
            package='scout',
            executable='estop',
            output='screen',
        ),

        # Bare node: roboclaw_driver.launch.py cannot remap, and /odom belongs to
        # the EKF. Driven by the mux output, not raw /cmd_vel.
        Node(
            package='roboclaw_driver',
            executable='roboclaw_driver_node',
            name='roboclaw_driver',
            output='screen',
            parameters=[os.path.join(config, 'roboclaw.yaml')],
            remappings=[('/odom', '/wheel_odom'), ('/cmd_vel', '/cmd_vel_out')],
        ),

        Node(
            package='scout',
            executable='battery_monitor',
            output='screen',
        ),

        # Health aggregator: rolls battery + tilt + drivetrain-link liveness
        # into one /diagnostics (DiagnosticArray) for Foxglove + the webui
        # strip — ADR-0014. Read-only; no motion, no device claim.
        Node(
            package='scout',
            executable='health_monitor',
            output='screen',
        ),

        # respawn: a hard SPI fault (beyond the soft retry inside the node)
        # brings the strip back instead of leaving it dark until a container
        # restart — this happened live (TimeoutError in xfer2 killed the node).
        Node(
            package='scout',
            executable='led_node',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[os.path.join(config, 'rplidar.yaml')],
        ),

        OpaqueFunction(function=_camera_setup),

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
            executable='tilt_monitor',
            output='screen',
            remappings=[
                ('imu/data', '/imu/data'),
                ('tilt_alarm', '/tilt_alarm'),
                ('explore/resume', '/explore/resume'),
                ('navigate_to_pose', '/navigate_to_pose'),
            ],
        ),

        # Trick macros (web UI). Inert until /play_trick is called.
        Node(
            package='scout',
            executable='trick_player',
            output='screen',
        ),

        # Lidar follow-me. Inert until /follow_me/start is called.
        Node(
            package='scout',
            executable='follow_me',
            output='screen',
        ),

        # Persistent under-lidar clutter layer (chair bases, shoes). Idles
        # until slam provides map->base_link; saves to /ros_ws/src/maps/.
        Node(
            package='scout',
            executable='clutter_mapper',
            output='screen',
            # Persistence off: slam runs mode:=new, so the map frame resets
            # every boot and a loaded clutter file paints phantom obstacles
            # at wrong coordinates (poisons nav2 planning). Restore the file
            # path once slam runs localization/continue on a saved map.
            parameters=[{'file': ''}],
        ),

        # Waypoint patrol + pose-stamped photo capture (progress docs).
        # Inert until /patrol/start; needs slam + nav2 for motion.
        Node(
            package='scout',
            executable='patrol_capture',
            output='screen',
        ),

        # LED arbitration: sole caller of /set_led_mode. Web UI talks to
        # /set_user_led on this node, never to led_node directly.
        Node(
            package='scout',
            executable='led_status',
            output='screen',
        ),

        Node(
            package='scout',
            executable='joystick_teleop',
            output='screen',
            condition=IfCondition(enable_joystick),
        ),

        # Link-loss watchdog: 5 s without gateway reachability cancels+stashes
        # nav goals, link recovery within 2 min re-dispatches them, longer
        # drops them. Born of the WiFi-dead-zone runaway (2026-08-14).
        Node(
            package='scout',
            executable='link_watchdog',
            output='screen',
        ),

        # Official AprilTag detector (apriltag_ros), single family — see
        # apriltag.yaml for why the all-families fan-out was reverted.
        # /detections + a TF frame per tag off the D455 color stream. Tag
        # MEANING (names/roles/home) lives in scout-skills' registry.
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            parameters=[os.path.join(config, 'apriltag.yaml')],
            remappings=[
                ('image_rect', '/camera/camera/color/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info'),
                ('detections', '/detections'),
            ],
        ),
    ])
