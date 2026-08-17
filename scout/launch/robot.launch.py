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
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription
from scout.robot_profile import merged_params, resolve_config_dir


def _fail_fast(node, why):
    """Fail-fast tier (ADR-0015): this node dying leaves the stack lying about
    itself (dead motion chain / dead yaw / dead TF), so its exit shuts the
    whole launch down and compose `restart: unless-stopped` recycles the
    service cleanly instead of limping half-alive."""
    return RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason=why))],
    ))


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


def _safety_setup(context, *args, **kwargs):
    # Last-hop collision monitor (ADR-0016): /cmd_vel_out -> /cmd_vel_safe.
    # Profile-aware (tight_tunnel shrinks the polygons), so resolved here like
    # the camera config. It is a lifecycle node — the manager activates it.
    profile = LaunchConfiguration('profile').perform(context)
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[merged_params('collision_monitor.yaml', profile)],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_safety',
        output='screen',
        parameters=[{'autostart': True,
                     'node_names': ['collision_monitor']}],
    )
    return [
        collision_monitor,
        lifecycle_manager,
        # Both are motion-chain members: with either dead the driver hears
        # nothing on /cmd_vel_safe — undrivable counts as "the stack lies".
        _fail_fast(collision_monitor,
                   'collision_monitor exited — cmd_vel_safe dead'),
        _fail_fast(lifecycle_manager,
                   'lifecycle_manager_safety exited — CM unmanaged'),
    ]


def generate_launch_description():
    scout_share = get_package_share_directory('scout')
    # Bind-mounted repo wins for live edits; share path is the fallback
    # after install (one policy, owned by scout.robot_profile — ADR-0013).
    config = resolve_config_dir()

    enable_joystick = LaunchConfiguration('enable_joystick')

    # Fail-fast tier (ADR-0015): named so OnProcessExit can target them.
    # The camera stays outside the tier — it lives behind rs_launch.py's own
    # include, so there is no Node action here to target; its death surfaces
    # as /imu/data silence -> EKF yaw stale -> health_monitor.
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[os.path.join(config, 'twist_mux.yaml')],
        remappings=[('cmd_vel_out', '/cmd_vel_out')],
    )
    estop = Node(
        package='scout',
        executable='estop',
        output='screen',
    )
    roboclaw = Node(
        package='roboclaw_driver',
        executable='roboclaw_driver_node',
        name='roboclaw_driver',
        output='screen',
        parameters=[os.path.join(config, 'roboclaw.yaml')],
        # /cmd_vel_safe = collision-monitor output (ADR-0016); the driver no
        # longer hears the mux directly.
        remappings=[('/odom', '/wheel_odom'), ('/cmd_vel', '/cmd_vel_safe')],
    )
    gyro_calibrator = Node(
        package='scout',
        executable='gyro_calibrator',
        output='screen',
        remappings=[
            ('imu_in', '/camera/camera/imu'),
            ('imu_out', '/imu/data'),
        ],
    )
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(config, 'ekf.yaml')],
        remappings=[('/odometry/filtered', '/odom')],
    )

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
        twist_mux,

        # Software e-stop: publishes the twist_mux /estop lock heartbeat (5 Hz,
        # fail-safe) and active-brakes on /cmd_vel_stop when engaged.
        # /estop/engage | /estop/release (Trigger).
        estop,

        # Bare node: roboclaw_driver.launch.py cannot remap, and /odom belongs to
        # the EKF. Driven by the mux output, not raw /cmd_vel.
        roboclaw,

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

        # respawn: USB-serial hiccups (CP2102) are recoverable; a lidar gap
        # degrades slam/nav2 but the robot stays drivable (ADR-0015 tier 2).
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[os.path.join(config, 'rplidar.yaml')],
            respawn=True,
            respawn_delay=2.0,
        ),

        OpaqueFunction(function=_camera_setup),

        OpaqueFunction(function=_safety_setup),

        # Bounded, logged escape hatch for the direction-blind PolygonStop
        # lockout (a plain `polygon` STOP zone zeroes cmd_vel regardless of
        # commanded direction — verified 2026-08-17 on hardware, no reverse-
        # to-escape path exists). /collision_monitor/bypass_{engage,release}
        # PAUSE/RESUME collision_monitor via lifecycle_manager_safety; auto-
        # releases after 30 s. See ADR-0016 addendum.
        Node(
            package='scout',
            executable='collision_bypass',
            output='screen',
        ),

        gyro_calibrator,

        ekf,

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

        # respawn: gamepad unplug/replug kills the evdev handle; come back
        # instead of losing teleop until a container restart (ADR-0015 tier 2).
        Node(
            package='scout',
            executable='joystick_teleop',
            output='screen',
            condition=IfCondition(enable_joystick),
            respawn=True,
            respawn_delay=2.0,
        ),

        # Dispatcher-aware nav cancel (/nav/cancel) + consolidated /nav_state
        # feedback from both bt_navigator actions (ADR-0018). Read-only until
        # the cancel service is called; no motion of its own.
        Node(
            package='scout',
            executable='nav_manager',
            output='screen',
        ),

        # rosbag2 record-on-demand: /record/start|stop (Trigger), latched
        # /record/active + /record/path. Inert until called; owns the
        # `ros2 bag record` subprocess + the auto-stop guard (ADR-0017).
        Node(
            package='scout',
            executable='bag_recorder',
            output='screen',
        ),

        # Keepout/speed zones: /zone_cmd (webui polygons) -> maps/zones.json
        # -> derived filter masks + hot-reload of nav2's mask servers
        # (ADR-0019). Like clutter persistence, only meaningful under slam
        # localization/continue — keep map_name in step with slam's map:=.
        Node(
            package='scout',
            executable='zone_manager',
            output='screen',
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
        # respawn: vision-only feature; a crash loses tag refresh, not motion
        # (ADR-0015 tier 2).
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
            respawn=True,
            respawn_delay=2.0,
        ),

        # Fail-fast tier: these dying makes the stack lie (see _fail_fast).
        _fail_fast(twist_mux, 'twist_mux exited — cmd_vel arbitration dead'),
        _fail_fast(estop, 'estop exited — mux lock heartbeat dead'),
        _fail_fast(roboclaw, 'roboclaw_driver exited — drivetrain dead'),
        _fail_fast(gyro_calibrator,
                   'gyro_calibrator exited — EKF yaw source dead'),
        _fail_fast(ekf, 'ekf exited — /odom + odom->base_link dead'),
    ])
