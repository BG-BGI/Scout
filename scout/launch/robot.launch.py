"""Core robot stack: drivetrain, sensors, odom fusion, LED, tilt monitor.

slam / nav2 / foxglove_bridge stay as separate compose services. App-behavior
nodes (patrol_capture) live in
behaviors.launch.py / the `behaviors` compose service, split out so a crash
or restart there doesn't touch the drivetrain/sensor stack here.

    ros2 launch scout robot.launch.py
    ros2 launch scout robot.launch.py enable_joystick:=false
    ros2 launch scout robot.launch.py profile:=tight_tunnel
"""

import os

import yaml
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
    # Autonomous-branch collision monitor (ADR-0016): /cmd_vel_auto -> /cmd_vel_safe.
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


def _cliff_setup(context, *args, **kwargs):
    # Negative-obstacle detector (ADR-0024). Profile-coupled three ways:
    # the node needs the realsense pointcloud, and the collision monitor's
    # `cliff` source needs the node (its silence reads as a fault and stops
    # autonomy via source_timeout — that is the fail-safe, but only when
    # deliberate). So: pointcloud off -> skip the node AND require the
    # profile's collision_monitor overlay to have stripped the source.
    # Mirror of nav2.launch.py's ADR-0002 stvl<->pointcloud guard.
    profile = LaunchConfiguration('profile').perform(context)
    with open(merged_params('realsense.yaml', profile)) as f:
        cam = yaml.safe_load(f) or {}
    pointcloud_off = cam.get('pointcloud.enable') is False
    with open(merged_params('collision_monitor.yaml', profile)) as f:
        cm = yaml.safe_load(f) or {}
    cm_sources = (cm.get('collision_monitor', {})
                    .get('ros__parameters', {})
                    .get('observation_sources') or [])
    if pointcloud_off:
        if 'cliff' in cm_sources:
            raise RuntimeError(
                'profile %r disables the realsense pointcloud but its '
                'collision_monitor still lists the `cliff` source — with '
                'cliff_detector unlaunched that source starves and '
                'source_timeout freezes autonomy permanently (ADR-0024)'
                % profile)
        return []
    cliff = Node(
        package='scout',
        executable='cliff_detector',
        output='screen',
        parameters=[os.path.join(resolve_config_dir(), 'cliff.yaml')],
        remappings=[
            ('points_in', '/camera/camera/depth/color/points'),
            ('cliff_points', '/cliff/points'),
            ('cliff_stop_points', '/cliff/stop_points'),
        ],
        # Tier 2 (ADR-0015): a crash loses ledge protection, not motion —
        # and while it is down the CM cliff source times out and stops
        # autonomy anyway, so the 2 s respawn gap fails safe.
        respawn=True,
        respawn_delay=2.0,
    )
    return [cliff]


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
    # Stage 1: autonomous-only arbiter -> /cmd_vel_auto -> collision_monitor.
    twist_mux_auto = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux_auto',
        output='screen',
        parameters=[os.path.join(config, 'twist_mux_auto.yaml')],
        remappings=[('cmd_vel_out', '/cmd_vel_auto')],
    )
    # Stage 2 (final): human teleop + collision-guarded autonomous + estop
    # brake -> /cmd_vel_out -> driver. Teleop bypasses the collision monitor.
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
    # Traction derate (docs/traction_control_spec.md): sole writer of the
    # driver's cmd_vel. /cmd_vel_out (final mux output) -> per-side derate
    # from /roboclaw_status current-vs-speed -> /cmd_vel_trac. Uncalibrated
    # (empty curves in traction.yaml) it is a pure passthrough.
    traction_monitor = Node(
        package='scout',
        executable='traction_monitor',
        output='screen',
        parameters=[os.path.join(config, 'traction.yaml')],
        remappings=[
            ('cmd_vel_in', '/cmd_vel_out'),
            ('cmd_vel_out', '/cmd_vel_trac'),
            ('roboclaw_status', '/roboclaw_status'),
            ('traction/derate_left', '/traction/derate_left'),
            ('traction/derate_right', '/traction/derate_right'),
            ('traction/status', '/traction/status'),
        ],
    )
    roboclaw = Node(
        package='roboclaw_driver',
        executable='roboclaw_driver_node',
        name='roboclaw_driver',
        output='screen',
        parameters=[os.path.join(config, 'roboclaw.yaml')],
        # /cmd_vel_trac = /cmd_vel_out (FINAL twist_mux output: human teleop +
        # collision-guarded autonomous + estop brake) after traction_monitor's
        # per-side derate. Teleop bypasses the collision monitor; the CM
        # guards only the autonomous branch (/cmd_vel_safe, the mux's `auto`
        # input). Operator decision 2026-08-17.
        remappings=[('/odom', '/wheel_odom'), ('/cmd_vel', '/cmd_vel_trac')],
    )
    gyro_calibrator = Node(
        package='scout',
        executable='gyro_calibrator',
        output='screen',
        remappings=[
            ('imu_in', '/camera/camera/imu'),
            ('imu_out', '/imu/data'),
            ('imu_out_slow', '/imu/data_slow'),
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
        # See scout/config/twist_mux.yaml, twist_mux_auto.yaml and ADR-0001.
        twist_mux_auto,
        twist_mux,

        # Software e-stop: publishes the twist_mux /estop lock heartbeat (5 Hz,
        # fail-safe) and active-brakes on /cmd_vel_stop when engaged.
        # /estop/engage | /estop/release (Trigger).
        estop,

        # Bare node: roboclaw_driver.launch.py cannot remap, and /odom belongs to
        # the EKF. Driven by traction_monitor's output, not raw /cmd_vel.
        traction_monitor,
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

        OpaqueFunction(function=_cliff_setup),

        # Direction-aware stop zone (narrow sides while driving straight,
        # wide while turning — a plain `polygon` STOP shape is direction-
        # blind by nav2 design, found on hardware passing between two
        # obstacles) + the bounded bypass escape hatch, one node so both
        # features share ownership of collision_monitor's stop-polygon
        # enabled flags without racing. Live-toggles via collision_monitor's
        # own set_parameters — NOT a lifecycle pause (that silently blocks
        # cmd_vel instead of passing it through, also found on hardware).
        # ADR-0016 addendum.
        Node(
            package='scout',
            executable='collision_polygon_manager',
            output='screen',
        ),

        gyro_calibrator,

        ekf,

        Node(
            package='scout',
            executable='tilt_monitor',
            output='screen',
            remappings=[
                # Decimated 20 Hz stream: tilt detection doesn't need 200 Hz, and
                # a python node eats ~20% of a core just deserializing the full rate.
                ('imu/data', '/imu/data_slow'),
                ('tilt_alarm', '/tilt_alarm'),
                ('explore/resume', '/explore/resume'),
                ('navigate_to_pose', '/navigate_to_pose'),
            ],
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

        # zone_manager moved to behaviors.launch.py (ADR-0023): it touches no
        # device, and living there means a site switch restarts it (fresh
        # map_name + masks) without ever restarting the drivetrain.

        # Link-loss watchdog: 5 s without gateway reachability cancels+stashes
        # nav goals, link recovery within 2 min re-dispatches them, longer
        # drops them. Born of the WiFi-dead-zone runaway (2026-08-14).
        Node(
            package='scout',
            executable='link_watchdog',
            output='screen',
        ),

        # 2 Hz color feed for apriltag (2026-08-24): the detector was running
        # on every 15 fps frame at ~16% of a core, and tag refresh (passive
        # tag_watch, register_tag) needs nothing faster than ~2 Hz. C++
        # throttle, so the 15 Hz subscription costs ~nothing. camera_info is
        # NOT throttled — apriltag's exact-time sync matches the 2 Hz images
        # against the full-rate info stream by identical RealSense stamps.
        Node(
            package='topic_tools',
            executable='throttle',
            name='apriltag_color_throttle',
            output='screen',
            arguments=['messages', '/camera/camera/color/image_raw', '2.0',
                       '/apriltag_color_throttled/image_raw'],
        ),

        # ⚠ image_transport's CameraSubscriber derives the camera_info topic
        # from the image topic's namespace and IGNORES a camera_info remap —
        # so the info stream must exist INSIDE the throttled namespace. Full
        # rate relay (info messages are tiny): every 2 Hz image then finds an
        # exactly-stamped partner.
        Node(
            package='topic_tools',
            executable='relay',
            name='apriltag_info_relay',
            output='screen',
            arguments=['/camera/camera/color/camera_info',
                       '/apriltag_color_throttled/camera_info'],
        ),

        # Official AprilTag detector (apriltag_ros), single family — see
        # apriltag.yaml for why the all-families fan-out was reverted.
        # /detections + a TF frame per tag off the D455 color stream (2 Hz
        # throttled — see above). Tag MEANING (names/roles/home) lives in
        # scout-skills' registry.
        # respawn: vision-only feature; a crash loses tag refresh, not motion
        # (ADR-0015 tier 2).
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            output='screen',
            parameters=[os.path.join(config, 'apriltag.yaml')],
            remappings=[
                ('image_rect', '/apriltag_color_throttled/image_raw'),
                ('camera_info', '/camera/camera/color/camera_info'),
                ('detections', '/detections'),
            ],
            respawn=True,
            respawn_delay=2.0,
        ),

        # Flipper Zero bridge (ADR-0025): enable-gated RFID scan loop
        # (/flipper/rfid_enable from the webui RFID panel) + /flipper/cli
        # passthrough. Flipper absent is normal — the node idles and retries.
        # respawn: USB unplug/replug is recoverable; loss degrades RFID only,
        # the robot stays drivable (ADR-0015 tier 2).
        Node(
            package='scout',
            executable='flipper_node',
            output='screen',
            parameters=[os.path.join(config, 'flipper.yaml')],
            remappings=[
                ('flipper/status', '/flipper/status'),
                ('flipper/rfid_enable', '/flipper/rfid_enable'),
                ('flipper/cli', '/flipper/cli'),
                ('rfid/reads', '/rfid/reads'),
            ],
            respawn=True,
            respawn_delay=2.0,
        ),

        # Fail-fast tier: these dying makes the stack lie (see _fail_fast).
        _fail_fast(twist_mux_auto,
                   'twist_mux_auto exited — autonomous cmd_vel arbitration dead'),
        _fail_fast(twist_mux, 'twist_mux exited — cmd_vel arbitration dead'),
        _fail_fast(estop, 'estop exited — mux lock heartbeat dead'),
        _fail_fast(traction_monitor,
                   'traction_monitor exited — driver cmd_vel feed dead'),
        _fail_fast(roboclaw, 'roboclaw_driver exited — drivetrain dead'),
        _fail_fast(gyro_calibrator,
                   'gyro_calibrator exited — EKF yaw source dead'),
        _fail_fast(ekf, 'ekf exited — /odom + odom->base_link dead'),
    ])
