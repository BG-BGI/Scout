"""App-behavior nodes, split out of robot.launch.py for fault isolation.

Patrol capture: inert until triggered, touches no drivetrain or sensor stack
directly. Crashing or restarting this container does not disturb
roboclaw_driver, the camera, lidar, EKF, or tilt_monitor in `robot`.
(trick_player, follow_me, zone_manager and clutter_mapper lived here until
2026-08-24 — the first three removed as unused features, clutter_mapper
replaced by nav2's spatio_temporal_voxel_layer stvl_layer.)

This container is also in the site switch's restart set (ADR-0023): patrol's
site-scoped paths are re-resolved per patrol run, so the restart just clears
in-flight state.

    ros2 launch scout behaviors.launch.py
"""

from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        # Waypoint patrol + pose-stamped photo capture (progress docs).
        # Inert until /patrol/start; needs slam + nav2 for motion. Site paths
        # come from its node defaults (re-resolved per patrol/run).
        Node(
            package='scout',
            executable='patrol_capture',
            output='screen',
        ),
    ])
