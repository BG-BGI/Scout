"""App-behavior nodes, split out of robot.launch.py for fault isolation.

Trick macros, follow-me, clutter mapping, patrol capture: all inert until
triggered by a service/topic call, none touch the drivetrain/sensor stack
directly. Crashing or restarting this container does not disturb
roboclaw_driver, the camera, lidar, EKF, or tilt_monitor in `robot`.

    ros2 launch scout behaviors.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
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
    ])
