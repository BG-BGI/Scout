"""Nav2 bringup with the scout profile overlay + a depth/pointcloud guard.

Composition path is VENDORED (2026-09-17, ADR-0033): upstream's
navigation_launch.py launches lifecycle_manager_navigation with a hardcoded
parameter dict — no params file, no bond_timeout — so the manager always ran
the ~4 s upstream default and could never inherit amcl.yaml's bond-disable
fix. Under Pi CPU starvation (or a mid-session clock step) heartbeats land
late and the manager silently deactivates all eight nodes while the
container looks healthy — the "nav2 crashes" failure mode. The vendored
list below is a faithful copy of upstream's composable node set (same
plugins, names, remappings) plus bond_timeout: 0.0 on the manager — health
is judged from /nav_state and topic liveness, not bond resets, same as the
localization manager (amcl.yaml, ADR-0028).

use_composition:=false still includes upstream navigation_launch.py for
debugging — its lifecycle manager keeps the default bonds.

(The keepout/speed zone filter wiring — ADR-0019 — was removed 2026-08-24
along with zone_manager; see git history if zones ever come back.)

    ros2 launch scout nav2.launch.py                       # default profile
    ros2 launch scout nav2.launch.py profile:=tight_tunnel
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

from launch import LaunchDescription
from scout.robot_profile import merged_params

LIFECYCLE_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
]


def _plugins_have_depth(params):
    for cm in ('local_costmap', 'global_costmap'):
        node = params.get(cm, {}).get(cm, {}).get('ros__parameters', {})
        if 'stvl_layer' in (node.get('plugins') or []):
            return True
    return False


def _launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration('profile').perform(context)
    params_file = merged_params('nav2.yaml', profile)

    # Coupling guard (ADR-0002): if a costmap still marks via stvl_layer, this
    # profile's camera MUST publish a pointcloud, or the layer starves silently.
    with open(params_file) as f:
        nav2 = yaml.safe_load(f) or {}
    if _plugins_have_depth(nav2):
        with open(merged_params('realsense.yaml', profile)) as f:
            cam = yaml.safe_load(f) or {}
        if cam.get('pointcloud.enable') is False:
            raise RuntimeError(
                'profile %r keeps nav2 stvl_layer but disables the realsense '
                'pointcloud — the depth costmap layer would starve (ADR-0002)'
                % profile)

    nav_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch',
        'navigation_launch.py')
    use_composition = (
        LaunchConfiguration('use_composition').perform(context).lower()
        in ('true', '1'))

    if not use_composition:
        return [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments={
                    'params_file': params_file,
                    'use_sim_time': 'false',
                    'use_composition': 'false',
                    'container_name': 'nav2_container',
                }.items(),
            )]

    # --- vendored composition path (see module docstring) -------------------
    # Mirrors upstream navigation_launch.py's LoadComposableNodes exactly:
    # same plugin classes, node names, topic remappings (cmd_vel ->
    # cmd_vel_nav on controller + behavior_server, velocity_smoother in
    # /cmd_vel_nav out /cmd_vel, and the /tf remap pair). The only
    # intentional delta is lifecycle_manager_navigation's bond_timeout: 0.0.
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={'use_sim_time': 'false', 'autostart': 'true'},
            convert_types=True),
        allow_substs=True)

    def comp(package, plugin, name, extra_remaps=()):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name=name,
            parameters=[configured_params],
            remappings=remappings + list(extra_remaps))

    return [
        Node(
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params],
            remappings=remappings,
            output='screen'),
        LoadComposableNodes(
            target_container='/nav2_container',
            composable_node_descriptions=[
                comp('nav2_controller', 'nav2_controller::ControllerServer',
                     'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
                comp('nav2_smoother', 'nav2_smoother::SmootherServer',
                     'smoother_server'),
                comp('nav2_planner', 'nav2_planner::PlannerServer',
                     'planner_server'),
                comp('nav2_behaviors', 'behavior_server::BehaviorServer',
                     'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
                comp('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator',
                     'bt_navigator'),
                comp('nav2_waypoint_follower',
                     'nav2_waypoint_follower::WaypointFollower',
                     'waypoint_follower'),
                comp('nav2_velocity_smoother',
                     'nav2_velocity_smoother::VelocitySmoother',
                     'velocity_smoother',
                     [('cmd_vel', 'cmd_vel_nav'),
                      ('cmd_vel_smoothed', 'cmd_vel')]),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation',
                    parameters=[{
                        'use_sim_time': False,
                        'autostart': True,
                        'node_names': LIFECYCLE_NODES,
                        'bond_timeout': 0.0,
                    }]),
            ]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='default',
            description='Config profile (default | tight_tunnel)'),
        DeclareLaunchArgument(
            'use_composition', default_value='true',
            description='Load the 8 nav2 nodes into one component container '
                        '(false = one process per node via upstream '
                        'navigation_launch.py, debug only — keeps default '
                        'bonds)'),
        OpaqueFunction(function=_launch_setup),
    ])
