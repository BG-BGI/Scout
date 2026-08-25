"""SC12: the cliff-stop arithmetic across four files is FROZEN (ADR-0024).

Whether a remembered ledge actually stops the robot depends on inequalities
spanning cliff_detector.py, cliff.yaml, collision_monitor.yaml and nav2.yaml.
They were all documented in prose and none was a test; the launch guard in
robot.launch.py checks only at runtime, so CI never saw a drift. Same
off-ROS pattern as test_profile_constants: parse the source with ast, load
the yamls, assert the inequalities.
"""

import ast
import pathlib

import yaml

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
CONFIG = REPO / 'scout' / 'config'


def _load(name):
    with open(CONFIG / name) as f:
        return yaml.safe_load(f)


def _stop_cluster():
    tree = ast.parse((REPO / 'scout' / 'scout' / 'cliff_detector.py').read_text())
    for node in tree.body:
        if (isinstance(node, ast.Assign)
                and node.targets[0].id == '_STOP_CLUSTER'):
            return ast.literal_eval(node.value)
    raise AssertionError('cliff_detector.py lost _STOP_CLUSTER')


def _cm():
    return _load('collision_monitor.yaml')['collision_monitor']['ros__parameters']


def _bbox(points):
    """collision_monitor polygon flat list [x1, y1, x2, y2, ...] -> bounds."""
    xs, ys = points[0::2], points[1::2]
    return min(xs), max(xs), min(ys), max(ys)


def _inside(p, points):
    x0, x1, y0, y1 = _bbox(points)
    return x0 <= p[0] <= x1 and y0 <= p[1] <= y1


def test_cluster_outnumbers_every_stop_polygon_threshold():
    cluster = _stop_cluster()
    cm = _cm()
    for poly in ('PolygonStopFront', 'PolygonStopRear', 'PolygonStopTurn'):
        assert len(cluster) > cm[poly]['max_points'], (
            '%s max_points must be < the %d-point stop cluster or a ledge '
            'never trips the stop' % (poly, len(cluster)))


def test_cluster_inside_front_and_turn_outside_rear():
    cluster = _stop_cluster()
    cm = _cm()
    front = cm['PolygonStopFront']['points']
    turn = cm['PolygonStopTurn']['points']
    rear = cm['PolygonStopRear']['points']
    for p in cluster:
        assert _inside(p, front), '%s escaped PolygonStopFront' % (p,)
        assert _inside(p, turn), '%s escaped PolygonStopTurn' % (p,)
        # OUTSIDE the rear box, so BackUp recovery can still escape a ledge.
        assert not _inside(p, rear), (
            '%s inside PolygonStopRear — a front ledge would veto the BackUp '
            'escape' % (p,))


def test_cluster_height_inside_cm_cliff_band():
    cluster = _stop_cluster()
    cm = _cm()
    assert 'cliff' in cm['observation_sources']
    src = cm['cliff']
    assert src['topic'] == '/cliff/stop_points'
    for p in cluster:
        assert src['min_height'] <= p[2] <= src['max_height'], (
            'cluster z=%.2f outside the CM cliff source height band' % p[2])


def test_mark_z_inside_both_stvl_cliff_bands():
    mark_z = _load('cliff.yaml')['cliff_detector']['ros__parameters']['mark_z']
    nav2 = _load('nav2.yaml')
    for costmap in ('local_costmap', 'global_costmap'):
        params = nav2[costmap][costmap]['ros__parameters']
        stvl = params['stvl_layer']
        assert 'cliff' in stvl['observation_sources'].split(), (
            '%s stvl_layer lost its cliff source (ADR-0024)' % costmap)
        cliff = stvl['cliff']
        assert cliff['topic'] == '/cliff/points'
        assert (cliff['min_obstacle_height'] <= mark_z
                <= cliff['max_obstacle_height']), (
            'cliff.yaml mark_z=%.2f outside %s stvl cliff band %.2f-%.2f — '
            'odom ledge marks would be filtered out silently'
            % (mark_z, costmap, cliff['min_obstacle_height'],
               cliff['max_obstacle_height']))
