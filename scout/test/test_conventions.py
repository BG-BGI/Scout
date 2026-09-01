"""Repo conventions as AST rules (SC1–SC7, SC11) — see docs/adr/0013-conventions-as-tests.md.

Like test_core_purity.py these parse SOURCE (ast.parse), never import ROS, and
run under bare pytest anywhere. Each assertion message states the fix.

Waivers: add the file to the rule's ALLOW dict with a reason (code-reviewed,
visible). A bare entry with an empty reason fails.
"""

import ast
import pathlib

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
PKG = REPO / 'scout' / 'scout'
CORE = PKG / 'core'
SKILLS = REPO / 'docker' / 'scout-skills'
SCRIPTS = REPO / 'scripts'


def _py_files(root, exclude=()):
    return [p for p in sorted(root.glob('*.py')) if p.name not in exclude]


def _tree(path):
    return ast.parse(path.read_text(), filename=str(path))


def _check_allow(allow):
    bare = [k for k, reason in allow.items() if not str(reason).strip()]
    assert not bare, 'ALLOW entries need a reason: %s' % bare


def _rel(path):
    return str(path.relative_to(REPO))


# --- SC5: no hand-rolled quaternion math -------------------------------------
# scout.core.geometry (and its vendored twin in docker/scout-skills) own the
# planar shortcuts. patrol_capture once re-derived sin/cos inline and the
# skills container forked 2*atan2(z,w) three times — this stops the next copy.

SC5_ALLOW = {
    'scout/scout/core/geometry.py': 'the implementation itself',
    'docker/scout-skills/geometry.py': 'vendored implementation (SC10-synced)',
}


def _is_zw_pair(a, b):
    def leaf(n, name):
        return ((isinstance(n, ast.Attribute) and n.attr == name)
                or (isinstance(n, ast.Subscript)
                    and isinstance(n.slice, ast.Constant) and n.slice.value == name))
    return leaf(a, 'z') and leaf(b, 'w')


def test_sc5_no_hand_rolled_quaternions():
    _check_allow(SC5_ALLOW)
    offenders = []
    for path in (_py_files(PKG) + _py_files(CORE) + _py_files(SKILLS)
                 + _py_files(SCRIPTS)):
        if _rel(path) in SC5_ALLOW:
            continue
        for node in ast.walk(_tree(path)):
            # 2*atan2(q.z, q.w) / atan2(q["z"], q["w"]) -> planar_yaw
            if (isinstance(node, ast.Call) and len(node.args) == 2
                    and getattr(node.func, 'attr', getattr(node.func, 'id', ''))
                    == 'atan2' and _is_zw_pair(node.args[0], node.args[1])):
                offenders.append('%s:%d hand-rolled planar_yaw'
                                 % (_rel(path), node.lineno))
            # x.z = sin(yaw/2) + x.w = cos(yaw/2) pairs -> yaw_to_quat_zw
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                trig = {'sin': None, 'cos': None}
                for sub in ast.walk(node):
                    if not (isinstance(sub, ast.Assign)
                            and isinstance(sub.targets[0], ast.Attribute)):
                        continue
                    calls = [c for c in ast.walk(sub.value)
                             if isinstance(c, ast.Call)]
                    for c in calls:
                        fn = getattr(c.func, 'attr', getattr(c.func, 'id', ''))
                        if fn == 'sin' and sub.targets[0].attr == 'z':
                            trig['sin'] = sub.lineno
                        if fn == 'cos' and sub.targets[0].attr == 'w':
                            trig['cos'] = sub.lineno
                if trig['sin'] and trig['cos']:
                    offenders.append('%s:%d hand-rolled yaw_to_quat_zw'
                                     % (_rel(path), trig['sin']))
    assert not offenders, (
        'Hand-rolled quaternion math — use scout.core.geometry '
        '(or docker/scout-skills/geometry.py in the skills container), '
        'ADR-0013:\n' + '\n'.join(offenders))


# --- SC7: every core module is adopted and tested -----------------------------
# core/battery.py sat orphaned for a month with 5 passing tests over dead code
# while battery_monitor kept a verbatim copy. This makes an unused core module
# a test failure.

def test_sc7_core_modules_adopted_and_tested():
    core_modules = [p.stem for p in _py_files(CORE, exclude=('__init__.py',))]
    # Launch files count as adopters: core.sites is consumed at launch time
    # (slam/behaviors resolve the active site before any node exists).
    node_sources = [p.read_text() for p in
                    _py_files(PKG) + _py_files(REPO / 'scout' / 'launch')]
    unadopted = [m for m in core_modules
                 if not any('scout.core.%s' % m in src
                            or 'scout.core import' in src and (' %s' % m) in src
                            for src in node_sources)]
    assert not unadopted, (
        'core modules imported by no node (orphaned logic — adopt or delete, '
        'ADR-0013): %s' % unadopted)
    test_dir = REPO / 'scout' / 'test'
    untested = [m for m in core_modules
                if not (test_dir / ('test_%s.py' % m)).exists()]
    assert not untested, (
        'core modules without a 1:1 test file (ADR-0012 pairs each core module '
        'with test_<module>.py): %s' % untested)


# --- SC1: node main()s delegate to run_node -----------------------------------
# Seven hand-rolled mains existed in three shapes, four with a real bug (catch
# KeyboardInterrupt only + unconditional rclpy.shutdown() -> RCLError on
# external shutdown). run_node (node_util) is the one blessed entry point
# (ADR-0012). ALLOW empties as Batch 3 migrates them — do not add to it.

SC1_ALLOW = {}


def _console_script_modules():
    """Node module paths, parsed from setup.py's console_scripts (authoritative)."""
    tree = _tree(REPO / 'scout' / 'setup.py')
    mods = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Constant) and isinstance(node.value, str)
                and ' = scout.' in node.value and ':main' in node.value):
            mods.append(node.value.split(' = ')[1].split(':')[0].split('.')[1])
    assert mods, 'could not parse console_scripts out of setup.py'
    return [PKG / ('%s.py' % m) for m in mods]


def test_sc1_mains_delegate_to_run_node():
    _check_allow(SC1_ALLOW)
    offenders = []
    for path in _console_script_modules():
        if _rel(path) in SC1_ALLOW:
            continue
        mains = [n for n in _tree(path).body
                 if isinstance(n, ast.FunctionDef) and n.name == 'main']
        assert mains, '%s: console script without a main()' % _rel(path)
        fn = mains[0]
        args_ok = ([a.arg for a in fn.args.args] == ['args']
                   and len(fn.args.defaults) == 1)
        body = [n for n in fn.body
                if not (isinstance(n, ast.Expr)
                        and isinstance(n.value, ast.Constant))]
        calls_run_node = (
            len(body) == 1 and isinstance(body[0], (ast.Expr, ast.Return))
            and isinstance(body[0].value, ast.Call)
            and getattr(body[0].value.func, 'id', '') == 'run_node')
        if not (args_ok and calls_run_node):
            offenders.append(_rel(path))
    assert not offenders, (
        'main() must be `def main(args=None)` delegating to run_node '
        '(scout.node_util, ADR-0012/0013) — hand-rolled shutdown paths have '
        'shipped real bugs:\n' + '\n'.join(offenders))


# --- SC2: sensor subscriptions use sensor QoS ---------------------------------
# gyro_calibrator publishes /imu/data best-effort; a default reliable
# subscription receives NOTHING and the only symptom is a one-line
# `incompatible QoS` discovery warning (CLAUDE.md). This keeps the trap dead.

SC2_SENSOR_TYPES = {'Imu', 'LaserScan', 'PointCloud2', 'Image',
                    'CompressedImage', 'CameraInfo', 'Range', 'NavSatFix'}


def test_sc2_sensor_subscriptions_use_sensor_qos():
    offenders = []
    for path in _py_files(PKG):
        tree = _tree(path)
        sensor_names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module == 'sensor_msgs.msg':
                sensor_names |= {a.asname or a.name for a in node.names
                                 if a.name in SC2_SENSOR_TYPES}
        if not sensor_names:
            continue
        for node in ast.walk(tree):
            if not (isinstance(node, ast.Call)
                    and getattr(node.func, 'attr', '') == 'create_subscription'
                    and len(node.args) >= 4
                    and getattr(node.args[0], 'id', '') in sensor_names):
                continue
            qos = node.args[3]
            if isinstance(qos, ast.Constant):
                offenders.append('%s:%d' % (_rel(path), node.lineno))
    assert not offenders, (
        'Sensor-topic subscription with a plain depth instead of '
        'qos_profile_sensor_data (or a scout.qos profile) — a best-effort '
        'publisher will silently never deliver (ADR-0013):\n'
        + '\n'.join(offenders))


# --- SC3: TF lookups go through node_util --------------------------------------
# lookup_pose2/lookup_matrix wrap the TF exception triple once; raw
# lookup_transform outside node_util re-forks that handling.

def test_sc3_no_raw_lookup_transform():
    offenders = []
    for path in _py_files(PKG, exclude=('node_util.py',)):
        for node in ast.walk(_tree(path)):
            if (isinstance(node, ast.Call)
                    and getattr(node.func, 'attr', '') == 'lookup_transform'):
                offenders.append('%s:%d' % (_rel(path), node.lineno))
    assert not offenders, (
        'Raw tf_buffer.lookup_transform — use lookup_pose2/lookup_matrix from '
        'scout.node_util (ADR-0013):\n' + '\n'.join(offenders))


# --- SC4: Twist publishers only via CmdVelSource / estop -----------------------
# ADR-0001 ended the many-writers era: CmdVelSource owns the motion contract
# (rate, caps, zero burst); estop deliberately bypasses it to brake through
# the twist_mux lock. A third Twist publisher is a policy violation, not code.

SC4_ALLOW = {
    'scout/scout/cmd_vel_source.py': 'the motion contract implementation',
    'scout/scout/estop.py': 'priority-255 brake through the mux lock (ADR-0001)',
    'scout/scout/traction_monitor.py':
        'mid-chain per-side derate scaler between the final mux and the '
        'driver — not a motion source; preserves incoming cadence and zero '
        'Twists (docs/traction_control_spec.md option (a))',
}


def test_sc4_twist_publishers_are_allowlisted():
    _check_allow(SC4_ALLOW)
    offenders = []
    for path in _py_files(PKG):
        if _rel(path) in SC4_ALLOW:
            continue
        for node in ast.walk(_tree(path)):
            if (isinstance(node, ast.Call)
                    and getattr(node.func, 'attr', '') == 'create_publisher'
                    and node.args
                    and getattr(node.args[0], 'id', '') in ('Twist',
                                                            'TwistStamped')):
                offenders.append('%s:%d' % (_rel(path), node.lineno))
    assert not offenders, (
        'New cmd_vel publisher — motion goes through CmdVelSource '
        '(scout.cmd_vel_source), which owns the caps, publish rate and '
        'zero-burst stop (ADR-0001/0013):\n' + '\n'.join(offenders))


# --- SC9 (structural half): status payloads come from core.status ---------------
# test_status.py freezes the wire strings; this bans the escape hatch that let
# /flipper/status and /traction/status ship as inline json.dumps dicts with no
# owner and no freeze (the ADR-0012 drift, reintroduced in JSON). A node never
# serializes a wire payload itself — it calls a scout.core.status formatter.

SC9_ALLOW = {
    'scout/scout/tag_relocalizer.py':
        'HTTP POST body to fleet_status (active_map persist, ADR-0029) — '
        'an HTTP API payload, not a ROS wire format',
}


def test_sc9_no_inline_json_wire_payloads():
    _check_allow(SC9_ALLOW)
    offenders = []
    for path in _py_files(PKG):
        if _rel(path) in SC9_ALLOW:
            continue
        for node in ast.walk(_tree(path)):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr == 'dumps'
                    and getattr(node.func.value, 'id', '') == 'json'):
                offenders.append('%s:%d' % (_rel(path), node.lineno))
    assert not offenders, (
        'Inline json.dumps in a node — wire formats live in scout.core.status '
        '(format_* + a test_status.py freeze), never ad hoc in the publisher '
        '(ADR-0012/0013 SC9):\n' + '\n'.join(offenders))


# --- SC6: one owner for the bind-mount config path ------------------------------
# The bind-mount-wins policy was implemented four times in four shapes; it now
# lives in scout.robot_profile (resolve_config / resolve_config_dir) only.

# --- SC11: no sync service/action calls ----------------------------------------
# Humble's Sync-Vs-Async how-to: a blocking Client.call() (or
# ActionClient.send_goal()) from inside any subscription/timer/service callback
# deadlocks the single-threaded executor with NO warning, NO exception, NO
# stack-trace evidence. Every node spins single-threaded via run_node, so the
# sync forms are banned: call_async / send_goal_async + done-callback
# (link_watchdog's CancelGoal pattern is the house reference).
#
# cancel_all_goals_async is banned too, for a different reason: it belongs to
# a per-action ActionClient, so a node holding its own client cancels ONE
# action and silently misses the other (tilt_monitor covered navigate_to_pose
# but not navigate_through_poses — on the safety path). node_util's
# cancel_nav_goals covers every NAV_ACTIONS entry.

SC11_BANNED = {'call': 'call_async', 'send_goal': 'send_goal_async',
               'cancel_all_goals_async': 'node_util.cancel_nav_goals'}
SC11_ALLOW = {}


def test_sc11_no_sync_service_or_action_calls():
    _check_allow(SC11_ALLOW)
    offenders = []
    for path in _py_files(PKG):
        if _rel(path) in SC11_ALLOW:
            continue
        for node in ast.walk(_tree(path)):
            if (isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Attribute)
                    and node.func.attr in SC11_BANNED):
                offenders.append('%s:%d .%s( -> .%s(' % (
                    _rel(path), node.lineno, node.func.attr,
                    SC11_BANNED[node.func.attr]))
    assert not offenders, (
        'Synchronous rclpy client call — deadlocks silently inside any '
        'callback on the single-threaded executor (Humble Sync-Vs-Async '
        'how-to). Use the async form + done-callback (see link_watchdog), '
        'ADR-0013 SC11:\n' + '\n'.join(offenders))


def test_sc11_site_paths_not_flat_pools():
    """Per-location state lives under sites/active (ADR-0023). A node default
    pointing at the retired flat maps/ or captures/ pools silently splits the
    stores across layouts — the exact corruption sites exist to prevent."""
    offenders = []
    launch_dir = REPO / 'scout' / 'launch'
    for path in _py_files(PKG) + _py_files(launch_dir) + _py_files(SKILLS):
        for lineno, line in enumerate(path.read_text().splitlines(), 1):
            if '/ros_ws/src/maps' in line or '/ros_ws/src/captures' in line:
                offenders.append('%s:%d' % (_rel(path), lineno))
    assert not offenders, (
        'Flat maps//captures/ pool path — use /ros_ws/src/sites/active/... '
        '(ADR-0023):\n' + '\n'.join(offenders))


def test_sc6_bind_path_only_in_robot_profile():
    offenders = []
    launch_dir = REPO / 'scout' / 'launch'
    for path in _py_files(PKG, exclude=('robot_profile.py',)) + _py_files(launch_dir):
        for lineno, line in enumerate(path.read_text().splitlines(), 1):
            if '/ros_ws/src/scout' in line:
                offenders.append('%s:%d' % (_rel(path), lineno))
    assert not offenders, (
        'Hardcoded bind-mount config path — use resolve_config/'
        'resolve_config_dir from scout.robot_profile (ADR-0013):\n'
        + '\n'.join(offenders))
