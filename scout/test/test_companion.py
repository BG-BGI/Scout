"""Companion-side convention checks (ADR-0022/0025) — source-level only.

The companion containers cannot import scout.* (separate images, no scout
package), so every cross-surface agreement there is a deliberate copy — and
until now none was frozen: the rfid recorder hand-rebuilt LATCHED_HISTORY_QOS
("must match scout/scout/qos.py", nothing checked), the inspection recorder
mirrors SITE_NAME_RE and hand-parses the /patrol_status grammar. These tests
parse the companion SOURCE (ast/text, never importing rclpy) and pin each
copy to its owner, the same mechanism as SC10's vendored-geometry freeze.
"""

import ast
import pathlib
import re

from scout.core import status
from scout.core.sites import SITE_NAME_RE

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
COMPANION = REPO / 'companion'
QOS_PY = REPO / 'scout' / 'scout' / 'qos.py'
RFID_RECORDER = COMPANION / 'rfid' / 'recorder.py'
INSPECTION_RECORDER = COMPANION / 'inspection' / 'recorder.py'

SENSOR_TYPES = {'Imu', 'LaserScan', 'PointCloud2', 'Image',
                'CompressedImage', 'CameraInfo', 'Range', 'NavSatFix'}


def _qos_profile_kwargs(path, var_name):
    """{kwarg: 'PolicyName' or const} for `var_name = QoSProfile(...)`."""
    for node in ast.parse(path.read_text()).body:
        if not (isinstance(node, ast.Assign)
                and isinstance(node.targets[0], ast.Name)
                and node.targets[0].id == var_name
                and isinstance(node.value, ast.Call)):
            continue
        out = {}
        for kw in node.value.keywords:
            v = kw.value
            out[kw.arg] = (v.value if isinstance(v, ast.Constant)
                           else ast.unparse(v).split('.')[-1])
        return out
    raise AssertionError('%s lost %s' % (path.name, var_name))


def test_rfid_reads_qos_matches_pi_latched_history():
    # A depth or durability change on the Pi silently breaks the companion's
    # outage replay — this is the check the recorder's comment asked for.
    pi = _qos_profile_kwargs(QOS_PY, 'LATCHED_HISTORY_QOS')
    companion = _qos_profile_kwargs(RFID_RECORDER, 'READS_QOS')
    assert companion == pi, (
        'companion READS_QOS drifted from scout.qos.LATCHED_HISTORY_QOS — '
        'sync the copy (it cannot import scout.qos)')


def test_rfid_registry_qos_matches_pi_latched():
    pi = _qos_profile_kwargs(QOS_PY, 'LATCHED_QOS')
    companion = _qos_profile_kwargs(RFID_RECORDER, 'REGISTRY_QOS')
    assert companion == pi, (
        'companion REGISTRY_QOS drifted from scout.qos.LATCHED_QOS — sync '
        'the copy')


def test_nfc_recorder_reuses_shared_script_with_nfc_params():
    # rfid/recorder.py is the SHARED tag recorder (ADR-0026); the nfc_recorder
    # service must run that same script with the three NFC params, so its QoS
    # (asserted above) and schema are literally the same as RFID's. Freeze the
    # wiring so a compose edit cannot silently point NFC at the RFID DB/topics.
    compose = (COMPANION / 'docker-compose.yaml').read_text()
    assert 'nfc_recorder:' in compose, 'companion lost the nfc_recorder service'
    for token in ('/rfid/recorder.py', '__node:=nfc_recorder',
                  'db_path:=/sites/active/nfc.db', 'reads_topic:=/nfc/reads',
                  'registry_topic:=/nfc/registry'):
        assert token in compose, (
            'nfc_recorder service lost %r — its wiring is frozen (ADR-0026)'
            % token)


def test_site_name_regex_copy_is_identical():
    m = re.search(r'SITE_NAME_RE = re\.compile\(r"([^"]+)"\)',
                  INSPECTION_RECORDER.read_text())
    assert m, 'inspection recorder lost its SITE_NAME_RE mirror'
    assert m.group(1) == SITE_NAME_RE.pattern, (
        'companion SITE_NAME_RE drifted from scout.core.sites — sync the copy')


def test_patrol_idle_states_match_grammar():
    # The inspection recorder decides "not driving" from the first |-field of
    # /patrol_status; its idle set must be exactly the tokens the grammar
    # emits for idle and planning.
    m = re.search(r'PATROL_IDLE_STATES = (\{[^}]+\})',
                  INSPECTION_RECORDER.read_text())
    assert m, 'inspection recorder lost PATROL_IDLE_STATES'
    idle_states = ast.literal_eval(m.group(1))
    assert idle_states == {
        status.format_patrol_status('idle', 0).split('|')[0],
        status.format_patrol_plan('').split('|')[0],
    }, ('companion PATROL_IDLE_STATES drifted from the core.status patrol '
        'grammar — sync the copy')


def test_companion_sensor_subscriptions_use_sensor_qos():
    # SC2, extended to the companion tree: a default reliable subscription to
    # a best-effort sensor publisher receives NOTHING (the realsense topics
    # cross the zenoh bridge best-effort).
    offenders = []
    for path in sorted(COMPANION.rglob('*.py')):
        tree = ast.parse(path.read_text())
        sensor_names = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module == 'sensor_msgs.msg':
                sensor_names |= {a.asname or a.name for a in node.names
                                 if a.name in SENSOR_TYPES}
        if not sensor_names:
            continue
        for node in ast.walk(tree):
            if (isinstance(node, ast.Call)
                    and getattr(node.func, 'attr', '') == 'create_subscription'
                    and len(node.args) >= 4
                    and getattr(node.args[0], 'id', '') in sensor_names
                    and isinstance(node.args[3], ast.Constant)):
                offenders.append('%s:%d' % (path.relative_to(REPO), node.lineno))
    assert not offenders, (
        'Companion sensor-topic subscription with a plain depth instead of '
        'qos_profile_sensor_data (SC2, ADR-0013):\n' + '\n'.join(offenders))
