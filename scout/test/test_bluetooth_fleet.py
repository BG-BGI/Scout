"""fleet_status Bluetooth module (docker/fleet-status/bluetooth.py).

Covers parsing, single-flight and the repair recipe."""
import importlib.util
import pathlib
import threading
from unittest import mock

_P = pathlib.Path(__file__).resolve().parents[2] / 'docker/fleet-status/bluetooth.py'
_spec = importlib.util.spec_from_file_location('fleet_bluetooth', _P)
bt = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bt)

MAC = '74:C4:12:D2:8A:AB'
INFO = (
    'Device 74:C4:12:D2:8A:AB (public)\n\tName: Xbox Wireless Controller\n'
    '\tPaired: yes\n\tBonded: yes\n\tTrusted: yes\n\tConnected: yes\n'
    '\tBattery Percentage: 0x64 (100)\n'
)


def test_parse_info():
    p = bt.parse_info(INFO)
    assert p == {'name': 'Xbox Wireless Controller', 'paired': True, 'bonded': True,
                 'trusted': True, 'connected': True, 'battery': 100}


def test_parse_info_unpaired_no_battery():
    p = bt.parse_info('\tName: x\n\tPaired: no\n\tConnected: no\n')
    assert p['paired'] is False and p['battery'] is None


def test_detect_mac_autodetect(monkeypatch):
    monkeypatch.delenv('BT_PAD_MAC', raising=False)
    txt = f'Device AA:BB:CC:DD:EE:FF Keyboard\nDevice {MAC} Xbox Wireless Controller\n'
    assert bt.detect_mac(txt) == MAC
    assert bt.detect_mac('Device AA:BB:CC:DD:EE:FF Keyboard\n') is None


def test_detect_mac_env(monkeypatch):
    monkeypatch.setenv('BT_PAD_MAC', 'aa:bb:cc:dd:ee:01')
    assert bt.detect_mac('') == 'AA:BB:CC:DD:EE:01'


def test_single_flight_409():
    gate = threading.Event()
    def slow():
        gate.wait(5)
        return True, 'done'
    with mock.patch.dict(bt.OPS, {'reconnect': slow}):
        bt._op.update(state='idle')
        code, _ = bt.start_op('reconnect')
        assert code == 202
        code2, payload = bt.start_op('repair')
        assert code2 == 409 and payload['op']['state'] == 'running'
        gate.set()
        for _ in range(100):
            if bt._op['state'] != 'running':
                break
            threading.Event().wait(0.05)
        assert bt._op['state'] == 'ok'


def test_unknown_op_404():
    assert bt.start_op('nope')[0] == 404


def test_repair_script_matches_recipe():
    s = bt.repair_script(MAC)
    order = ['agent NoInputNoOutput', 'default-agent', 'pairable on', 'scan on',
             f'pair {MAC}', f'trust {MAC}', f'connect {MAC}', 'scan off']
    idx = [s.index(x) for x in order]
    assert idx == sorted(idx)
    assert 'sleep 12' in s and s.rstrip().endswith('timeout 60 bluetoothctl')
    assert s.count('bluetoothctl') == 1  # single session
