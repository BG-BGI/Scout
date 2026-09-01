"""Per-site elevator.json validation (docker/scout-skills/elevator_config.py,
loaded by path — separate container, shared schema per ADR-0011/ADR-0030)."""

import importlib.util
import sys
from pathlib import Path

import pytest

MOD_PY = Path(__file__).resolve().parents[2] / 'docker' / 'scout-skills' / 'elevator_config.py'

spec = importlib.util.spec_from_file_location('skills_elevator_config', MOD_PY)
ec = importlib.util.module_from_spec(spec)
sys.modules['skills_elevator_config'] = ec
spec.loader.exec_module(ec)


def base_config(**floor_overrides):
    floor = {'label': '0', 'door_waypoint': 'elev_lobby'} | floor_overrides
    return {
        'version': 1,
        'elevators': {'main': {'equipment_number': 'EQ-1-1-1', 'floors': {'2': floor}}},
    }


def test_valid_config_defaults():
    cfg = ec.load_elevator_config(base_config())
    assert cfg['default_elevator'] == 'main'  # single elevator becomes default
    floor = cfg['elevators']['main']['floors'][2]  # key int-coerced
    assert floor['entrance_side'] == 'Front'
    assert floor['board_depth_m'] == pytest.approx(1.4)
    assert floor['exit'] == 'reverse'
    assert floor['exit_move_m'] == pytest.approx(1.7)  # depth + 0.3


def test_exit_move_follows_custom_depth():
    cfg = ec.load_elevator_config(base_config(board_depth_m=2.0))
    assert cfg['elevators']['main']['floors'][2]['exit_move_m'] == pytest.approx(2.3)


def test_version_gate():
    data = base_config()
    with pytest.raises(ValueError, match='version'):
        ec.load_elevator_config({**{'version': 2}, 'elevators': data['elevators']})
    with pytest.raises(ValueError, match='version'):
        ec.load_elevator_config({'elevators': data['elevators']})


def test_bad_entrance_side():
    with pytest.raises(ValueError, match='entrance_side'):
        ec.load_elevator_config(base_config(entrance_side='Back'))


def test_bad_exit_direction():
    with pytest.raises(ValueError, match='exit'):
        ec.load_elevator_config(base_config(exit='backwards'))


def test_floor_key_must_be_int():
    data = base_config()
    data['elevators']['main']['floors'] = {'lobby': {}}
    with pytest.raises(ValueError, match='not an integer'):
        ec.load_elevator_config(data)


def test_floor_number_ge_1():
    data = base_config()
    data['elevators']['main']['floors'] = {'0': {}}
    with pytest.raises(ValueError, match='>= 1'):
        ec.load_elevator_config(data)


def test_equipment_number_required():
    data = base_config()
    del data['elevators']['main']['equipment_number']
    with pytest.raises(ValueError, match='equipment_number'):
        ec.load_elevator_config(data)


def test_identity_shape():
    data = base_config()
    data['elevators']['main']['identity'] = 'robot@example.com'
    with pytest.raises(ValueError, match='identity'):
        ec.load_elevator_config(data)


def test_default_elevator_must_exist():
    data = base_config()
    data['default_elevator'] = 'freight'
    with pytest.raises(ValueError, match='default_elevator'):
        ec.load_elevator_config(data)


def test_resolve_elevator():
    cfg = ec.load_elevator_config(base_config())
    name, entry = ec.resolve_elevator(cfg, None)
    assert name == 'main' and entry['equipment_number'] == 'EQ-1-1-1'
    with pytest.raises(ValueError, match='freight'):
        ec.resolve_elevator(cfg, 'freight')


def test_no_default_among_many():
    data = base_config()
    data['elevators']['freight'] = {'equipment_number': 'EQ-1-1-2', 'floors': {}}
    cfg = ec.load_elevator_config(data)
    assert cfg['default_elevator'] is None
    with pytest.raises(ValueError, match='no default_elevator'):
        ec.resolve_elevator(cfg, None)
