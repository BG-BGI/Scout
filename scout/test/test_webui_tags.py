"""webui tag selector (U11): rows say which other maps a tag is surveyed on, as
plain text. tagOptionLabel is lifted out of app.js (which needs a DOM) and run
under Node; skips when `node` is absent (Pi container) — CI / the dev Mac run it."""

import json
import pathlib
import shutil
import subprocess

import pytest

APP_JS = pathlib.Path(__file__).resolve().parents[2] / 'webui' / 'app.js'
SRC = APP_JS.read_text()


def _function_source(name):
    start = SRC.index(f'function {name}(')
    return SRC[start:SRC.index('\n}\n', start) + 2]


def _label(tag, active):
    node = shutil.which('node')
    if node is None:
        pytest.skip('node not installed (Pi container) — CI / dev Mac run it')
    script = (_function_source('tagOptionLabel')
              + '\nconst a = JSON.parse(process.argv[1]);'
                'console.log(JSON.stringify(tagOptionLabel(a.tag, a.active)));')
    proc = subprocess.run([node, '-e', script, json.dumps({'tag': tag, 'active': active})],
                          capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    return json.loads(proc.stdout)


def _pose(x=1.0):
    return {'x': x, 'y': 0.0, 'yaw': 0.0, 'surveyed_at': None}


def test_tag_only_on_the_active_map_keeps_its_plain_name():
    assert _label({'name': 'doghouse', 'map_x': 1.0,
                   'poses': {'floor1': _pose()}}, 'floor1') == 'doghouse'


def test_tag_with_no_poses_field_is_just_the_name():
    assert _label({'name': 'doghouse', 'map_x': None}, 'floor1') == 'doghouse'


def test_tag_also_surveyed_elsewhere_lists_the_other_maps():
    label = _label({'name': 'doghouse', 'map_x': 1.0,
                    'poses': {'floor1': _pose(), 'floor1_plan': _pose(),
                              'floor2': _pose()}}, 'floor1')
    assert label == 'doghouse - also surveyed on: floor1_plan, floor2'


def test_tag_unsurveyed_here_says_so():
    label = _label({'name': 'doghouse', 'map_x': None,
                    'poses': {'floor1': _pose()}}, 'floor1_plan')
    assert label == 'doghouse - not surveyed here; surveyed on: floor1'


def test_db_strings_are_rendered_as_text_never_html():
    label = _label({'name': '<img src=x onerror=1>', 'map_x': 1.0,
                    'poses': {'<b>m</b>': _pose()}}, 'floor1')
    assert label == '<img src=x onerror=1> - also surveyed on: <b>m</b>'
    body = SRC[SRC.index('async function tagSetStart('):]
    body = body[:body.index('\n}\n')]
    assert 'o.textContent = tagOptionLabel(' in body
    assert 'innerHTML =' not in body.replace("tagNameSel.innerHTML = '';", '')
