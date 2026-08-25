"""Lint is part of the suite: pytest green = ruff green (ADR-0013).

Skips when ruff is absent (e.g. `colcon test` in the Pi container) — CI and the
dev Mac install it via requirements-dev.txt, so the gate always runs somewhere
that blocks a merge.
"""

import pathlib
import shutil
import subprocess

import pytest

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent.parent


def test_ruff_clean():
    ruff = shutil.which('ruff')
    if ruff is None:
        pytest.skip('ruff not installed (Pi container) — CI runs it')
    proc = subprocess.run(
        [ruff, 'check', '.'], cwd=REPO_ROOT, capture_output=True, text=True)
    assert proc.returncode == 0, 'ruff violations:\n' + proc.stdout + proc.stderr
