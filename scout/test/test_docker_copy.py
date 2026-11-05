"""Every sibling module docker/fleet-status/server.py imports must be COPY'd into the image.

2026-11-05: server.py gained `import mapsafe` but the Dockerfile's explicit COPY
list did not, so the container crash-looped on start (ModuleNotFoundError) and
the deploy health check failed. Catch it here instead.
"""

import pathlib
import re

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
IMAGE = REPO / 'docker' / 'fleet-status'


def test_fleet_status_dockerfile_copies_every_local_import():
    src = (IMAGE / 'server.py').read_text()
    local = {p.stem for p in IMAGE.glob('*.py')} - {'server'}
    imported = set(re.findall(r'^(?:import|from) (\w+)', src, re.M)) & local
    copy_lines = [
        line for line in (IMAGE / 'Dockerfile').read_text().splitlines()
        if line.startswith('COPY')
    ]
    copied = set(re.findall(r'(\w+)\.py', ' '.join(copy_lines)))
    assert imported, 'expected server.py to import local modules'
    assert imported <= copied, 'not COPY-d into the image: %s' % sorted(imported - copied)
