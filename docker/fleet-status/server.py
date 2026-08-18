"""Fleet status backend for the webui System panel (http://<pi>:9002/api/...).

Standalone like docker/ros-mcp and docker/scout-skills: no ROS, no DDS. Talks
to the Docker Engine over /var/run/docker.sock (mounted read-write so it can
start/stop/restart containers) and reads host vitals straight out of /proc
and /sys, which are not PID/mount-namespaced so they reflect the real Pi
even though this runs in its own container.

⚠ No auth — LAN-trust only, same caveat as ros_mcp/scout_skills. Unlike
those two, this ALSO holds the Docker socket, so the blast radius if this
endpoint were ever exposed off the LAN is much larger (full container
lifecycle control). Scope is deliberately narrowed here: every action is
restricted to containers carrying this compose project's label, and this
service refuses to touch its own container (a self-restart would kill the
request that triggered it before the response could go out).
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import docker
from docker.errors import NotFound

PROJECT = os.environ.get('COMPOSE_PROJECT_NAME', 'scout')
SELF_SERVICE = 'fleet_status'
RESTART_ALL_STAGGER_S = 2.0
# /hostfs is the host's root, bind-mounted read-only (compose) so disk usage
# reflects the Pi's real SD card, not this container's own overlay fs.
HOST_ROOT = '/hostfs' if os.path.isdir('/hostfs') else '/'

client = docker.from_env()


def _containers():
    return client.containers.list(
        all=True, filters={'label': f'com.docker.compose.project={PROJECT}'},
    )


def _service_name(container):
    return container.labels.get('com.docker.compose.service', container.name)


def _container_stats(container):
    """CPU%/mem for a running container; zeros for a stopped one (stats()
    blocks forever on a dead container otherwise)."""
    if container.status != 'running':
        return 0.0, 0, 0
    try:
        stats = container.stats(stream=False)
        cpu_delta = (stats['cpu_stats']['cpu_usage']['total_usage']
                     - stats['precpu_stats']['cpu_usage']['total_usage'])
        sys_delta = (stats['cpu_stats']['system_cpu_usage']
                     - stats['precpu_stats']['system_cpu_usage'])
        n_cpus = stats['cpu_stats'].get('online_cpus') or len(
            stats['cpu_stats']['cpu_usage'].get('percpu_usage') or [1])
        cpu_pct = (cpu_delta / sys_delta) * n_cpus * 100 if sys_delta > 0 else 0.0
        mem_bytes = stats['memory_stats'].get('usage', 0)
        mem_limit = stats['memory_stats'].get('limit', 0)
        return round(cpu_pct, 1), mem_bytes // (1024 * 1024), mem_limit // (1024 * 1024)
    except (KeyError, ZeroDivisionError):
        return 0.0, 0, 0


def list_containers():
    out = []
    for c in _containers():
        cpu_pct, mem_mb, mem_limit_mb = _container_stats(c)
        out.append({
            'name': c.name,
            'service': _service_name(c),
            'status': c.status,
            'cpu_percent': cpu_pct,
            'mem_mb': mem_mb,
            'mem_limit_mb': mem_limit_mb,
            'self': _service_name(c) == SELF_SERVICE,
        })
    out.sort(key=lambda r: r['service'])
    return out


def _read(path):
    with open(path) as f:
        return f.read()


def host_stats():
    # CPU%: two /proc/stat samples 0.2 s apart (host-wide, not namespaced).
    def cpu_snapshot():
        line = _read('/proc/stat').splitlines()[0].split()[1:]
        vals = list(map(int, line))
        idle = vals[3] + vals[4]
        total = sum(vals)
        return idle, total

    idle0, total0 = cpu_snapshot()
    time.sleep(0.2)
    idle1, total1 = cpu_snapshot()
    d_idle, d_total = idle1 - idle0, total1 - total0
    cpu_percent = round((1 - d_idle / d_total) * 100, 1) if d_total > 0 else 0.0

    meminfo = {}
    for line in _read('/proc/meminfo').splitlines():
        k, v = line.split(':')
        meminfo[k.strip()] = int(v.strip().split()[0])  # kB
    mem_total_mb = meminfo.get('MemTotal', 0) // 1024
    mem_avail_mb = meminfo.get('MemAvailable', 0) // 1024
    mem_used_mb = mem_total_mb - mem_avail_mb

    load1, load5, load15 = map(float, _read('/proc/loadavg').split()[:3])
    uptime_s = float(_read('/proc/uptime').split()[0])

    temp_c = None
    try:
        temp_c = round(int(_read('/sys/class/thermal/thermal_zone0/temp')) / 1000, 1)
    except OSError:
        pass

    disk_total_gb = disk_used_gb = None
    try:
        st = os.statvfs(HOST_ROOT)
        total_b = st.f_blocks * st.f_frsize
        free_b = st.f_bavail * st.f_frsize
        disk_total_gb = round(total_b / 1e9, 1)
        disk_used_gb = round((total_b - free_b) / 1e9, 1)
    except OSError:
        pass

    return {
        'cpu_percent': cpu_percent,
        'load_avg': [load1, load5, load15],
        'mem_used_mb': mem_used_mb,
        'mem_total_mb': mem_total_mb,
        'temp_c': temp_c,
        'disk_used_gb': disk_used_gb,
        'disk_total_gb': disk_total_gb,
        'uptime_s': uptime_s,
    }


def _find(service_or_name):
    for c in _containers():
        if c.name == service_or_name or _service_name(c) == service_or_name:
            return c
    return None


def restart_all_bg():
    for c in _containers():
        if _service_name(c) == SELF_SERVICE:
            continue
        try:
            c.restart(timeout=10)
        except NotFound:
            pass
        time.sleep(RESTART_ALL_STAGGER_S)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # docker stats polling every 30s isn't worth the access log

    def _send_json(self, code, payload):
        body = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        if self.path == '/api/stats':
            self._send_json(200, host_stats())
        elif self.path == '/api/containers':
            self._send_json(200, list_containers())
        else:
            self._send_json(404, {'error': 'not found'})

    def do_POST(self):
        parts = self.path.strip('/').split('/')

        if parts[:2] == ['api', 'restart-all']:
            threading.Thread(target=restart_all_bg, daemon=True).start()
            self._send_json(202, {'ok': True, 'note': 'restarting, staggered'})
            return

        if len(parts) == 4 and parts[0] == 'api' and parts[1] == 'containers':
            name, action = parts[2], parts[3]
            if action not in ('restart', 'stop', 'start'):
                self._send_json(400, {'error': f'unknown action {action}'})
                return
            container = _find(name)
            if container is None:
                self._send_json(404, {'error': f'no such service {name}'})
                return
            if _service_name(container) == SELF_SERVICE and action != 'start':
                self._send_json(400, {'error': 'refusing to stop/restart my own container'})
                return
            try:
                (container.start() if action == 'start' else getattr(container, action)(timeout=10))
                self._send_json(200, {'ok': True})
            except NotFound:
                self._send_json(404, {'error': 'container disappeared mid-request'})
            return

        self._send_json(404, {'error': 'not found'})


if __name__ == '__main__':
    ThreadingHTTPServer(('0.0.0.0', 9002), Handler).serve_forever()
