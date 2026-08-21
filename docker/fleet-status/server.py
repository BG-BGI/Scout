"""Fleet status backend for the webui System panel (http://<pi>:9003/api/...;
9002 belongs to observability_mcp).

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

Also drives the host's WiFi via `nmcli` (talks to NetworkManager over the
D-Bus system bus, mounted in at /run/dbus/system_bus_socket). nmcli's
default (non `--show-secrets`) output never includes PSKs, and every wifi_*
function here deliberately avoids `--show-secrets` so credentials never
transit this API even though it has no auth of its own.
"""

import json
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import docker
from docker.errors import NotFound

WIFI_IFACE = os.environ.get('WIFI_IFACE', 'wlan0')
# Companion machine to health-check (ADR-0021). Unset = feature hidden: the
# Pi must work identically with no companion (spec §0.7).
COMPANION_HOST = os.environ.get('COMPANION_HOST', '')

PROJECT = os.environ.get('COMPOSE_PROJECT_NAME', 'scout')
SELF_SERVICE = 'fleet_status'

# Host reboot is OFF by default and enabled only where it's wanted — the
# companion box sets FLEET_ALLOW_HOST_REBOOT=1 in its compose. The Pi's
# fleet_status leaves it unset, so /api/reboot-host 403s there and the webui
# hides the button: no accidental "reboot the robot's brain".
ALLOW_HOST_REBOOT = os.environ.get('FLEET_ALLOW_HOST_REBOOT') == '1'
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


def reboot_host():
    """Reboot the HOST via logind over the bind-mounted system D-Bus socket —
    the same socket nmcli already uses. No host PID namespace needed (nsenter
    would only see the container's PID 1). Companion-only (ALLOW_HOST_REBOOT)."""
    try:
        subprocess.run(
            ['dbus-send', '--system', '--print-reply',
             '--dest=org.freedesktop.login1', '/org/freedesktop/login1',
             'org.freedesktop.login1.Manager.Reboot', 'boolean:true'],
            check=True, capture_output=True, timeout=10,
        )
        return True, 'reboot requested'
    except Exception as e:  # noqa: BLE001 — surface any failure to the caller
        return False, str(e)


def _nmcli(*args, timeout=15):
    """Run nmcli, return (ok, stdout/stderr). Never pass --show-secrets."""
    try:
        result = subprocess.run(
            ['nmcli', *args], capture_output=True, text=True, timeout=timeout,
        )
        if result.returncode != 0:
            return False, result.stderr.strip() or f'nmcli exited {result.returncode}'
        return True, result.stdout
    except FileNotFoundError:
        return False, 'nmcli not installed in this container'
    except subprocess.TimeoutExpired:
        return False, 'nmcli timed out'


def wifi_status():
    ok, out = _nmcli('-t', '-f', 'GENERAL.CONNECTION,IP4.ADDRESS', 'device', 'show', WIFI_IFACE)
    if not ok:
        return {'connected': False, 'error': out}
    fields = dict(line.split(':', 1) for line in out.splitlines() if ':' in line)
    connection = fields.get('GENERAL.CONNECTION', '')
    ip4 = fields.get('IP4.ADDRESS[1]', '')

    signal = None
    ok2, out2 = _nmcli('-t', '-f', 'ACTIVE,SSID,SIGNAL', 'device', 'wifi', 'list', 'ifname', WIFI_IFACE)
    if ok2:
        for line in out2.splitlines():
            parts = line.split(':')
            if len(parts) >= 3 and parts[0] == 'yes':
                signal = int(parts[-1]) if parts[-1].isdigit() else None
                break

    return {
        'connected': bool(connection) and connection != '--',
        'ssid': connection if connection != '--' else None,
        'ip4': ip4 or None,
        'signal': signal,
    }


# --- Companion health + WiFi quality history (ADR-0020/0021) ---------------
# Background samplers, LAN-only observers: display/telemetry, never gating —
# per-topic staleness gating is a Phase 3 concern and lives on the Pi's ROS
# side, not here.

_quality_lock = threading.Lock()
# ring of (t_unix, signal_or_None); 360 samples at 10 s = 1 h of history
_quality_ring = []
_quality_dropouts = 0
_companion = {'configured': bool(COMPANION_HOST), 'host': COMPANION_HOST or None,
              'reachable': None, 'rtt_ms': None, 'last_seen': None}


def _ping_once(host, timeout_s=1):
    """(reachable, rtt_ms) via one system ping; no raw sockets needed."""
    try:
        r = subprocess.run(['ping', '-c', '1', '-W', str(timeout_s), host],
                           capture_output=True, text=True, timeout=timeout_s + 2)
        if r.returncode != 0:
            return False, None
        for tok in r.stdout.split():
            if tok.startswith('time='):
                return True, float(tok[5:])
        return True, None
    except (subprocess.TimeoutExpired, FileNotFoundError, ValueError):
        return False, None


def _companion_sampler():
    while True:
        ok, rtt = _ping_once(COMPANION_HOST)
        _companion['reachable'] = ok
        _companion['rtt_ms'] = rtt
        if ok:
            _companion['last_seen'] = time.time()
        time.sleep(5)


def _wifi_quality_sampler():
    global _quality_dropouts
    was_connected = True
    while True:
        st = wifi_status()
        connected = bool(st.get('connected'))
        with _quality_lock:
            _quality_ring.append((round(time.time(), 1), st.get('signal')))
            del _quality_ring[:-360]
            if was_connected and not connected:
                _quality_dropouts += 1
        was_connected = connected
        time.sleep(10)


def companion_health():
    if not COMPANION_HOST:
        return {'configured': False}
    return dict(_companion)


def wifi_quality():
    with _quality_lock:
        samples = list(_quality_ring)
        dropouts = _quality_dropouts
    signals = [s for _, s in samples if s is not None]
    return {
        'iface': WIFI_IFACE,
        'samples': samples,
        'signal_now': signals[-1] if signals else None,
        'signal_min_1h': min(signals) if signals else None,
        'dropouts_since_start': dropouts,
    }


def wifi_connections():
    """Known (saved) wifi profiles. nmcli's default output never includes PSKs."""
    ok, out = _nmcli('-t', '-f', 'NAME,TYPE,AUTOCONNECT,ACTIVE', 'connection', 'show')
    if not ok:
        return {'error': out}
    out_list = []
    for line in out.splitlines():
        parts = line.split(':')
        if len(parts) != 4:
            continue
        name, conn_type, autoconnect, active = parts
        if conn_type != '802-11-wireless':
            continue
        out_list.append({
            'name': name,
            'autoconnect': autoconnect == 'yes',
            'active': active == 'yes',
        })
    return out_list


def wifi_scan():
    """Nearby SSIDs (rescan). Slower than the other endpoints — call on demand."""
    ok, out = _nmcli('-t', '-f', 'SSID,SIGNAL,SECURITY', 'device', 'wifi', 'list',
                      'ifname', WIFI_IFACE, '--rescan', 'yes', timeout=20)
    if not ok:
        return {'error': out}
    seen = set()
    out_list = []
    for line in out.splitlines():
        parts = line.split(':')
        if len(parts) != 3:
            continue
        ssid, signal, security = parts
        if not ssid or ssid in seen:
            continue
        seen.add(ssid)
        out_list.append({
            'ssid': ssid,
            'signal': int(signal) if signal.isdigit() else None,
            'security': security or None,
        })
    out_list.sort(key=lambda r: r['signal'] or 0, reverse=True)
    return out_list


def wifi_connect(name=None, ssid=None, password=None):
    """Bring up a known profile by name, or a new SSID with a password."""
    if name:
        ok, out = _nmcli('connection', 'up', name, timeout=30)
        return ok, out
    if ssid:
        args = ['device', 'wifi', 'connect', ssid, 'ifname', WIFI_IFACE]
        if password:
            args += ['password', password]
        ok, out = _nmcli(*args, timeout=30)
        return ok, out
    return False, 'must supply name or ssid'


def wifi_forget(name):
    ok, out = _nmcli('connection', 'delete', name)
    return ok, out


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
        elif self.path == '/api/wifi/status':
            self._send_json(200, wifi_status())
        elif self.path == '/api/wifi/connections':
            self._send_json(200, wifi_connections())
        elif self.path == '/api/wifi/scan':
            self._send_json(200, wifi_scan())
        elif self.path == '/api/wifi/quality':
            self._send_json(200, wifi_quality())
        elif self.path == '/api/companion':
            self._send_json(200, companion_health())
        else:
            self._send_json(404, {'error': 'not found'})

    def _read_json_body(self):
        length = int(self.headers.get('Content-Length', 0))
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length))
        except json.JSONDecodeError:
            return {}

    def do_POST(self):
        parts = self.path.strip('/').split('/')

        if parts[:2] == ['api', 'wifi'] and len(parts) == 3 and parts[2] in ('connect', 'forget'):
            body = self._read_json_body()
            if parts[2] == 'connect':
                ok, out = wifi_connect(
                    name=body.get('name'), ssid=body.get('ssid'), password=body.get('password'),
                )
            else:
                name = body.get('name')
                if not name:
                    self._send_json(400, {'error': 'missing name'})
                    return
                conns = wifi_connections()
                is_active = any(c['name'] == name and c['active'] for c in conns
                                 if isinstance(conns, list))
                if is_active and not body.get('force'):
                    self._send_json(400, {
                        'error': 'refusing to forget the active connection without force:true '
                                 '(would strand this session)',
                    })
                    return
                ok, out = wifi_forget(name)
            self._send_json(200 if ok else 400, {'ok': ok, 'detail': out})
            return

        if parts[:2] == ['api', 'restart-all']:
            threading.Thread(target=restart_all_bg, daemon=True).start()
            self._send_json(202, {'ok': True, 'note': 'restarting, staggered'})
            return

        if parts[:2] == ['api', 'reboot-host']:
            if not ALLOW_HOST_REBOOT:
                self._send_json(403, {'error': 'host reboot not enabled here '
                                      '(FLEET_ALLOW_HOST_REBOOT unset)'})
                return
            ok, detail = reboot_host()
            self._send_json(202 if ok else 500, {'ok': ok, 'detail': detail})
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
    threading.Thread(target=_wifi_quality_sampler, daemon=True).start()
    if COMPANION_HOST:
        threading.Thread(target=_companion_sampler, daemon=True).start()
    ThreadingHTTPServer(('0.0.0.0', 9003), Handler).serve_forever()
