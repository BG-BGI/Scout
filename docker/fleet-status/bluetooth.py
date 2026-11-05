"""Bluetooth game-controller recovery for the fleet_status System panel.

Stdlib only. Drives the HOST's bluetoothd through `bluetoothctl` over the
bind-mounted D-Bus system socket (same mechanism as nmcli/dbus-send in
server.py). Never touches ROS and never publishes motion. Every bluetoothctl
call is wrapped in `timeout` because bluetoothd can wedge and hang clients.
Nothing secret is parsed or returned (no link keys, no PINs).
"""
import os
import re
import subprocess
import threading
import time

PAD_RE = re.compile(r"xbox|controller|gamepad", re.I)
JS0_PATH = os.path.join("/hostfs" if os.path.isdir("/hostfs") else "", "dev/input/js0")
MAC_RE = re.compile(r"^([0-9A-F]{2}(:[0-9A-F]{2}){5})$", re.I)

_lock = threading.Lock()
_op = {"name": None, "state": "idle", "msg": "", "started_s": None}


def _run(args, timeout=10):
    """Run a command under `timeout`; return (ok, stdout+stderr)."""
    try:
        r = subprocess.run(["timeout", str(timeout), *args], capture_output=True,
                           text=True, timeout=timeout + 5)
        return r.returncode == 0, (r.stdout + r.stderr).strip()
    except FileNotFoundError:
        return False, f"{args[0]} not installed in this container"
    except subprocess.TimeoutExpired:
        return False, f"{args[0]} timed out"


def _ctl(*args, timeout=10):
    return _run(["bluetoothctl", *args], timeout=timeout)


def parse_devices(text):
    """`bluetoothctl devices [Paired]` lines -> [(mac, name)]."""
    out = []
    for line in text.splitlines():
        m = re.match(r"^Device\s+(\S+)\s+(.*)$", line.strip())
        if m and MAC_RE.match(m.group(1)):
            out.append((m.group(1).upper(), m.group(2).strip()))
    return out


def detect_mac(devices_text=None):
    """BT_PAD_MAC if set, else first paired device named Xbox/Controller/Gamepad."""
    env = os.environ.get("BT_PAD_MAC", "").strip()
    if env:
        return env.upper()
    if devices_text is None:
        ok, devices_text = _ctl("devices", "Paired")
        if not ok:
            # older bluetoothctl has no filter arg
            ok, devices_text = _ctl("paired-devices")
            if not ok:
                return None
    for mac, name in parse_devices(devices_text):
        if PAD_RE.search(name):
            return mac
    return None


def parse_info(text):
    """`bluetoothctl info <MAC>` -> pad dict fields."""
    def yn(key):
        m = re.search(rf"^\s*{key}:\s*(yes|no)\s*$", text, re.M | re.I)
        return bool(m) and m.group(1).lower() == "yes"
    name = re.search(r"^\s*Name:\s*(.+?)\s*$", text, re.M)
    bat = re.search(r"^\s*Battery Percentage:\s*0x[0-9a-fA-F]+\s*\((\d+)\)", text, re.M)
    return {
        "name": name.group(1) if name else None,
        "paired": yn("Paired"), "bonded": yn("Bonded"),
        "trusted": yn("Trusted"), "connected": yn("Connected"),
        "battery": int(bat.group(1)) if bat else None,
    }


def status():
    ok, show = _ctl("show", timeout=5)
    powered = bool(re.search(r"^\s*Powered:\s*yes", show, re.M)) if ok else False
    mac = detect_mac()
    pad = {"mac": mac, "name": None, "paired": False, "bonded": False,
           "trusted": False, "connected": False, "battery": None}
    if mac:
        ok, info = _ctl("info", mac, timeout=5)
        if ok:
            pad.update(parse_info(info))
    return {"adapter_powered": powered, "pad": pad,
            "js0_present": os.path.exists(JS0_PATH), "op": op_snapshot()}


def op_snapshot():
    with _lock:
        d = dict(_op)
    if d["started_s"] is not None:
        d["started_s"] = round(time.time() - d["started_s"], 1)  # seconds ago
    return d


def repair_script(mac):
    """Single bluetoothctl session body (piped on stdin) — the recipe that works.
    One-shot `bluetoothctl pair` without a registered agent FAILS."""
    return (
        "(\n"
        "echo 'agent NoInputNoOutput'; echo 'default-agent'; echo 'pairable on';\n"
        "echo 'scan on'; sleep 10;\n"
        f"echo 'pair {mac}'; sleep 12;\n"
        f"echo 'trust {mac}'; sleep 1;\n"
        f"echo 'connect {mac}'; sleep 5;\n"
        "echo 'scan off'; sleep 1; echo 'quit'\n"
        ") | timeout 60 bluetoothctl"
    )


def _do_reconnect():
    mac = detect_mac()
    if not mac:
        return False, "no paired controller found"
    _ctl("disconnect", mac, timeout=10)
    time.sleep(1)
    ok, out = _ctl("connect", mac, timeout=20)
    if ok and "successful" in out.lower():
        return True, "connected"
    return False, (out.splitlines() or ["connect failed"])[-1]


def _do_restart():
    ok, out = _run(["dbus-send", "--system", "--print-reply",
                    "--dest=org.freedesktop.systemd1", "/org/freedesktop/systemd1",
                    "org.freedesktop.systemd1.Manager.RestartUnit",
                    "string:bluetooth.service", "string:replace"], timeout=15)
    if not ok:
        return False, out.splitlines()[-1] if out else "RestartUnit failed"
    time.sleep(4)
    _ctl("power", "on", timeout=5)
    return True, "bluetooth.service restarted"


def _discover_mac():
    """Scan ~10 s for a device named like a pad (pad must be in pairing mode)."""
    _ctl("--timeout", "10", "scan", "on", timeout=15)
    ok, out = _ctl("devices", timeout=5)
    if ok:
        for mac, name in parse_devices(out):
            if PAD_RE.search(name):
                return mac
    return None


def _do_repair():
    mac = detect_mac() or _discover_mac()
    if not mac:
        return False, "controller not found; hold sync until light blinks fast and retry"
    _run(["bluetoothctl", "remove", mac], timeout=10)
    try:
        r = subprocess.run(["bash", "-c", repair_script(mac)], capture_output=True,
                           text=True, timeout=75)
    except Exception as e:  # noqa: BLE001
        return False, str(e)
    ok, info = _ctl("info", mac, timeout=5)
    p = parse_info(info) if ok else {}
    if p.get("paired") and p.get("connected"):
        return True, "paired and connected"
    if p.get("paired"):
        return True, "paired; not connected yet (press the Xbox button)"
    return False, "pairing failed; hold sync until fast blink and retry"


OPS = {"reconnect": _do_reconnect, "restart": _do_restart, "repair": _do_repair}


def _worker(name):
    try:
        ok, msg = OPS[name]()
    except Exception as e:  # noqa: BLE001
        ok, msg = False, str(e)
    with _lock:
        _op.update(state="ok" if ok else "error", msg=msg)


def start_op(name):
    """Single-flight. Returns (http_code, payload)."""
    if name not in OPS:
        return 404, {"error": "not found"}
    with _lock:
        if _op["state"] == "running":
            return 409, {"error": "operation already running", "op": dict(_op)}
        _op.update(name=name, state="running", msg="", started_s=time.time())
    threading.Thread(target=_worker, args=(name,), daemon=True).start()
    return 202, {"ok": True, "op": op_snapshot()}
