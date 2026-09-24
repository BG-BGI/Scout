#!/usr/bin/env python3
"""perf_capture.py — host-side CPU/starvation baseline capture (keeper).

The Phase-1 instrument for the CPU-starvation investigation (handoff
2026-09-24; docs/perf-experiments.md): synchronized host + per-container
cgroup evidence, captured OUTSIDE docker with stdlib only, deliberately NOT
the observability exporter (which is itself under investigation and off by
default). Every experiment run in docs/perf-experiments.md wraps itself in
this script.

  sudo ./scripts/perf_capture.py --minutes 15 [--interval 2] [--ros-probe]

Writes captures/perf/<UTC>/ (gitignored, mirrors captures/bags/<UTC>/):
  meta.json       once — git SHA, SCOUT_TAG/SCOUT_PROFILE from .env, compose
                  services up, Pi model/RAM/kernel, PSI availability, and a
                  (wall, monotonic) clock anchor for correlating files.
  cgroups.json    once — per-container cpu.max quota/period, cpu.weight,
                  memory.max, and the name<->id<->cgroup-path map.
  host.csv        per tick — loadavg, busy/iowait %, ctxt/s (the cs-storm
                  signal, ADR-0034), procs running/blocked, PSI cpu/mem/io,
                  swap + major-fault rates, MemAvailable, SoC temp, min/max
                  core freq, vcgencmd throttled hex, and self_cpu_ms (this
                  script's own cost — the dataset carries its tax receipt).
  containers.csv  per tick — per compose container cpu.stat deltas
                  (usage/nr_periods/nr_throttled/THROTTLED_USEC — the direct
                  quota-denial evidence), cpu.pressure, memory.current,
                  memory.events oom/max deltas.
  threads.csv     per tick — top-N processes by CPU delta plus pinned comms
                  (slam/ekf/rplidar/realsense/component containers); per
                  thread: cpu time delta and SCHEDULING DELAY delta from
                  /proc/<pid>/task/<tid>/schedstat (runnable-but-waiting is
                  the starvation number a CPU%% can't show).
  ros.jsonl       1 Hz from the optional ROS probe (see --ros-probe).

--ros-probe launches scripts/perf_ros_probe.py as a THROWAWAY compose-run
container off the robot image (its cost lands in its own cgroup row, capped
to 0.3 cpus via docker update — never inside robot/nav2/slam quotas), sharing
the host monotonic clock so ros.jsonl rows line up with host.csv directly.
Subscribe-only; it never publishes. CLAUDE.md bans diag containers during
nav because unbounded ones starve the Pi — this one is the sanctioned,
single, capped exception and it measures its own cost in containers.csv.

Pre-flight (once per Pi): PSI needs `psi=1` on the kernel cmdline if
/proc/pressure is absent; cgroup driver assumed systemd (falls back to
cgroupfs paths); vcgencmd present on Pi OS. Overhead is bounded by
--interval clamping, the top-N process cap, os.nice(10), and is measured
(self_cpu_ms).
"""
import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
CG_ROOT = Path("/sys/fs/cgroup")
COMPOSE_PROJECT = "scout"
# Always-tracked comm prefixes (hot ROS processes), on top of the top-N sweep.
PINNED_COMMS = ("component_contai", "async_slam", "ekf_node", "rplidar", "realsense")
PROBE_NAME = "scout-perfprobe"
PROBE_CPUS = "0.3"


def read_text(path) -> str:
    try:
        return Path(path).read_text()
    except OSError:
        return ""


def run(cmd, timeout=15) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True,
                              timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError):
        return ""


# --- parsers (pure: text in, dict out) ---------------------------------------

def parse_proc_stat(text) -> dict:
    out = {}
    for line in text.splitlines():
        f = line.split()
        if not f:
            continue
        if f[0] == "cpu":  # aggregate jiffies
            v = list(map(int, f[1:]))
            out["total"] = sum(v)
            out["idle"] = v[3] + v[4]
            out["iowait"] = v[4]
        elif f[0] == "ctxt":
            out["ctxt"] = int(f[1])
        elif f[0] == "procs_running":
            out["procs_running"] = int(f[1])
        elif f[0] == "procs_blocked":
            out["procs_blocked"] = int(f[1])
    return out


def parse_pressure(text) -> dict:
    # "some avg10=1.23 avg60=... total=456" (+ optional "full ..." line)
    out = {}
    for line in text.splitlines():
        f = line.split()
        if not f:
            continue
        kv = dict(p.split("=") for p in f[1:])
        out[f[0]] = {"avg10": float(kv.get("avg10", 0.0)),
                     "total": int(kv.get("total", 0))}
    return out


def parse_kv_lines(text, keys) -> dict:
    out = {}
    for line in text.splitlines():
        f = line.split()
        if len(f) >= 2 and f[0].rstrip(":") in keys:
            out[f[0].rstrip(":")] = int(f[1])
    return out


def parse_cpu_stat(text) -> dict:
    return parse_kv_lines(text, {"usage_usec", "nr_periods", "nr_throttled",
                                 "throttled_usec"})


# --- docker / cgroup discovery ------------------------------------------------

def docker_ps() -> list[dict]:
    """[{id, name, service}] for this compose project's running containers."""
    out = run(["docker", "ps", "--no-trunc",
               "--filter", f"label=com.docker.compose.project={COMPOSE_PROJECT}",
               "--format",
               '{{.ID}}\t{{.Names}}\t{{.Label "com.docker.compose.service"}}'])
    rows = []
    for line in out.splitlines():
        parts = line.split("\t")
        if len(parts) == 3:
            rows.append({"id": parts[0], "name": parts[1], "service": parts[2]})
    return rows


def cgroup_dir(cid) -> Path | None:
    for p in (CG_ROOT / "system.slice" / f"docker-{cid}.scope",  # systemd driver
              CG_ROOT / "docker" / cid):                          # cgroupfs driver
        if p.is_dir():
            return p
    return None


def cgroup_config(cdir) -> dict:
    return {
        "cpu.max": read_text(cdir / "cpu.max").strip(),
        "cpu.weight": read_text(cdir / "cpu.weight").strip(),
        "memory.max": read_text(cdir / "memory.max").strip(),
    }


# --- thread sweep ---------------------------------------------------------------

def proc_cpu_jiffies(pid) -> tuple[str, int] | None:
    """(comm, utime+stime) from /proc/<pid>/stat; None if gone."""
    text = read_text(f"/proc/{pid}/stat")
    if not text:
        return None
    # comm is parenthesized and may contain spaces — split around it.
    try:
        lpar, rpar = text.index("("), text.rindex(")")
        comm = text[lpar + 1:rpar]
        f = text[rpar + 2:].split()
        return comm, int(f[11]) + int(f[12])  # utime, stime
    except (ValueError, IndexError):
        return None


def thread_sched(pid, tid) -> tuple[int, int] | None:
    """(run_ns, wait_ns) from schedstat — wait is time RUNNABLE but not on a
    CPU, i.e. scheduling delay."""
    f = read_text(f"/proc/{pid}/task/{tid}/schedstat").split()
    if len(f) < 2:
        return None
    return int(f[0]), int(f[1])


def sweep_pids() -> dict[int, tuple[str, int]]:
    out = {}
    for entry in os.listdir("/proc"):
        if entry.isdigit():
            got = proc_cpu_jiffies(int(entry))
            if got:
                out[int(entry)] = got
    return out


# --- meta ---------------------------------------------------------------------

def parse_env_file(path) -> dict:
    out = {}
    for line in read_text(path).splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            out[k] = v
    return out


def write_meta(run_dir, interval_s, top_n, ros_probe):
    env = parse_env_file(REPO / ".env")
    meta = {
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "clock_anchor": {"wall": time.time(), "mono": time.monotonic()},
        "git_sha": run(["git", "-C", str(REPO), "rev-parse", "HEAD"]).strip(),
        "scout_tag": env.get("SCOUT_TAG"),
        "scout_profile": env.get("SCOUT_PROFILE"),
        "companion_host": env.get("COMPANION_HOST"),
        "compose_ps": run(["docker", "compose", "ps", "--format", "json"],
                          timeout=30).strip().splitlines(),
        "uptime_s": float(read_text("/proc/uptime").split()[0] or 0)
        if read_text("/proc/uptime") else None,
        "model": read_text("/proc/device-tree/model").strip("\x00").strip() or None,
        "kernel": run(["uname", "-a"]).strip(),
        "mem_total_kb": parse_kv_lines(read_text("/proc/meminfo"),
                                       {"MemTotal"}).get("MemTotal"),
        "psi_available": os.path.isdir("/proc/pressure"),
        "vcgencmd": bool(shutil.which("vcgencmd")),
        "capture": {"interval_s": interval_s, "top_n": top_n,
                    "ros_probe": ros_probe},
    }
    (run_dir / "meta.json").write_text(json.dumps(meta, indent=2))
    if not meta["psi_available"]:
        print("WARN: /proc/pressure absent — add psi=1 to the kernel cmdline "
              "and reboot for PSI columns", file=sys.stderr)


def write_cgroups(run_dir, containers):
    entries = []
    for c in containers:
        cdir = cgroup_dir(c["id"])
        entries.append({**c, "cgroup": str(cdir) if cdir else None,
                        **(cgroup_config(cdir) if cdir else {})})
    (run_dir / "cgroups.json").write_text(json.dumps(entries, indent=2))


# --- ROS probe lifecycle --------------------------------------------------------

def start_ros_probe(run_rel):
    """Throwaway compose-run container off the robot image, subscribe-only,
    capped after start (compose run can't take --cpus directly)."""
    subprocess.run(["docker", "rm", "-f", PROBE_NAME], capture_output=True)
    r = subprocess.run(
        ["docker", "compose", "run", "--rm", "-d", "--name", PROBE_NAME,
         "robot", "python3", "/ros_ws/src/scripts/perf_ros_probe.py",
         "--out", f"/ros_ws/src/{run_rel}"],
        cwd=REPO, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"WARN: ros probe failed to start: {r.stderr.strip()}",
              file=sys.stderr)
        return False
    subprocess.run(["docker", "update", f"--cpus={PROBE_CPUS}", PROBE_NAME],
                   capture_output=True)
    return True


def stop_ros_probe():
    subprocess.run(["docker", "stop", "-t", "2", PROBE_NAME], capture_output=True)


# --- main loop ------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--minutes", type=float, required=True)
    ap.add_argument("--interval", type=float, default=2.0)
    ap.add_argument("--top-n", type=int, default=8)
    ap.add_argument("--out", default="captures/perf")
    ap.add_argument("--ros-probe", action=argparse.BooleanOptionalAction,
                    default=True)
    args = ap.parse_args()
    interval = max(1.0, min(args.interval, 10.0))

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_rel = f"{args.out}/{stamp}"
    run_dir = REPO / run_rel
    run_dir.mkdir(parents=True, exist_ok=True)
    try:
        os.nice(10)  # never compete with the stack we're measuring
    except OSError:
        pass

    containers = docker_ps()
    write_meta(run_dir, interval, args.top_n, args.ros_probe)
    write_cgroups(run_dir, containers)
    known_names = {c["name"] for c in containers}

    probe_up = args.ros_probe and start_ros_probe(run_rel)

    host_f = open(run_dir / "host.csv", "w", newline="")
    host_w = csv.writer(host_f)
    host_w.writerow([
        "ts_wall", "ts_mono", "load1", "load5", "load15", "cpu_busy_pct",
        "cpu_iowait_pct", "ctxt_per_s", "procs_running", "procs_blocked",
        "psi_cpu_some_avg10", "psi_cpu_full_avg10", "psi_mem_some_avg10",
        "psi_io_some_avg10", "pswpin_s", "pswpout_s", "pgmajfault_s",
        "mem_avail_mb", "swap_free_mb", "temp_c", "freq_khz_min",
        "freq_khz_max", "throttled_hex", "self_cpu_ms"])
    cont_f = open(run_dir / "containers.csv", "w", newline="")
    cont_w = csv.writer(cont_f)
    cont_w.writerow([
        "ts_mono", "service", "name", "usage_usec_d", "nr_periods_d",
        "nr_throttled_d", "throttled_usec_d", "psi_cpu_some_avg10",
        "psi_cpu_full_avg10", "mem_current_bytes", "oom_d", "oom_kill_d"])
    thr_f = open(run_dir / "threads.csv", "w", newline="")
    thr_w = csv.writer(thr_f)
    thr_w.writerow(["ts_mono", "pid", "tid", "comm", "cpu_ms_d",
                    "run_delay_ms_d"])

    hz = os.sysconf("SC_CLK_TCK")
    prev_stat = parse_proc_stat(read_text("/proc/stat"))
    prev_vm = parse_kv_lines(read_text("/proc/vmstat"),
                             {"pswpin", "pswpout", "pgmajfault"})
    prev_cg: dict[str, dict] = {}
    prev_mem_ev: dict[str, dict] = {}
    prev_pids = sweep_pids()
    prev_threads: dict[tuple[int, int], tuple[int, int]] = {}
    prev_self = proc_cpu_jiffies(os.getpid())[1]
    prev_t = time.monotonic()

    deadline = time.monotonic() + args.minutes * 60.0
    ticks = 0
    try:
        while time.monotonic() < deadline:
            time.sleep(interval)
            now_mono = time.monotonic()
            now_wall = time.time()
            dt = now_mono - prev_t
            prev_t = now_mono

            # --- host row
            stat = parse_proc_stat(read_text("/proc/stat"))
            d_total = stat["total"] - prev_stat["total"]
            d_idle = stat["idle"] - prev_stat["idle"]
            d_iow = stat["iowait"] - prev_stat["iowait"]
            busy = (1 - d_idle / d_total) * 100 if d_total else 0.0
            iow = d_iow / d_total * 100 if d_total else 0.0
            ctxt_s = (stat["ctxt"] - prev_stat["ctxt"]) / dt
            prev_stat = stat
            vm = parse_kv_lines(read_text("/proc/vmstat"),
                                {"pswpin", "pswpout", "pgmajfault"})
            vm_d = {k: (vm.get(k, 0) - prev_vm.get(k, 0)) / dt for k in vm}
            prev_vm = vm
            mem = parse_kv_lines(read_text("/proc/meminfo"),
                                 {"MemAvailable", "SwapFree"})
            psi_cpu = parse_pressure(read_text("/proc/pressure/cpu"))
            psi_mem = parse_pressure(read_text("/proc/pressure/memory"))
            psi_io = parse_pressure(read_text("/proc/pressure/io"))
            load = read_text("/proc/loadavg").split()[:3] or ["", "", ""]
            temps = [int(t) / 1000 for t in
                     (read_text(p).strip() for p in
                      sorted(Path("/sys/class/thermal").glob("thermal_zone*/temp"))
                      ) if t.isdigit()] if Path("/sys/class/thermal").is_dir() else []
            freqs = [int(t) for t in
                     (read_text(p).strip() for p in
                      sorted(Path("/sys/devices/system/cpu").glob(
                          "cpu[0-9]*/cpufreq/scaling_cur_freq"))) if t.isdigit()]
            throttled = ""
            if shutil.which("vcgencmd"):
                out = run(["vcgencmd", "get_throttled"], timeout=5)
                throttled = out.strip().split("=")[-1] if "=" in out else ""
            self_j = proc_cpu_jiffies(os.getpid())[1]
            self_ms = (self_j - prev_self) * 1000 / hz
            prev_self = self_j
            host_w.writerow([
                f"{now_wall:.3f}", f"{now_mono:.3f}", *load, f"{busy:.1f}",
                f"{iow:.1f}", f"{ctxt_s:.0f}", stat.get("procs_running", ""),
                stat.get("procs_blocked", ""),
                psi_cpu.get("some", {}).get("avg10", ""),
                psi_cpu.get("full", {}).get("avg10", ""),
                psi_mem.get("some", {}).get("avg10", ""),
                psi_io.get("some", {}).get("avg10", ""),
                f"{vm_d.get('pswpin', 0):.1f}", f"{vm_d.get('pswpout', 0):.1f}",
                f"{vm_d.get('pgmajfault', 0):.1f}",
                mem.get("MemAvailable", 0) // 1024,
                mem.get("SwapFree", 0) // 1024,
                f"{max(temps):.1f}" if temps else "",
                min(freqs) if freqs else "", max(freqs) if freqs else "",
                throttled, f"{self_ms:.1f}"])

            # --- container rows (re-discover when the set changes)
            live = docker_ps()
            if {c["name"] for c in live} != known_names:
                known_names = {c["name"] for c in live}
                containers = live
                write_cgroups(run_dir, containers)
            for c in containers:
                cdir = cgroup_dir(c["id"])
                if cdir is None:
                    continue
                cs = parse_cpu_stat(read_text(cdir / "cpu.stat"))
                pc = parse_pressure(read_text(cdir / "cpu.pressure"))
                ev = parse_kv_lines(read_text(cdir / "memory.events"),
                                    {"oom", "oom_kill"})
                p = prev_cg.get(c["id"], cs)
                pe = prev_mem_ev.get(c["id"], ev)
                cont_w.writerow([
                    f"{now_mono:.3f}", c["service"], c["name"],
                    cs.get("usage_usec", 0) - p.get("usage_usec", 0),
                    cs.get("nr_periods", 0) - p.get("nr_periods", 0),
                    cs.get("nr_throttled", 0) - p.get("nr_throttled", 0),
                    cs.get("throttled_usec", 0) - p.get("throttled_usec", 0),
                    pc.get("some", {}).get("avg10", ""),
                    pc.get("full", {}).get("avg10", ""),
                    read_text(cdir / "memory.current").strip(),
                    ev.get("oom", 0) - pe.get("oom", 0),
                    ev.get("oom_kill", 0) - pe.get("oom_kill", 0)])
                prev_cg[c["id"]] = cs
                prev_mem_ev[c["id"]] = ev

            # --- thread rows: top-N by CPU delta + pinned comms
            pids = sweep_pids()
            deltas = {pid: (v[0], v[1] - prev_pids.get(pid, (v[0], v[1]))[1])
                      for pid, v in pids.items()}
            hot = sorted(deltas, key=lambda p: deltas[p][1], reverse=True)
            selected = set(hot[:args.top_n])
            selected |= {pid for pid, (comm, _) in deltas.items()
                         if comm.startswith(PINNED_COMMS)}
            prev_pids = pids
            seen = set()
            for pid in selected:
                task_dir = Path(f"/proc/{pid}/task")
                try:
                    tids = [int(t) for t in os.listdir(task_dir)]
                except OSError:
                    continue
                for tid in tids:
                    sched = thread_sched(pid, tid)
                    if sched is None:
                        continue
                    seen.add((pid, tid))
                    p_run, p_wait = prev_threads.get((pid, tid), sched)
                    comm = read_text(f"/proc/{pid}/task/{tid}/comm").strip()
                    thr_w.writerow([
                        f"{now_mono:.3f}", pid, tid, comm,
                        f"{(sched[0] - p_run) / 1e6:.1f}",
                        f"{(sched[1] - p_wait) / 1e6:.1f}"])
                    prev_threads[(pid, tid)] = sched
            prev_threads = {k: v for k, v in prev_threads.items() if k in seen}

            ticks += 1
            if ticks % 15 == 0:
                for f in (host_f, cont_f, thr_f):
                    f.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if probe_up:
            stop_ros_probe()
        for f in (host_f, cont_f, thr_f):
            f.close()
    print(f"{ticks} ticks -> {run_dir}")


if __name__ == "__main__":
    main()
