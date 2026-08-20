#!/usr/bin/env python3
"""wifi_link_baseline.py — continuous-streaming WiFi link measurement (keeper).

Measures the link profile the offboard architecture actually depends on
(docs/offboard-architecture.md §6.5): steady per-packet RTT and dropout
events, not bulk-transfer throughput. Run on either end (Pi host or
companion); no ROS required unless --load is used.

  ./scripts/wifi_link_baseline.py <peer-ip> [--rate 10] [--minutes 30]
      [--out captures/net] [--load /scan]

Output: per-sample CSV (t, seq, rtt_ms or blank on miss) under --out, plus a
summary line: RTT p50/p95/p99, miss %, dropout events (>=3 consecutive
misses) with durations, dropouts/hour. With --load, also subscribes to the
named topic via rclpy (sensor QoS) and reports its achieved Hz — so the
baseline covers the loaded profile, not idle.

Pass criterion for the ADR-0020 re-test: no wlan0 lockup (a lockup shows as a
multi-minute dropout while the association holds) and a dropout profile
matching the idle baseline.
"""
import argparse
import csv
import os
import select
import socket
import struct
import sys
import threading
import time

ICMP_ECHO = 8


def _checksum(data: bytes) -> int:
    if len(data) % 2:
        data += b"\x00"
    s = sum(struct.unpack(f"!{len(data)//2}H", data))
    s = (s >> 16) + (s & 0xFFFF)
    s += s >> 16
    return ~s & 0xFFFF


def ping_once(sock: socket.socket, addr: str, ident: int, seq: int, timeout: float):
    payload = struct.pack("!d", time.monotonic()) + b"scoutlink"
    header = struct.pack("!BBHHH", ICMP_ECHO, 0, 0, ident, seq & 0xFFFF)
    pkt = struct.pack("!BBHHH", ICMP_ECHO, 0, _checksum(header + payload), ident, seq & 0xFFFF) + payload
    t0 = time.monotonic()
    sock.sendto(pkt, (addr, 0))
    deadline = t0 + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        r, _, _ = select.select([sock], [], [], remaining)
        if not r:
            return None
        data, _ = sock.recvfrom(2048)
        # SOCK_DGRAM ICMP: kernel strips the IP header; SOCK_RAW keeps it.
        icmp = data[20:] if len(data) > 20 and (data[0] >> 4) == 4 else data
        if len(icmp) < 8:
            continue
        rtype, _, _, rident, rseq = struct.unpack("!BBHHH", icmp[:8])
        # DGRAM sockets rewrite ident to the local port; match on seq alone there.
        if rtype == 0 and rseq == seq & 0xFFFF:
            return (time.monotonic() - t0) * 1000.0


def make_socket():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_ICMP)
        return s, "dgram"
    except PermissionError:
        pass
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_RAW, socket.IPPROTO_ICMP)
        return s, "raw"
    except PermissionError:
        sys.exit(
            "need ICMP socket: run as root, or "
            "`sudo sysctl net.ipv4.ping_group_range='0 2147483647'`"
        )


def percentile(sorted_vals, p):
    if not sorted_vals:
        return float("nan")
    k = (len(sorted_vals) - 1) * p / 100.0
    lo, hi = int(k), min(int(k) + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (k - lo)


class TopicLoad(threading.Thread):
    """Optional --load: subscribe a topic with sensor QoS, count messages."""

    def __init__(self, topic):
        super().__init__(daemon=True)
        self.topic = topic
        self.count = 0
        self.t0 = None
        self._stop = threading.Event()

    def run(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from ros2topic.api import get_msg_class

        rclpy.init()
        node = Node("wifi_link_baseline_load")
        msg_cls = get_msg_class(node, self.topic, include_hidden_topics=True, blocking=True)

        def cb(_msg):
            if self.t0 is None:
                self.t0 = time.monotonic()
            self.count += 1

        node.create_subscription(msg_cls, self.topic, cb, qos_profile_sensor_data)
        while not self._stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        rclpy.shutdown()

    def stop(self):
        self._stop.set()

    def hz(self):
        if self.t0 is None or self.count < 2:
            return 0.0
        return self.count / max(time.monotonic() - self.t0, 1e-6)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("peer", help="IP to ping (the other end of the Pi<->companion link)")
    ap.add_argument("--rate", type=float, default=10.0, help="pings per second (default 10)")
    ap.add_argument("--minutes", type=float, default=30.0, help="duration (default 30)")
    ap.add_argument("--timeout", type=float, default=1.0, help="per-ping timeout s (counts as miss)")
    ap.add_argument("--out", default="captures/net", help="CSV output dir")
    ap.add_argument("--load", metavar="TOPIC", help="also subscribe TOPIC via rclpy and report its Hz")
    args = ap.parse_args()

    sock, kind = make_socket()
    sock.setblocking(False)
    ident = os.getpid() & 0xFFFF

    os.makedirs(args.out, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    csv_path = os.path.join(args.out, f"link_{args.peer.replace('.', '-')}_{stamp}.csv")

    load = None
    if args.load:
        load = TopicLoad(args.load)
        load.start()

    period = 1.0 / args.rate
    total = int(args.minutes * 60 * args.rate)
    rtts, misses = [], 0
    dropouts = []  # (start_t, duration_s, n_missed)
    run_miss_start, run_miss_n = None, 0
    print(f"pinging {args.peer} at {args.rate} Hz for {args.minutes} min ({kind} socket) -> {csv_path}")

    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_unix", "seq", "rtt_ms"])
        t_next = time.monotonic()
        for seq in range(total):
            rtt = ping_once(sock, args.peer, ident, seq, args.timeout)
            now = time.time()
            if rtt is None:
                misses += 1
                w.writerow([f"{now:.3f}", seq, ""])
                if run_miss_start is None:
                    run_miss_start = time.monotonic()
                run_miss_n += 1
            else:
                rtts.append(rtt)
                w.writerow([f"{now:.3f}", seq, f"{rtt:.2f}"])
                if run_miss_n >= 3:  # dropout = >=3 consecutive misses
                    dur = time.monotonic() - run_miss_start
                    dropouts.append((run_miss_start, dur, run_miss_n))
                    print(f"  dropout: {dur:.1f}s ({run_miss_n} misses)")
                run_miss_start, run_miss_n = None, 0
            if seq % int(max(args.rate, 1) * 60) == 0 and seq:
                f.flush()
            t_next += period
            time.sleep(max(0.0, t_next - time.monotonic()))
        if run_miss_n >= 3:
            dropouts.append((run_miss_start, time.monotonic() - run_miss_start, run_miss_n))

    if load:
        load.stop()
    rtts.sort()
    hours = args.minutes / 60.0
    print(f"\nsamples {total}  miss {misses} ({100.0*misses/max(total,1):.2f}%)")
    print(f"RTT ms  p50 {percentile(rtts,50):.1f}  p95 {percentile(rtts,95):.1f}  p99 {percentile(rtts,99):.1f}  max {rtts[-1] if rtts else float('nan'):.1f}")
    print(f"dropouts (>=3 misses): {len(dropouts)}  ({len(dropouts)/hours:.1f}/hr)")
    for _, dur, n in dropouts:
        print(f"  {dur:.1f}s ({n} misses)")
    if load:
        print(f"load topic {args.load}: {load.hz():.1f} Hz over the run")
    print(f"csv: {csv_path}")


if __name__ == "__main__":
    main()
