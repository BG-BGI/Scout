# Pi host platform baseline + shared-DDS re-test procedure

Asserted mechanically by `scripts/preflight.sh` (read-only, safe mid-mission).
Network/transport decisions: ADR-0020 (shared DDS domain), ADR-0021 (no
companion bridge). Companion stack: `companion/`.

## Host baseline

| Item | Value |
|---|---|
| Board | Raspberry Pi 5, 16 GB |
| OS | Raspberry Pi OS 64-bit (aarch64) |
| Hostname | `scout` (mDNS → `http://scout.local`) |
| Boot config | `dtparam=uart0=on` (GPIO14/15 → `/dev/ttyAMA0`), `dtparam=spi=on` (SPI0 → APA102), `usb_max_current_enable=1` |
| Devices | `/dev/ttyAMA0` (RoboClaw), `/dev/ttyUSB0` (RPLIDAR A2, CP2102), D455 USB 3.2 (`8086:0b5c`), `/dev/input/js0` (optional), `/dev/spidev0.0` (pins must read ALT0 via `pinctrl get 10,11` — `/dev/spidev10.0` is a trap) |
| Clock | No RTC — NTP sync is a preflight gate; sensor stamps are wrong until synced |
| Power | DeWALT 20V MAX, no BMS; RoboClaw 16.0 V Min Main is the only pack protection |
| Docker | Engine + Compose v2, `docker.service` enabled |
| **Network** | **Corp WiFi (wlan0) is the PRIMARY link** — degradation is an expected operating condition, not an edge case. Size every staleness/timeout number against the measured dropout profile below |

## DDS identity + transport (ADR-0022)

| Item | Value |
|---|---|
| `ROS_DOMAIN_ID` | **17** on every service, both machines (load-bearing locally, cosmetic across the bridge) |
| Discovery | **Simple discovery on loopback, `ROS_LOCALHOST_ONLY=1`, both machines.** The Fast DDS Discovery Server + `super_client.xml` are retired (ADR-0022: the zenoh bridge is CycloneDDS-based and can't join a server graph). Throwaway-container discovery false negatives are back — judge liveness from logs/data topics |
| Cross-machine | `eclipse/zenoh-bridge-ros2dds:1.10.0`, one per machine: Pi listens `tcp/:7447`, companion (10.1.57.18) dials out. Allowlists in `scout/config/zenoh_bridge.json5` / `companion/config/zenoh_bridge.json5`, kept mirrored; Pi accepts nothing inbound. **Verified working end-to-end 2026-08-20** (`/scan` at full rate on the companion) |
| ⚠ | The `ROS_LOCALHOST_ONLY` **env var overrides** the bridge's config key — compose sets it on both bridge services; don't trust the json5 alone |
| ⚠ | Three bridge env vars are all load-bearing (ADR-0022 bring-up findings): `ROS_LOCALHOST_ONLY=1`, `ROS_DISTRO=humble` (assumes iron otherwise), and the `CYCLONEDDS_URI` peer-ping config — without the last, Fast DDS and the Cyclone bridge are **mutually deaf on loopback** (session up, zero routes, no error anywhere) |
| Debugging | Bridge health reads in this order: session line (`New ROS 2 bridge detected`) → `Route Publisher/Subscriber created` lines → `ros2 topic echo <topic> <type> --once` on the companion. `ros2 topic list` there is a false signal (subscriptions list topics with zero data flowing) |

## Re-test outcome (2026-08-20)

Plain cross-VLAN DDS is dead on this network: with the Pi's discovery server
LAN-bound on `:11812` and the Debian box (10.1.57.18, different VLAN) dialing
it, `tcpdump -ni wlan0 udp port 11812` on the Pi saw **zero packets** — corp
inter-VLAN UDP filtering. The wlan0-lockup question was never reached and is
moot while DDS stays on loopback. Step 1 below is retained as the recipe if
same-subnet placement ever revives the plain-DDS option.

## Link baselines (fill in as measured)

Instrument: `scripts/wifi_link_baseline.py <peer-ip> --rate 10 --minutes 30`
(add `--load /scan` for the loaded profile). CSVs land in `captures/net/`.

brasfield@scout:~/Desktop/Scout $ sudo python3 scripts/wifi_link_baseline.py 10.1.80.1 --minutes 30 [sudo] password for brasfield: pinging 10.1.80.1 at 10.0 Hz for 30.0 min (dgram socket) -> captures/net/link_10-1-80-1_20260820-085124.csv dropout: 4.4s (5 misses) samples 18000 miss 5 (0.03%) RTT ms p50 4.9 p95 36.0 p99 93.0 max 394.2 dropouts (>=3 misses): 1 (2.0/hr) 4.4s (5 misses) csv: captures/net/link_10-1-80-1_20260820-085124.csv

  GNU nano 8.4             captures/net/link_10-1-80-1_20260820-085124.csv                      
t_unix,seq,rtt_ms
1787233884.300,0,2.25
1787233884.400,1,1.24
1787233884.502,2,4.03
1787233884.600,3,1.39
1787233884.704,4,6.01
1787233884.801,5,2.71
1787233884.900,6,1.22
1787233885.004,7,6.09
1787233885.102,8,3.22
1787233885.202,9,4.03
1787233885.300,10,1.42
1787233885.400,11,1.42
1787233885.500,12,1.57
1787233885.600,13,1.67
1787233885.700,14,2.20
1787233885.803,15,4.97
1787233885.905,16,6.35
1787233886.002,17,3.94
1787233886.101,18,2.61
1787233886.202,19,4.12
1787233886.304,20,5.69
1787233886.402,21,4.11
1787233886.502,22,4.11
1787233886.600,23,2.12
1787233886.704,24,5.50
1787233886.803,25,4.73
1787233886.903,26,4.92
1787233887.003,27,4.23
1787233887.109,28,10.89
1787233887.200,29,1.70
1787233887.302,30,4.18
1787233887.403,31,4.41
1787233887.502,32,4.13
1787233887.603,33,4.41
1787233887.702,34,4.09
1787233887.805,35,7.16
1787233887.904,36,5.39
1787233888.007,37,8.42
1787233888.103,38,4.82
1787233888.202,39,4.01
1787233888.303,40,4.22
1787233888.402,41,4.09
1787233888.502,42,4.13
1787233888.603,43,4.43
            [ File 'captures/net/link_10-1-80-1_20260820-085124.csv' is unwritable ]
^G Help         ^O Write Out    ^F Where Is     ^K Cut          ^T Execute      ^C Location
^X Exit         ^R Read File    ^\ Replace      ^U Paste        ^J Justify      ^/ Go To Line

## wlan0/DDS lockup re-test (gate for ADR-0020 — spec §0.1)

History: after stack start, wlan0 once blackholed for ~10 min (association
held, no disconnect logged). Blamed on DDS multicast; **never confirmed** —
possibly ethernet or an unrelated regression. `ROS_LOCALHOST_ONLY=1` was the
containment. The re-test decides whether ADR-0020 stands.

**A lockup presents as a multi-minute dropout in `wifi_link_baseline.py`
while `nmcli` still shows the association up.**

### Step 1 — isolated participant (stack untouched)

1. Pi stack running exactly as today (localhost-only still in effect for it).
2. On the Pi, start a second, throwaway discovery server LAN-bound on `:11812`:
   `docker compose run --rm -e ROS_LOCALHOST_ONLY=0 -e ROS_DOMAIN_ID=17 robot fastdds discovery -i 1 -l 0.0.0.0 -p 11812`
3. In a second throwaway container (`-e ROS_LOCALHOST_ONLY=0 -e ROS_DOMAIN_ID=17 -e ROS_DISCOVERY_SERVER=127.0.0.1:11812`),
   run a `/scan`-sized synthetic talker (`ros2 topic pub -r 12 /retest_scan sensor_msgs/msg/LaserScan "{...1590 ranges}"` or `demo_nodes_cpp talker` at minimum).
4. On the companion host (`ROS_DOMAIN_ID=17`, `ROS_DISCOVERY_SERVER=<pi-ip>:11812`),
   run a listener; confirm messages flow.
5. Run `wifi_link_baseline.py <gateway-ip>` on the Pi for ≥30 min alongside.
6. **Pass:** zero lockups; dropout profile ≈ idle baseline. **Fail:** stop,
   adopt the zenoh fallback (ADR-0021), revert the ADR-0020 rebind.

### Step 2 — full stack (after the Phase 2 compose change, pre-merge)

1. Deploy the ADR-0020 compose change on the branch (`ROS_LOCALHOST_ONLY`
   removed, domain 17, discovery `-l 0.0.0.0`).
2. Companion host up with rtabmap subscribed (compressed color + compressedDepth
   + `/scan` — the real streaming load).
3. ≥30 min under load; robot static, teleop only with the operator at the
   controls. `wifi_link_baseline.py` running throughout on both ends.
4. Same pass criterion. Record both rows in the baseline table above.
