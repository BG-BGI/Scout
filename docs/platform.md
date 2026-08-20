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

## DDS identity (ADR-0020)

| Item | Value |
|---|---|
| `ROS_DOMAIN_ID` | **17** — set explicitly on every Pi service and every companion service; a machine on another value silently sees nothing |
| Discovery | Fast DDS Discovery Server v2, id 0, on the Pi at `:11811`, LAN-bound (`-l 0.0.0.0`). Pi services keep `ROS_DISCOVERY_SERVER=127.0.0.1:11811` (standalone path unchanged); companion sets `ROS_DISCOVERY_SERVER=<pi-ip>:11811` |
| Introspection | Shells still need SUPER_CLIENT (`scout/config/super_client.xml` recipe) or `ros2 topic list` reads near-empty |

## Link baselines (fill in as measured)

Instrument: `scripts/wifi_link_baseline.py <peer-ip> --rate 10 --minutes 30`
(add `--load /scan` for the loaded profile). CSVs land in `captures/net/`.

| Baseline | Date | RTT p50/p95/p99 ms | Miss % | Dropouts/hr (durations) |
|---|---|---|---|---|
| Idle (stack up, no companion subs) | — | — | — | — |
| Loaded (companion rtabmap subscribed) | — | — | — | — |

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
