#!/usr/bin/env python3
"""Boot relocalization + floor transit from registered AprilTags.

The site policy loads a saved map in localization mode, which has to guess a
start pose (map origin unless told otherwise) — and a robot powered on
anywhere else is silently mislocalized, planning through walls. This node
closes that gap: on the first sighting of a *registered* tag (scout-skills'
registry at tags.db, which stores each tag's surveyed pose PER MAP), it
solves the robot's map pose from the live TF of the detection and publishes
/initialpose, which amcl (localization mode's map->odom owner, ADR-0028)
consumes natively to re-centre its particle cloud. The filter polishes from
there.

Floor transit (ADR-0029): a site holds multiple maps and a tag is surveyed per
map (a floor can hold a scan map and a drawing map). A tag with a pose on the
active map relocalizes there and is never a transit candidate. Seeing a tag
with NO pose on the active map but poses elsewhere means the robot may have
changed floors — core.tag_homes picks the home: the one other map with the
SAME source as the active map, else no transit (zero or several candidates
log a throttled warning naming them) — the
lidar can't tell, the tag can. After `min_transit_sightings` consistent
sightings (and only in localization mode, with no nav goal in flight, outside
the cooldown), this node live-swaps the grid via /map_server/load_map, seeds
/initialpose with the tag-solved pose (already in the new map's frame — the
tag was surveyed there), and best-effort POSTs the new active_map to
fleet_status (the only site.json writer) so the switch survives restarts.
LoadMap republishes the latched /map before responding; amcl and the global
costmap's static layer both re-consume it, and a short delay orders amcl's
map callback ahead of the /initialpose.
A transit between maps of different source (scan <-> drawing) is never done
live — amcl's parameters are fixed at launch — it logs one throttled line and
leaves the restart to the operator.

After the first seed the node keeps correcting (core.landmark_correction):
a median window of close, stationary tag solves that disagrees with the
amcl TF pose by more than a threshold re-seeds /initialpose, rate-limited,
and logs the delta (the drift measurement) + /tag_relocalizer/correction.
/tag_relocalizer/reseed
(std_srvs/Trigger) re-arms it (and the transit machinery) for the next
sighting. A transit resets the flag itself — arriving on a new floor is a new
boot as far as seeding is concerned.

Portable-base contract: the registry pose is only as fresh as the last
detect_apriltags/apriltag_watch sighting made while WELL-LOCALIZED. Move the base while
the robot is off and the seed will be confidently wrong — re-survey the tag
(drive to it and run detect_apriltags) after any move, before the next cold boot.

Geometry (SE2 — tags are assumed roughly vertical, like the base's face):
the registry stores map_x/map_y (tag centre) and map_yaw = the standoff
heading, i.e. pointing AT the face, so the face normal in map is
map_yaw + pi. The live detection gives the tag in base_link; the face normal
there is the tag frame's z-axis floor projection, sign-disambiguated toward
the robot (the robot is by definition on the visible side — same convention
as scout-skills' map_geometry). Then
    theta = (map_yaw + pi) - atan2(n_y, n_x)
    p_robot = p_tag_map - R(theta) @ p_tag_base
"""

import json
import math
import os
import sqlite3
import threading
import time
import urllib.request

from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from scout.core import sites, tag_homes
from scout.core.geometry import yaw_to_quat_zw
from scout.core.landmark_correction import LandmarkCorrector
from scout.core.status import NAV_BUSY_STATES, parse_nav_state
from scout.node_util import lookup_matrix, run_node
from scout.qos import LATCHED_QOS

# A per-frame read must not stall the detection callback on a locked tags.db.
REGISTRY_BUSY_S = 0.5

# Same bind mount the maps come through; scout-skills writes it as /maps.
DEFAULT_TAGS_DB = '/ros_ws/src/sites/active/maps/tags.db'
DEFAULT_SITE_JSON = '/ros_ws/src/sites/active/site.json'
DEFAULT_MAPS_DIR = '/ros_ws/src/sites/active/maps'
DEFAULT_FLEET_API = 'http://127.0.0.1:9003'

# Below this floor-projection of the tag z-axis the face is lying flat and
# has no usable heading (mirrors scout-skills map_geometry).
MIN_NORMAL_PROJ = 0.2

# Delay between LoadMap success and the /initialpose, so amcl's map callback
# (from the republished latched /map) lands first.
SEED_DELAY_S = 0.7


def _norm_family(fam):
    """'tagStandard52h13' / 'Standard52h13' / '36h11' -> comparable form."""
    f = fam.lower()
    return f[3:] if f.startswith('tag') else f


class TagRelocalizer(Node):

    def __init__(self):
        super().__init__('tag_relocalizer')
        self.declare_parameter('tags_db', DEFAULT_TAGS_DB)
        self.declare_parameter('site_json', DEFAULT_SITE_JSON)
        self.declare_parameter('maps_dir', DEFAULT_MAPS_DIR)
        # Beyond this the single-view tag pose (especially yaw) is too noisy
        # to seed from; wait for a closer look.
        self.declare_parameter('max_tag_dist_m', 3.0)
        self.declare_parameter('cov_xy', 0.05 ** 2)
        self.declare_parameter('cov_yaw', math.radians(5.0) ** 2)
        # Floor transit: consecutive frames agreeing on the same foreign map
        # before switching, and the refractory period after any attempt.
        self.declare_parameter('min_transit_sightings', 3)
        self.declare_parameter('transit_cooldown_s', 30.0)
        self.declare_parameter('fleet_api', DEFAULT_FLEET_API)
        # Continuous correction after the first seed (median window).
        self.declare_parameter('correct_max_dist_m', 1.5)
        self.declare_parameter('correct_window', 5)
        self.declare_parameter('correct_spread_xy_m', 0.03)
        self.declare_parameter('correct_spread_yaw_deg', 2.0)
        self.declare_parameter('correct_xy_m', 0.10)
        self.declare_parameter('correct_yaw_deg', 3.0)
        self.declare_parameter('correct_cooldown_s', 5.0)
        self.declare_parameter('correct_max_v', 0.10)
        self.declare_parameter('correct_max_w', 0.20)

        g = self.get_parameter
        self._corrector = LandmarkCorrector(
            window=g('correct_window').value,
            spread_xy_m=g('correct_spread_xy_m').value,
            spread_yaw_deg=g('correct_spread_yaw_deg').value,
            xy_m=g('correct_xy_m').value,
            yaw_deg=g('correct_yaw_deg').value,
            cooldown_s=g('correct_cooldown_s').value,
            max_dist_m=g('correct_max_dist_m').value,
            max_v=g('correct_max_v').value,
            max_w=g('correct_max_w').value)
        self._v = 0.0
        self._w = 0.0
        self._done = False
        self._nav_busy = False
        self._switching = False
        self._last_switch_t = 0.0
        # Transit candidate: consecutive-frame vote for one foreign map.
        self._cand = {'map': None, 'count': 0, 'pose': None, 'name': None}
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._corr_pub = self.create_publisher(
            Float32MultiArray, '/tag_relocalizer/correction', 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self._load_map_cli = self.create_client(LoadMap, '/map_server/load_map')
        self.create_subscription(
            AprilTagDetectionArray, '/detections', self._on_detections, 10)
        self.create_subscription(
            String, '/nav_state', self._on_nav_state, LATCHED_QOS)
        self.create_service(
            Trigger, '/tag_relocalizer/reseed', self._on_reseed)
        self.get_logger().info('waiting for a registered tag to seed /initialpose')

    # --- registry -------------------------------------------------------------

    def _registered(self, family, tag_id):
        """{'name', 'poses': {map: {'x','y','yaw'}}} for a registered tag with
        at least one usable survey, or None. A tag is surveyed per MAP (tags.db
        tag_poses; a file not yet migrated falls back to its legacy columns).
        Read fresh each sighting — the portable base is re-surveyed whenever
        detect_apriltags or apriltag_watch sees it, and this must pick up the
        newest pose. Runs in the per-frame callback: a corrupt, locked or
        half-migrated file means "not registered", never an exception."""
        path = self.get_parameter('tags_db').value
        try:
            if not os.path.exists(path):
                return None
            db = sqlite3.connect(
                f'file:{path}?mode=ro', uri=True, timeout=REGISTRY_BUSY_S)
            try:
                nf = _norm_family(family)
                for tag in db.execute(
                        'SELECT family, tag_id, name FROM tags').fetchall():
                    if tag[1] == tag_id and _norm_family(tag[0]) == nf:
                        poses = tag_homes.registry_poses(db, tag[0], tag[1])
                        if not poses or not isinstance(tag[2], str):
                            return None
                        return {'name': tag[2], 'poses': poses}
            finally:
                db.close()
        except (sqlite3.Error, ValueError, TypeError, AttributeError,
                OverflowError) as e:
            self.get_logger().warn(
                f'tags.db read failed: {e}', throttle_duration_sec=30.0)
        return None

    def _read_site(self):
        """(active_map, slam_mode, maps, mode) from site.json, read fresh per
        frame (same live-switch story as the tags.db reopen). `slam_mode` is
        the raw policy; `mode` is what it actually runs as (core.sites.
        effective_mode: `auto` on a drawing map with its grid is localization,
        so amcl is running). `maps` is the raw name -> entry mapping ({} for
        v1), used only to read each map's source. Tolerates v1 (default_map)
        and v2 (active_map); (None, None, {}, None) on any problem = transit
        disabled, seeding behaves as before."""
        path = self.get_parameter('site_json').value
        try:
            with open(path) as f:
                data = json.load(f)
        except (OSError, json.JSONDecodeError):
            return None, None, {}, None
        if not isinstance(data, dict):
            return None, None, {}, None
        active = data.get('active_map') or data.get('default_map') or None
        maps = data.get('maps') if isinstance(data.get('maps'), dict) else {}
        mode = sites.effective_mode_from_data(
            data, self.get_parameter('maps_dir').value)
        return active, data.get('slam_mode'), maps, mode

    # --- seeding ----------------------------------------------------------------

    def _on_nav_state(self, msg):
        status, _, _ = parse_nav_state(msg.data)
        self._nav_busy = status in NAV_BUSY_STATES

    def _on_odom(self, msg):
        self._v = msg.twist.twist.linear.x
        self._w = msg.twist.twist.angular.z

    def _on_reseed(self, _req, resp):
        self._done = False
        self._corrector.reset()
        self._cand = {'map': None, 'count': 0, 'pose': None, 'name': None}
        self._last_switch_t = 0.0
        resp.success = True
        resp.message = ('re-armed: next registered-tag sighting seeds '
                        '/initialpose (same map) or switches maps (transit)')
        return resp

    def _on_detections(self, msg):
        if not msg.detections:
            return
        active_map, slam_mode, maps, mode = self._read_site()
        # Nearest registered tag in this frame wins, per bucket.
        best_same = None
        best_foreign = None
        saw_registered = False
        for det in msg.detections:
            entry = self._registered(det.family, det.id)
            if entry is None:
                continue
            kind, home, pose, cands = tag_homes.choose_home(
                entry['poses'], active_map, maps)
            if kind == 'refused':
                self.get_logger().warn(
                    f"tag '{entry['name']}' has no survey on the active "
                    f"{sites.map_source(maps, active_map)} map "
                    f"'{active_map}' but is surveyed on {', '.join(cands)} — "
                    "transit needs exactly one map of the active map's "
                    'source to go to; not switching',
                    throttle_duration_sec=30.0)
                continue
            if kind == 'none':
                continue
            saw_registered = True
            solved = self._solve(det, {
                'name': entry['name'], 'map_x': pose['x'],
                'map_y': pose['y'], 'map_yaw': pose['yaw']})
            if not solved:
                continue
            if kind == 'active':
                if best_same is None or solved[0] < best_same[0]:
                    best_same = solved
            elif best_foreign is None or solved[0] < best_foreign[1]:
                best_foreign = (home,) + solved
        if best_foreign is not None:
            self._track_transit(*best_foreign, slam_mode=slam_mode,
                                active_map=active_map, maps=maps, mode=mode)
        elif saw_registered and self._cand['count']:
            # Same-map tags with no foreign vote decay a stale candidate.
            self._cand['count'] -= 1
        if best_same is None:
            return
        if self._done:
            self._maybe_correct(*best_same)
            return
        dist, name, theta, pose = best_same
        self._pub.publish(pose)
        self._done = True
        self._corrector.reset()
        self.get_logger().info(
            'seeded /initialpose from tag %r %.2f m away: map (%.2f, %.2f) yaw %.1f deg'
            % (name, dist, pose.pose.pose.position.x,
               pose.pose.pose.position.y, math.degrees(theta)))

    def _maybe_correct(self, dist, name, theta, pose):
        """Post-seed landmark correction: compare the window-median tag solve
        with the current amcl pose (TF map->base_link) and re-seed on drift.
        No TF = silently behave as the old one-shot node."""
        if self._switching:
            return
        m = lookup_matrix(self._tf, 'map', 'base_link')
        if m is None:
            return
        rot, t = m
        amcl = (float(t[0]), float(t[1]),
                math.atan2(float(rot[1, 0]), float(rot[0, 0])))
        p = pose.pose.pose
        res = self._corrector.add(
            (p.position.x, p.position.y, theta), amcl,
            self._v, self._w, dist, time.monotonic())
        if res is None:
            return
        dx, dy, dyaw, dist, (mx, my, myaw) = res
        pose.pose.pose.position.x = mx
        pose.pose.pose.position.y = my
        qz, qw = yaw_to_quat_zw(myaw)
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        pose.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(pose)
        out = Float32MultiArray()
        out.data = [float(dx), float(dy), float(math.degrees(dyaw)),
                    float(dist)]
        self._corr_pub.publish(out)
        self.get_logger().info(
            'landmark correction #%d via tag %r %.2f m: dx %.3f dy %.3f '
            'dyaw %.2f deg -> map (%.2f, %.2f) yaw %.1f deg'
            % (self._corrector.count, name, dist, dx, dy,
               math.degrees(dyaw), mx, my, math.degrees(myaw)))

    # --- floor transit (ADR-0029) --------------------------------------------

    def _track_transit(self, home, dist, name, theta, pose, *,
                       slam_mode, active_map, maps, mode):
        if home == self._cand['map']:
            self._cand['count'] += 1
        else:
            self._cand = {'map': home, 'count': 1, 'pose': None, 'name': name}
        self._cand['pose'] = pose  # newest solve is the freshest geometry
        self._cand['name'] = name
        if self._cand['count'] < self.get_parameter(
                'min_transit_sightings').value:
            return
        if self._switching:
            return
        if time.monotonic() - self._last_switch_t < self.get_parameter(
                'transit_cooldown_s').value:
            return
        # map_server only exists in localization mode (ADR-0028) — the mode
        # check and the service check are belt and braces for the same fact.
        # `mode` is the EFFECTIVE mode: `auto` on a drawing map runs amcl.
        if mode != 'localization' or not self._load_map_cli.service_is_ready():
            self.get_logger().warn(
                f"tag '{name}' lives on map '{home}' but slam_mode is "
                f"'{slam_mode}' — not switching (localization mode only)",
                throttle_duration_sec=30.0)
            return
        if sites.sources_differ(maps, active_map, home):
            # amcl's params are fixed at launch, so a scan <-> drawing transit
            # cannot be a live LoadMap; restarting slam is an operator action.
            self.get_logger().warn(
                f"tag '{name}' lives on {sites.map_source(maps, home)} map "
                f"'{home}' but the active map '{active_map}' is a "
                f'{sites.map_source(maps, active_map)} map — not switching '
                'live: a cross-source switch needs a slam restart (operator '
                'action)', throttle_duration_sec=30.0)
            return
        if self._nav_busy:
            self.get_logger().warn(
                f"tag '{name}' says floor transit to '{home}' but a nav goal "
                'is in flight — not switching', throttle_duration_sec=30.0)
            return
        yaml_path = os.path.join(
            self.get_parameter('maps_dir').value, home + '.yaml')
        if not os.path.exists(yaml_path):
            self.get_logger().warn(
                f"tag '{name}' lives on map '{home}' but it has no grid "
                '(.yaml) — re-save that map before localizing on it',
                throttle_duration_sec=30.0)
            return
        self._switching = True
        old = active_map
        req = LoadMap.Request()
        req.map_url = yaml_path
        future = self._load_map_cli.call_async(req)
        future.add_done_callback(
            lambda fut: self._on_map_loaded(fut, old, home, name))

    def _on_map_loaded(self, future, old, home, name):
        try:
            result = future.result().result
        except Exception as e:  # noqa: BLE001 — service died mid-call; cooldown + retry on next sighting
            self.get_logger().error(f'LoadMap for {home!r} failed: {e}')
            result = None
        if result != LoadMap.Response.RESULT_SUCCESS:
            self.get_logger().error(
                f'map_server refused {home!r} (result={result})')
            self._last_switch_t = time.monotonic()  # cooldown a broken yaml too
            self._switching = False
            return
        pose = self._cand['pose']
        # Let amcl consume the republished latched /map before the seed.
        self._seed_timer = self.create_timer(
            SEED_DELAY_S, lambda: self._finish_transit(pose, old, home, name))

    def _finish_transit(self, pose, old, home, name):
        self._seed_timer.cancel()
        pose.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(pose)
        self._done = True
        self._cand = {'map': None, 'count': 0, 'pose': None, 'name': None}
        self._last_switch_t = time.monotonic()
        self._switching = False
        self.get_logger().info(
            'floor transit: %r -> %r via tag %r: map (%.2f, %.2f)'
            % (old, home, name, pose.pose.pose.position.x,
               pose.pose.pose.position.y))
        threading.Thread(
            target=self._persist_active_map, args=(home,), daemon=True).start()

    def _persist_active_map(self, home):
        """fleet_status owns site.json — POST the new active_map so the switch
        survives restarts. Best effort: on failure the live map and site.json
        disagree (visible in the webui) until the operator fixes it; the next
        slam restart reverts to site.json's map and the tag re-transits."""
        url = f"{self.get_parameter('fleet_api').value}/api/sites/active"
        body = json.dumps({'active_map': home}).encode()
        for attempt in range(3):
            try:
                req = urllib.request.Request(
                    url, data=body, method='POST',
                    headers={'Content-Type': 'application/json'})
                with urllib.request.urlopen(req, timeout=5.0) as r:
                    if r.status == 200:
                        return
            except OSError:
                pass
            time.sleep(2.0 * (attempt + 1))
        self.get_logger().error(
            f'could not persist active_map={home!r} to {url} — site.json now '
            'disagrees with the live map; set it in the webui Site panel')

    def _solve(self, det, entry):
        """(distance, name, theta, PoseWithCovarianceStamped) or None."""
        # apriltag_ros 3.4.0 publishes the child frame as "<family>:<id>"
        # with no "tag" prefix; older configs carried one. Try both.
        m = None
        for frame in (f'{det.family}:{det.id}', f'tag{det.family}:{det.id}'):
            m = lookup_matrix(self._tf, 'base_link', frame)
            if m is not None:
                break
        if m is None:
            return None
        rot, t = m

        dist = math.hypot(float(t[0]), float(t[1]))
        if dist > self.get_parameter('max_tag_dist_m').value:
            return None

        # Face normal in base_link: tag z-axis (third rotation column),
        # floor-projected, pointed at the robot (the base_link origin).
        nx, ny = float(rot[0, 2]), float(rot[1, 2])
        n = math.hypot(nx, ny)
        if n < MIN_NORMAL_PROJ:
            return None  # tag lying flat — no heading to seed from
        nx, ny = nx / n, ny / n
        if nx * -float(t[0]) + ny * -float(t[1]) < 0.0:
            nx, ny = -nx, -ny

        # map_yaw points AT the face (standoff heading) => face normal in map.
        face_map = entry['map_yaw'] + math.pi
        theta = face_map - math.atan2(ny, nx)
        c, s = math.cos(theta), math.sin(theta)
        rx = entry['map_x'] - (c * float(t[0]) - s * float(t[1]))
        ry = entry['map_y'] - (s * float(t[0]) + c * float(t[1]))

        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = rx
        pose.pose.pose.position.y = ry
        qz, qw = yaw_to_quat_zw(theta)
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw
        cov_xy = self.get_parameter('cov_xy').value
        pose.pose.covariance[0] = cov_xy
        pose.pose.covariance[7] = cov_xy
        pose.pose.covariance[35] = self.get_parameter('cov_yaw').value
        return dist, entry['name'], theta, pose


def main(args=None):
    run_node(TagRelocalizer, args=args)


if __name__ == '__main__':
    main()
