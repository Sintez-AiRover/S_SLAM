#!/usr/bin/env python3
"""Repeatable scripted drive for the 3-rover crop-field sim (ROS_DOMAIN_ID=43).

WHY THIS EXISTS
    Hand teleop is not reproducible, so "did that config change help?" could
    never be answered -- two runs differ more by driving than by tuning. This
    drives a FIXED pattern so runs are comparable, and logs ground truth so
    the answer is a number rather than an impression.

WHY IT IS CLOSED-LOOP ON GROUND TRUTH
    Blind open-loop drives have repeatedly ended with rovers climbed on top of
    crop cylinders (pitched ~42 deg, lidar pointing at the sky, scans matching
    nothing). This tracks the true world pose from Ignition and refuses to keep
    driving a rover that is tilting, stalled, off its lane, or too close to an
    obstacle -- it stops that rover and reports, instead of grinding into the
    crop.

    Ground truth comes from the streaming Ignition topic
    /world/rover_world/dynamic_pose/info. Do NOT use `ign model -m X -p` in a
    control loop: it costs ~5 s per call.

THE PATTERN
    Three lanes exist between the crop rows: A (y=-2.4), B (y=0.0, the wide
    one), C (y=+2.4). Each phase, every rover drives its assigned lane west to
    east and back again (the return leg is a deliberate revisit, which is what
    lets intra-robot loop closures fire). Then lane assignments ROTATE, so each
    rover ends up driving ground the other two have already mapped -- that
    overlap is what inter-robot loop closures need.

    Rovers are always in DIFFERENT lanes at the same time (assignments are a
    rotation), and lane changes happen one rover at a time on the west
    headland, so they cannot collide with each other.

USAGE
    export ROS_DOMAIN_ID=43 && source /root/ros2_ws/install/setup.bash
    python3 /root/ros2_ws/rover_drive_pattern.py                 # 3 phases
    python3 /root/ros2_ws/rover_drive_pattern.py --phases 2 --speed 0.3
    python3 /root/ros2_ws/rover_drive_pattern.py --log /tmp/drive.csv

    Ctrl-C stops all rovers cleanly.
"""

import argparse
import csv
import math
import os
import re
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Twist

GT_TOPIC = '/world/rover_world/dynamic_pose/info'
WORLD_SDF = ('/root/ros2_ws/install/rover_description/share/rover_description/'
             'worlds/crop_field.sdf')

# Field geometry (see crop_field.sdf). Crop rows sit at y = -3.0, -1.8, +1.8,
# +3.0 with plants from x=1.5 to x=9.9; lanes are the gaps between them.
LANE_Y = {'A': -2.4, 'B': 0.0, 'C': 2.4}
# The ring's two vertical legs run along these x values, so each must be a
# clear north-south corridor for the whole lane span -- and wide enough to
# absorb WAYPOINT_TOL, because a rover starts its turn up to that far short of
# the corner and then tracks the new leg from there.
#
# West: passing BETWEEN the boxes (0.6, +-0.7) and the first plant row (x=1.5)
# leaves a legal band of only [1.017, 1.174] -- 0.157 m, narrower than twice
# the tolerance, and it duly failed. Going WEST of the boxes instead gives
# [-0.383, 0.183], a 0.565 m band; -0.10 is its centre.
X_WEST = -0.10
# East: bounded by the last plants (x=9.9) and landmark lm_e1 (x=11.2,
# r=0.25) -> legal band [10.226, 10.674]; 10.45 is its centre. x=10.3 was
# tried first and cornering short by the tolerance put a rover 0.316 m from
# the plant at (9.9,-1.8), inside the 0.326 m requirement.
X_EAST = 10.45

# Rover chassis is 0.234 x 0.207 m -> half-diagonal 0.156 m.
ROVER_HALF = 0.156
OBSTACLE_MARGIN = 0.12     # extra clearance demanded beyond the two radii
LANE_DEVIATION_MAX = 0.35  # abort a rover that wanders this far off lane centre
TILT_MAX_DEG = 12.0        # above this the rover is climbing something
STALL_SECS = 5.0           # commanded to move but hasn't -> stuck
STALL_DIST = 0.05

WAYPOINT_TOL = 0.15
# Below this range, never turn in place: close to a waypoint a few cm of
# cross-track error produces a huge bearing error, and spinning on the spot
# cannot reduce the distance -- the rover just rotates until the stall guard
# trips. Inside CLOSE_RANGE we drive straight through with gentle correction.
CLOSE_RANGE = 0.45
TURN_THRESH = 0.25         # rad: beyond this, turn in place before driving
K_ANG = 1.6
K_LIN = 0.8
MAX_ANG = 0.8              # DiffDrive caps at 1.0 rad/s
CONTROL_HZ = 10.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def quat_rpy(qx, qy, qz, qw):
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    s = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
    return roll, math.asin(s), math.atan2(2 * (qw * qz + qx * qy),
                                          1 - 2 * (qy * qy + qz * qz))


class GroundTruth:
    """Streams every model's true world pose from Ignition transport."""

    def __init__(self, names):
        self.names = set(names)
        self.poses = {}
        self._lock = threading.Lock()
        self._stop = False
        self._p = subprocess.Popen(['ign', 'topic', '-e', '-t', GT_TOPIC],
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.DEVNULL,
                                   text=True, bufsize=1)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        name, vals, section = None, {}, None
        for line in self._p.stdout:
            if self._stop:
                return
            line = line.strip()
            m = re.match(r'name:\s*"([^"]+)"', line)
            if m:
                name, vals, section = m.group(1), {}, None
                continue
            if line.startswith('position'):
                section = 'p'
                continue
            if line.startswith('orientation'):
                section = 'o'
                continue
            m = re.match(r'([xyzw]):\s*(-?[\d.eE+-]+)', line)
            if m and name and section:
                vals[section + m.group(1)] = float(m.group(2))
                if section == 'o' and 'ow' in vals and len(vals) >= 7:
                    if name in self.names:
                        r, p, y = quat_rpy(vals['ox'], vals['oy'],
                                           vals['oz'], vals['ow'])
                        with self._lock:
                            self.poses[name] = (vals['px'], vals['py'],
                                                vals['pz'], r, p, y)
                    name, section = None, None

    def get(self, name):
        with self._lock:
            return self.poses.get(name)

    def close(self):
        self._stop = True
        try:
            self._p.terminate()
        except Exception:
            pass


def load_obstacles(path):
    """[(x, y, radius)] for every static model in the world.

    Parsed from the SDF rather than hardcoded so landmarks added to the world
    are picked up automatically.
    """
    obs = []
    try:
        world = ET.parse(path).getroot().find('world')
    except (OSError, ET.ParseError) as e:
        print(f'WARNING: could not parse {path} ({e}); obstacle guard disabled')
        return obs
    for model in world.findall('model'):
        name = model.get('name', '')
        if name.startswith('rover_') or name == 'ground':
            continue
        pose = model.find('pose')
        if pose is None or not pose.text:
            continue
        parts = pose.text.split()
        if len(parts) < 2:
            continue
        x, y = float(parts[0]), float(parts[1])
        rad = 0.15
        cyl = model.find('.//cylinder/radius')
        box = model.find('.//box/size')
        if cyl is not None:
            rad = float(cyl.text)
        elif box is not None and box.text:
            s = box.text.split()
            rad = 0.5 * math.hypot(float(s[0]), float(s[1]))
        obs.append((x, y, rad))
    return obs


class Rover:
    def __init__(self, node, idx, lane_order):
        self.idx = idx
        self.name = f'rover_{idx}'
        self.pub = node.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        self.lane_order = lane_order
        self.plan = []
        self.pi = 0
        self.state = 'run'          # run | done | fault
        self.fault = None
        self.lane_y = None
        self.moved_ref = None
        self.moved_t = time.time()
        self.clash_since = None
        self.dist_driven = 0.0
        self._last_xy = None

    def stop(self):
        t = Twist()
        t.linear.x = float(0.0)
        t.angular.z = float(0.0)
        self.pub.publish(t)

    def command(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.pub.publish(t)


# The circuit every rover drives, as (x, y, lane_centre_or_None). lane_centre
# is the y the deviation guard holds the rover to while it is between crop
# rows; None on the open headlands where no such constraint applies.
#
# It threads all three lanes and closes on itself, so a rover revisits its own
# earlier path every lap (intra-robot loop closures) and drives over ground the
# other two have already mapped (inter-robot loop closures).
RING = [
    (X_EAST, LANE_Y['A'], LANE_Y['A']),   # lane A, eastbound
    (X_EAST, LANE_Y['B'], None),          # east headland, north
    (X_WEST, LANE_Y['B'], LANE_Y['B']),   # lane B, westbound
    (X_WEST, LANE_Y['C'], None),          # west headland, north
    (X_EAST, LANE_Y['C'], LANE_Y['C']),   # lane C, eastbound
    (X_EAST, LANE_Y['A'], None),          # east headland, south
    (X_WEST, LANE_Y['A'], LANE_Y['A']),   # lane A, westbound -> closes loop
]

# Where each rover joins the ring, as an index into RING. Spread around the
# circuit so the three never bunch up.
RING_ENTRY = [0, 2, 4]

# Staging targets: the point each rover drives to before the ring starts,
# which must be the waypoint immediately BEFORE its entry index.
STAGING = [
    (X_WEST, LANE_Y['A']),
    (X_EAST, LANE_Y['B']),
    (X_WEST, LANE_Y['C']),
]


def build_plan(rover, laps):
    """Staging, then `laps` circuits of the shared ring.

    All rovers travel the ring in the SAME direction. That is the whole point:
    the previous design rotated lane assignments between rovers, which is a
    3-cycle (r0 wants r1's lane, r1 wants r2's, r2 wants r0's) and deadlocks --
    nobody can move first, and the proximity guard freezes whoever tries.
    Same-direction circulation has no such conflict, and same-direction traffic
    also cannot meet head-on in a 1.2 m lane.
    """
    plan = []
    sx, sy = STAGING[rover.idx]
    lane_y = sy if abs(sx - X_EAST) < 1e-6 else None
    plan.append(('goto', sx, sy, lane_y))
    plan.append(('barrier', 'staged'))     # everyone starts the ring together
    idx = RING_ENTRY[rover.idx]
    for _ in range(laps):
        for k in range(len(RING)):
            x, y, ly = RING[(idx + k) % len(RING)]
            plan.append(('goto', x, y, ly))
    return plan


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--laps', type=int, default=3,
                    help='circuits of the shared ring each rover drives')
    ap.add_argument('--speed', type=float, default=0.35)
    ap.add_argument('--robots', type=int, default=3)
    ap.add_argument('--log', default='/root/ros2_ws/drive_gt_log.csv')
    ap.add_argument('--timeout', type=float, default=1800.0)
    args = ap.parse_args()

    names = [f'rover_{i}' for i in range(args.robots)]
    obstacles = load_obstacles(WORLD_SDF)
    print(f'[drive] {len(obstacles)} static obstacles loaded from world')

    gt = GroundTruth(names)
    print('[drive] waiting for ground-truth stream...')
    t0 = time.time()
    while time.time() - t0 < 20 and any(gt.get(n) is None for n in names):
        time.sleep(0.3)
    missing = [n for n in names if gt.get(n) is None]
    if missing:
        print(f'ERROR: no ground truth for {missing}. Is the sim running on '
              f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID")}?')
        gt.close()
        return 1

    # Refuse to start if a rover is already tilted -- it is stuck on something
    # and driving would only grind it further.
    bad = []
    for n in names:
        _, _, _, r, p, _ = gt.get(n)
        if max(abs(math.degrees(r)), abs(math.degrees(p))) > TILT_MAX_DEG:
            bad.append((n, math.degrees(r), math.degrees(p)))
    if bad:
        print('ERROR: rovers are already tilted (stuck on an obstacle):')
        for n, r, p in bad:
            print(f'   {n}: roll={r:.1f} deg pitch={p:.1f} deg')
        print('Restart the sim to reset them, then re-run this script.')
        gt.close()
        return 2

    rclpy.init()
    node = rclpy.create_node('rover_drive_pattern')

    rovers = []
    for i in range(args.robots):
        rv = Rover(node, i, [])
        rv.plan = build_plan(rv, args.laps)
        rovers.append(rv)
        print(f'[drive] rover_{i}: joins ring at waypoint {RING_ENTRY[i]}, '
              f'{args.laps} laps, {len(rv.plan)} plan items')

    log = open(args.log, 'w', newline='')
    writer = csv.writer(log)
    writer.writerow(['t', 'rover', 'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                     'state', 'waypoint_idx'])

    start = time.time()
    last_progress = [-1, start]
    period = 1.0 / CONTROL_HZ
    last_log = 0.0
    last_report = 0.0

    def barrier_tag(rv):
        if rv.pi < len(rv.plan) and rv.plan[rv.pi][0] == 'barrier':
            return rv.plan[rv.pi][1]
        return None

    try:
        while rclpy.ok():
            now = time.time()
            if now - start > args.timeout:
                print('[drive] TIMEOUT reached, stopping')
                break

            # Global progress watchdog. If the whole fleet stops advancing
            # through its plan, something is wedged -- say so and stop rather
            # than sitting there looking busy.
            progress = sum(r.pi for r in rovers)
            if progress != last_progress[0]:
                last_progress = [progress, now]
            elif now - last_progress[1] > 150.0:
                print(f'[drive] !! NO PROGRESS for 150 s (fleet wedged at '
                      f'{[r.pi for r in rovers]}), aborting')
                break

            active = [r for r in rovers if r.state == 'run']
            if not active:
                break

            # --- barrier resolution -------------------------------------
            # A barrier releases when every still-running rover is waiting on
            # that same tag. Faulted rovers are excluded so one stuck rover
            # cannot deadlock the rest of the fleet.
            tags = {}
            for rv in active:
                t = barrier_tag(rv)
                if t:
                    tags.setdefault(t, []).append(rv)
            for tag, waiting in tags.items():
                if len(waiting) == len(active):
                    for rv in waiting:
                        rv.pi += 1

            for rv in rovers:
                if rv.state != 'run':
                    continue
                pose = gt.get(rv.name)
                if pose is None:
                    continue
                x, y, z, roll, pitch, yaw = pose

                if rv._last_xy is not None:
                    rv.dist_driven += math.dist((x, y), rv._last_xy)
                rv._last_xy = (x, y)

                if now - last_log > 0.5:
                    writer.writerow([f'{now - start:.2f}', rv.name,
                                     f'{x:.4f}', f'{y:.4f}', f'{z:.4f}',
                                     f'{roll:.4f}', f'{pitch:.4f}',
                                     f'{yaw:.4f}', rv.state, rv.pi])

                # --- safety guards ---------------------------------------
                tilt = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
                if tilt > TILT_MAX_DEG:
                    rv.state, rv.fault = 'fault', f'tilted {tilt:.1f} deg (climbing an obstacle)'
                    rv.stop()
                    print(f'[drive] !! {rv.name} FAULT: {rv.fault} at ({x:.2f},{y:.2f})')
                    continue

                if rv.pi >= len(rv.plan):
                    rv.state = 'done'
                    rv.stop()
                    print(f'[drive] {rv.name} finished, drove {rv.dist_driven:.1f} m')
                    continue

                item = rv.plan[rv.pi]
                if item[0] == 'barrier':
                    rv.stop()
                    continue

                _, tx, ty, lane_y = item

                if lane_y is not None and abs(y - lane_y) > LANE_DEVIATION_MAX:
                    rv.state, rv.fault = 'fault', (
                        f'off lane centre by {abs(y - lane_y):.2f} m')
                    rv.stop()
                    print(f'[drive] !! {rv.name} FAULT: {rv.fault}')
                    continue

                near = None
                for ox, oy, orad in obstacles:
                    d = math.hypot(ox - x, oy - y) - orad - ROVER_HALF
                    if near is None or d < near[0]:
                        near = (d, ox, oy)
                if near and near[0] < OBSTACLE_MARGIN:
                    rv.state, rv.fault = 'fault', (
                        f'obstacle within {near[0]:.2f} m at '
                        f'({near[1]:.2f},{near[2]:.2f})')
                    rv.stop()
                    print(f'[drive] !! {rv.name} FAULT: {rv.fault}')
                    continue

                clash = None
                for other in rovers:
                    if other is rv:
                        continue
                    op = gt.get(other.name)
                    if op and math.dist((x, y), (op[0], op[1])) < 2 * ROVER_HALF + 0.15:
                        clash = other.name
                if clash:
                    # Yielding to a nearby rover is normal on a shared ring, but
                    # it must not be able to wait forever: an earlier design
                    # deadlocked here silently for ten minutes.
                    if rv.clash_since is None:
                        rv.clash_since = now
                    elif now - rv.clash_since > 30.0:
                        rv.state, rv.fault = 'fault', f'blocked by {clash} for 30 s'
                        rv.stop()
                        print(f'[drive] !! {rv.name} FAULT: {rv.fault}')
                        continue
                    rv.stop()
                    continue
                rv.clash_since = None

                # --- waypoint control ------------------------------------
                dx, dy = tx - x, ty - y
                dist = math.hypot(dx, dy)
                if dist < WAYPOINT_TOL:
                    rv.pi += 1
                    rv.stop()
                    rv.moved_ref = None
                    continue

                yaw_err = wrap(math.atan2(dy, dx) - yaw)
                if abs(yaw_err) > TURN_THRESH and dist > CLOSE_RANGE:
                    lin = 0.0
                    ang = max(-MAX_ANG, min(MAX_ANG, K_ANG * yaw_err))
                else:
                    lin = min(args.speed, K_LIN * dist)
                    ang = max(-MAX_ANG / 2, min(MAX_ANG / 2, K_ANG * yaw_err))
                rv.command(lin, ang)

                # --- stall detection -------------------------------------
                # Rotating counts as progress: a rover turning on the spot is
                # working, not stuck, and must not be failed for it.
                if rv.moved_ref is None:
                    rv.moved_ref, rv.moved_t = (x, y, yaw), now
                elif (math.dist((x, y), rv.moved_ref[:2]) > STALL_DIST
                        or abs(wrap(yaw - rv.moved_ref[2])) > 0.15):
                    rv.moved_ref, rv.moved_t = (x, y, yaw), now
                elif now - rv.moved_t > STALL_SECS and (lin > 0.05 or abs(ang) > 0.05):
                    rv.state, rv.fault = 'fault', 'stalled (commanded but not moving)'
                    rv.stop()
                    print(f'[drive] !! {rv.name} FAULT: {rv.fault} at ({x:.2f},{y:.2f})')

            if now - last_log > 0.5:
                last_log = now
            if now - last_report > 20.0:
                last_report = now
                bits = []
                for rv in rovers:
                    p = gt.get(rv.name)
                    loc = f'({p[0]:5.2f},{p[1]:5.2f})' if p else '(?)'
                    bits.append(f'{rv.name} {rv.state} wp{rv.pi}/{len(rv.plan)} {loc}')
                print(f'[{now - start:6.1f}s] ' + ' | '.join(bits))

            time.sleep(period)

    except KeyboardInterrupt:
        print('\n[drive] interrupted')
    finally:
        for rv in rovers:
            for _ in range(5):
                rv.stop()
                time.sleep(0.02)
        log.close()
        gt.close()
        print('\n=== drive summary ===')
        for rv in rovers:
            msg = f'  {rv.name}: {rv.state}, {rv.dist_driven:.1f} m driven'
            if rv.fault:
                msg += f'  FAULT: {rv.fault}'
            print(msg)
        print(f'  ground-truth log: {args.log}')
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
