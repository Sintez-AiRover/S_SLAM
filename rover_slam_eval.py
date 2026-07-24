#!/usr/bin/env python3
"""Accuracy evaluation for rover Swarm-SLAM against Ignition ground truth.

Reports the two numbers that actually matter for a merged multi-robot map:

  solo ATE   per-robot RMSE after aligning that robot's own estimated
             trajectory to its own ground truth. Measures drift only.
  joint ATE  RMSE after ONE shared alignment computed over all merged
             robots together. Includes inter-robot merge error, so the
             joint-minus-solo gap is exactly "how bad is the merge".

  baseline   error in the distance between each pair of rovers. Needs no
             alignment at all, so it is the most assumption-free check and
             is directly comparable across runs.

Alignment is yaw + translation only (never full 3D Kabsch). The S3E work
learned this the hard way: a full-3D fit on a near-straight trajectory has a
rotational gauge freedom and the aligned path spins about the travel axis.

Ground truth streams from Ignition transport; estimates come from cslam's
per-robot current_pose_estimate, which is published in the shared origin
frame once robots merge. Both are sampled in the same loop so the pairs are
simultaneous by construction.

USAGE
    export ROS_DOMAIN_ID=43 && source /root/ros2_ws/install/setup.bash
    python3 /root/ros2_ws/rover_slam_eval.py --duration 120
"""

import argparse
import itertools
import math
import re
import subprocess
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from cslam_common_interfaces.msg import PoseGraph

GT_TOPIC = '/world/rover_world/dynamic_pose/info'


class GroundTruth:
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
                        with self._lock:
                            self.poses[name] = (vals['px'], vals['py'])
                    name, section = None, None

    def get(self, n):
        with self._lock:
            return self.poses.get(n)

    def close(self):
        self._stop = True
        try:
            self._p.terminate()
        except Exception:
            pass


def align_yaw(src, dst):
    """Best yaw+translation taking src onto dst (both Nx2). Returns rmse.

    Deliberately NOT a full 3D Kabsch -- see module docstring.
    """
    if len(src) < 2:
        return None
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    cs, cd = src.mean(0), dst.mean(0)
    s, d = src - cs, dst - cd
    # Closed-form optimal rotation angle in 2D.
    num = float((s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]).sum())
    den = float((s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]).sum())
    th = math.atan2(num, den)
    R = np.array([[math.cos(th), -math.sin(th)],
                  [math.sin(th), math.cos(th)]])
    resid = (R @ s.T).T - d
    return float(np.sqrt((resid ** 2).sum(1).mean()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--robots', type=int, default=3)
    ap.add_argument('--rate', type=float, default=1.0, help='samples per second')
    ap.add_argument('--out', default='/root/ros2_ws/rover_eval_latest.txt')
    args = ap.parse_args()

    names = [f'rover_{i}' for i in range(args.robots)]
    gt = GroundTruth(names)

    rclpy.init()
    node = rclpy.create_node('rover_slam_eval')
    rel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
    est = {}
    graphs = {}
    for i in range(args.robots):
        node.create_subscription(PoseStamped,
                                 f'/r{i}/cslam/current_pose_estimate',
                                 lambda m, i=i: est.__setitem__(i, m), rel)
    node.create_subscription(PoseGraph, '/cslam/viz/pose_graph',
                             lambda m: graphs.__setitem__(m.robot_id, m), rel)

    time.sleep(3.0)
    samples = {i: [] for i in range(args.robots)}   # (gt_xy, est_xy)
    t_end = time.time() + args.duration
    nxt = 0.0
    print(f'[eval] sampling for {args.duration:.0f}s ...')
    while rclpy.ok() and time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.time()
        if now < nxt:
            continue
        nxt = now + 1.0 / args.rate
        for i in range(args.robots):
            g = gt.get(f'rover_{i}')
            e = est.get(i)
            if g and e:
                samples[i].append((g, (e.pose.position.x, e.pose.position.y)))

    lines = []

    def emit(s):
        print(s)
        lines.append(s)

    emit('=== rover Swarm-SLAM evaluation vs Ignition ground truth ===')
    origins = {}
    for rid in sorted(graphs):
        origins[rid] = graphs[rid].origin_robot_id
    groups = {}
    for rid, o in origins.items():
        groups.setdefault(o, []).append(rid)
    emit(f'merge groups (origin -> robots): '
         f'{ {o: sorted(r) for o, r in groups.items()} }')
    merged = sorted([r for o, rs in groups.items() if len(rs) > 1 for r in rs])
    emit(f'merged robots: {merged if merged else "NONE"}')

    emit('')
    emit('--- solo ATE (own drift only) ---')
    for i in range(args.robots):
        pairs = samples[i]
        if len(pairs) < 5:
            emit(f'  r{i}: only {len(pairs)} samples, skipped')
            continue
        rmse = align_yaw([p[1] for p in pairs], [p[0] for p in pairs])
        emit(f'  r{i}: {rmse:6.3f} m   ({len(pairs)} samples)')

    emit('')
    emit('--- joint ATE (one shared alignment; includes merge error) ---')
    if len(merged) >= 2:
        src, dst = [], []
        n = min(len(samples[i]) for i in merged)
        for i in merged:
            for k in range(n):
                g, e = samples[i][k]
                src.append(e)
                dst.append(g)
        rj = align_yaw(src, dst)
        emit(f'  all merged robots {merged}: {rj:6.3f} m   ({n} samples each)')
        for i in merged:
            pairs = samples[i][:n]
            rs = align_yaw([p[1] for p in pairs], [p[0] for p in pairs])
            emit(f'    r{i} solo {rs:6.3f} m  -> merge cost {rj - rs:+6.3f} m')
    else:
        emit('  fewer than 2 robots merged; joint ATE not meaningful')

    emit('')
    emit('--- inter-robot baseline error (alignment-free) ---')
    for a, b in itertools.combinations(range(args.robots), 2):
        n = min(len(samples[a]), len(samples[b]))
        if n < 5:
            emit(f'  r{a}-r{b}: too few samples')
            continue
        errs = []
        for k in range(n):
            ga, ea = samples[a][k]
            gb, eb = samples[b][k]
            errs.append(math.dist(ea, eb) - math.dist(ga, gb))
        errs = np.array(errs)
        emit(f'  r{a}-r{b}: mean {errs.mean():+6.3f} m  '
             f'absmax {np.abs(errs).max():6.3f} m  ({n} samples)')

    with open(args.out, 'a') as f:
        f.write('\n' + time.strftime('%Y-%m-%d %H:%M:%S') + '\n')
        f.write('\n'.join(lines) + '\n')
    emit('')
    emit(f'appended to {args.out}')

    gt.close()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
