#!/usr/bin/env python3
# Sanity-checks a directory written by map_saver_trigger.py. Not part of
# the persistence feature itself (no reload exists yet to round-trip
# against) -- this is a standalone diagnostic: numeric sanity checks plus
# an optional 2D plot of the saved poses, colored by robot, so you can
# visually compare against what you saw in RViz.
#
# Usage: python3 check_saved_map.py --dir <path> [--plot out.png]

import argparse
import json
import os
import sys

import numpy as np


def load_poses(path):
    # robot_id keyframe_id x y z qx qy qz qw
    return np.loadtxt(path, ndmin=2)


def load_edges(path):
    # robot_id_from kf_from robot_id_to kf_to x y z qx qy qz qw sx sy sz srx sry srz
    return np.loadtxt(path, ndmin=2)


def check(args):
    problems = []
    warnings = []

    poses_path = os.path.join(args.dir, 'poses.txt')
    edges_path = os.path.join(args.dir, 'edges.txt')
    manifest_path = os.path.join(args.dir, 'manifest.json')

    for p in (poses_path, edges_path, manifest_path):
        if not os.path.isfile(p):
            problems.append('missing file: {}'.format(p))
    if problems:
        return problems, warnings, None

    poses = load_poses(poses_path)
    edges = load_edges(edges_path)
    manifest = json.load(open(manifest_path))

    print('poses.txt: {} rows, edges.txt: {} rows'.format(len(poses), len(edges)))

    # 1. No NaN/Inf anywhere
    if not np.isfinite(poses).all():
        problems.append('poses.txt contains NaN/Inf values')
    if not np.isfinite(edges).all():
        problems.append('edges.txt contains NaN/Inf values')

    # 2. Quaternions should be unit norm (corruption/formatting bugs show
    # up here first)
    quat_cols = poses[:, 5:9]
    norms = np.linalg.norm(quat_cols, axis=1)
    bad = np.abs(norms - 1.0) > 0.01
    if bad.any():
        problems.append('{} pose quaternions are not unit-norm '
                        '(min={:.4f} max={:.4f})'.format(
                            bad.sum(), norms.min(), norms.max()))

    edge_quat = edges[:, 7:11]
    edge_norms = np.linalg.norm(edge_quat, axis=1)
    bad_e = np.abs(edge_norms - 1.0) > 0.01
    if bad_e.any():
        problems.append('{} edge quaternions are not unit-norm'.format(bad_e.sum()))

    # 3. Noise std values should be positive
    noise_std = edges[:, 11:17]
    if (noise_std <= 0).any():
        problems.append('some edges.txt noise_std values are <= 0')

    # 4. robot_id set sanity
    robot_ids_in_poses = set(poses[:, 0].astype(int))
    expected_robots = set(range(manifest['max_nb_robots']))
    missing_robots = expected_robots - robot_ids_in_poses
    if missing_robots:
        warnings.append('robots {} have no poses saved -- they were not '
                        'merged into the optimizer\'s frame at save time'
                        .format(sorted(missing_robots)))

    # 5. Cross-check against manifest's last_keyframe_id (the orphan-
    # detection insurance map_saver_trigger.py was built to enable):
    # if the saved graph's max keyframe_id per robot falls far short of
    # what the trigger observed live, poses may be missing/incomplete.
    for robot_id_str, last_kf in manifest['last_keyframe_id'].items():
        robot_id = int(robot_id_str)
        rows = poses[poses[:, 0].astype(int) == robot_id]
        if len(rows) == 0:
            continue
        max_kf_saved = int(rows[:, 1].max())
        if max_kf_saved < last_kf * 0.5:
            warnings.append(
                'robot {}: manifest saw keyframe {} live, but the saved '
                'graph only goes up to keyframe {} -- large gap, was the '
                'save triggered mid-optimization?'.format(
                    robot_id, last_kf, max_kf_saved))

    # 6. Descriptor DB presence + shape sanity, cross-checked against poses
    for robot_id in range(manifest['max_nb_robots']):
        meta_path = os.path.join(args.dir, 'robot{}_local_meta.json'.format(robot_id))
        data_path = os.path.join(args.dir, 'robot{}_local_data.npy'.format(robot_id))
        if not os.path.isfile(meta_path) or not os.path.isfile(data_path):
            warnings.append('robot {}: descriptor DB not saved (missing '
                            '{}ーis loop_closure_detection_node running for '
                            'this robot?)'.format(robot_id, os.path.basename(data_path)))
            continue
        meta = json.load(open(meta_path))
        arr = np.load(data_path)
        if arr.shape != (meta['n'], meta['dim']):
            problems.append('robot {}: descriptor array shape {} does not '
                            'match meta n={} dim={}'.format(
                                robot_id, arr.shape, meta['n'], meta['dim']))
        if not np.isfinite(arr).all():
            problems.append('robot {}: descriptor data contains NaN/Inf'.format(robot_id))

    return problems, warnings, poses


def plot(poses, out_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))
    colors = ['tab:red', 'tab:green', 'tab:blue', 'tab:orange', 'tab:purple']
    for robot_id in sorted(set(poses[:, 0].astype(int))):
        rows = poses[poses[:, 0].astype(int) == robot_id]
        ax.plot(rows[:, 2], rows[:, 3], '.', markersize=2,
               color=colors[robot_id % len(colors)],
               label='robot {} ({} poses)'.format(robot_id, len(rows)))
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_aspect('equal')
    ax.legend()
    ax.set_title('Saved map poses (top-down) -- compare against RViz')
    fig.savefig(out_path, dpi=150)
    print('Wrote plot to {}'.format(out_path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dir', required=True)
    parser.add_argument('--plot', help='Output PNG path for a top-down pose plot')
    args = parser.parse_args()

    problems, warnings, poses = check(args)

    if problems:
        print('\nPROBLEMS (likely a real bug):')
        for p in problems:
            print('  - ' + p)
    if warnings:
        print('\nWARNINGS (may be expected, e.g. robot not yet merged):')
        for w in warnings:
            print('  - ' + w)
    if not problems and not warnings:
        print('\nAll checks passed cleanly.')

    if args.plot and poses is not None:
        plot(poses, args.plot)

    sys.exit(1 if problems else 0)


if __name__ == '__main__':
    main()
