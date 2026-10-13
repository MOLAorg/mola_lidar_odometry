#!/usr/bin/env python3
#                _
#  _ __ ___   ___ | | __ _
# | '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
# | | | | | | (_) | | (_| | Localization and mApping (MOLA)
# |_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola
#
# Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
#                         and individual contributors.
# SPDX-License-Identifier: GPL-3.0
# See LICENSE for full license information.

"""
Integration test: a complete, headless mola-cli run of a launch YAML on a
dataset, as a user would launch it, and its trajectory compared against a
reference one.

The launch YAML and its variables are given by the caller (CMake). This script
only adds what makes the run a test: no GUI, quit at the end of the dataset, a
lossless input queue, and the trajectory saved to a temporary file.
"""

import argparse
import math
import os
import subprocess
import sys
import tempfile


def load_tum(path):
    """Return a list of (t, [x, y, z], [qx, qy, qz, qw]) from a TUM file."""
    poses = []
    with open(path) as f:
        for line in f:
            fields = line.split()
            if not fields or fields[0].startswith('#'):
                continue
            v = [float(x) for x in fields[:8]]
            poses.append((v[0], v[1:4], v[4:8]))
    return poses


def rotation_angle_between(qa, qb):
    """Angle [rad] of the relative rotation between two unit quaternions."""
    dot = abs(sum(a * b for a, b in zip(qa, qb)))
    return 2.0 * math.acos(min(1.0, dot))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mola-cli', required=True, help='mola-cli executable')
    parser.add_argument('--launch', required=True, help='mola-cli launch YAML')
    parser.add_argument('--reference', required=True, help='Reference trajectory (TUM)')
    parser.add_argument('--max-position-error', type=float, default=0.5, help='[m]')
    parser.add_argument('--max-rotation-error', type=float, default=5.0, help='[deg]')
    parser.add_argument('--timeout', type=float, default=300.0, help='[s]')
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix='mola_cli_test_') as tmp:
        trajectory_file = os.path.join(tmp, 'estimated_trajectory.tum')
        env = dict(os.environ)
        env.update({
            'MOLA_WITH_GUI': 'false',
            'MOLA_QUIT_ON_DATASET_END': 'true',
            # Process every scan, so the result does not depend on host load:
            'MOLA_DROP_STALE_SCANS': 'false',
            'MOLA_ASYNC_BACKEND': 'false',
            'MOLA_SAVE_TRAJECTORY': 'true',
            'MOLA_TUM_TRAJECTORY_OUTPUT': trajectory_file,
            'MOLA_PROFILER': 'false',
        })

        cmd = [args.mola_cli, args.launch]
        print('Running:', ' '.join(cmd), flush=True)
        try:
            proc = subprocess.run(cmd, env=env, cwd=tmp, timeout=args.timeout)
        except subprocess.TimeoutExpired:
            print(f'FAIL: mola-cli did not finish within {args.timeout} s')
            return 1

        if proc.returncode != 0:
            print(f'FAIL: mola-cli exit code {proc.returncode}')
            return 1
        if not os.path.isfile(trajectory_file):
            print('FAIL: no trajectory was saved')
            return 1

        estimated = load_tum(trajectory_file)

    reference = load_tum(args.reference)
    print(f'Estimated poses: {len(estimated)}, reference poses: {len(reference)}')

    # Every scan must have been processed (the first one may only seed the map):
    if len(estimated) < len(reference) - 1:
        print('FAIL: too few estimated poses')
        return 1

    # Paired in order, like the in-process tests: a data source may stamp a
    # scan at a different instant of its sweep, so timestamps may not match.
    failures = 0
    for (t, p, q), (t_ref, p_ref, q_ref) in zip(estimated, reference):
        pos_err = math.dist(p, p_ref)
        rot_err = math.degrees(rotation_angle_between(q, q_ref))
        ok = pos_err < args.max_position_error and rot_err < args.max_rotation_error
        if not ok:
            failures += 1
        print(f't={t:.4f} (dt={t - t_ref:+.4f} s) position error={pos_err:.3f} m'
              f' rotation error={rot_err:.2f} deg{"" if ok else "  <-- FAIL"}')

    if failures:
        print(f'FAIL: {failures} poses beyond the error thresholds')
        return 1

    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
