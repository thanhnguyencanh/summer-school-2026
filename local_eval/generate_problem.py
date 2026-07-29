#!/usr/bin/env python3
"""
Generator of synthetic 'unseen' inspection problems similar to apocalypse_large.
Creates <name>.problem, obstacles/<name>.asc (surface-sampled point cloud) and
worlds/<name>.yaml inside mrim_task/mrim_resources.

Usage (inside the local_eval venv):
    python local_eval/generate_problem.py --name unseen_comp_a --seed 1 --n-ips 34
"""

import argparse
import os

import numpy as np
from scipy.spatial import cKDTree

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
RES_DIR   = os.path.join(REPO_ROOT, 'mrim_task', 'mrim_resources')

VP_DIST = 4.0  # viewpoint distance used by the virtual challenge


# | ------------------- structure sampling ------------------- |

def sample_box(rng, cx, cy, w, d, h, step=0.8):
    """Surface point samples of an axis-aligned box standing on the ground."""
    pts = []
    # walls
    for z in np.arange(0.0, h + step, step):
        for x in np.arange(-w / 2, w / 2 + step, step):
            pts.append((cx + x, cy - d / 2, z))
            pts.append((cx + x, cy + d / 2, z))
        for y in np.arange(-d / 2, d / 2 + step, step):
            pts.append((cx - w / 2, cy + y, z))
            pts.append((cx + w / 2, cy + y, z))
    # roof
    for x in np.arange(-w / 2, w / 2 + step, step):
        for y in np.arange(-d / 2, d / 2 + step, step):
            pts.append((cx + x, cy + y, h))
    return pts


def sample_cylinder(rng, cx, cy, r, h, step=0.7):
    pts = []
    n_ang = max(8, int(2 * np.pi * r / step))
    for z in np.arange(0.0, h + step, step):
        for a in np.linspace(0, 2 * np.pi, n_ang, endpoint=False):
            pts.append((cx + r * np.cos(a), cy + r * np.sin(a), z))
    # top disc
    for rr in np.arange(0, r + step, step):
        n_a = max(4, int(2 * np.pi * rr / step))
        for a in np.linspace(0, 2 * np.pi, n_a, endpoint=False):
            pts.append((cx + rr * np.cos(a), cy + rr * np.sin(a), h))
    return pts


def sample_mound(rng, cx, cy, r, step=0.7):
    """Rubble mound: upper hemisphere."""
    pts = []
    for th in np.arange(0, np.pi / 2 + 0.1, step / r):
        rr = r * np.cos(th)
        z  = r * np.sin(th)
        n_a = max(4, int(2 * np.pi * rr / step))
        for a in np.linspace(0, 2 * np.pi, n_a, endpoint=False):
            pts.append((cx + rr * np.cos(a), cy + rr * np.sin(a), z))
    return pts


# | --------------------- world building --------------------- |

def build_world(rng, half_x, half_y, n_structures, min_gap, starts):
    """Places structures with guaranteed corridors, returns (points, structures)."""

    structures = []  # (cx, cy, radius_footprint, height, kind)
    obstacles  = []

    # ground plane (with jitter, like the original .asc)
    for x in np.arange(-half_x, half_x + 1.0, 1.0):
        for y in np.arange(-half_y, half_y + 1.0, 1.0):
            obstacles.append((x + rng.uniform(-0.2, 0.2), y + rng.uniform(-0.2, 0.2), 0.0))

    attempts = 0
    while len(structures) < n_structures and attempts < 4000:
        attempts += 1
        kind = rng.choice(['box', 'box', 'box', 'cyl', 'mound'])

        cx = rng.uniform(-half_x + 6, half_x - 6)
        cy = rng.uniform(-half_y + 6, half_y - 6)

        if kind == 'box':
            w, d = rng.uniform(4.5, 11.0), rng.uniform(4.5, 11.0)
            h    = rng.uniform(5.0, 22.0)
            foot = float(np.hypot(w, d)) / 2.0
        elif kind == 'cyl':
            r    = rng.uniform(1.5, 3.2)
            h    = rng.uniform(8.0, 24.0)
            foot = r
        else:
            r    = rng.uniform(3.0, 6.0)
            h    = r
            foot = r

        # keep flyable corridors between structures and clear zones at the starts
        ok = all(np.hypot(cx - sx, cy - sy) > foot + sfoot + min_gap for sx, sy, sfoot, _, _ in structures)
        ok = ok and all(np.hypot(cx - s[0], cy - s[1]) > foot + 7.0 for s in starts)
        if not ok:
            continue

        if kind == 'box':
            pts = sample_box(rng, cx, cy, w, d, h)
        elif kind == 'cyl':
            pts = sample_cylinder(rng, cx, cy, foot, h)
        else:
            pts = sample_mound(rng, cx, cy, foot)

        obstacles.extend(pts)
        structures.append((cx, cy, foot, h, kind))

    return obstacles, structures


# | ------------------ inspection point gen ------------------ |

def vp_of(ip, heading, tilt):
    x = ip[0] + VP_DIST * np.cos(heading) * np.sin(tilt)
    y = ip[1] + VP_DIST * np.sin(heading) * np.sin(tilt)
    z = ip[2] + VP_DIST * np.cos(tilt)
    return np.array([x, y, z])


def gen_ips(rng, tree, structures, starts, n_ips, half_x, half_y, max_z,
            clearance_lo, clearance_hi, purple_frac):
    ips = []
    vps = []
    attempts = 0

    while len(ips) < n_ips and attempts < 20000:
        attempts += 1

        cx, cy, foot, h, kind = structures[rng.integers(0, len(structures))]

        tilt_choice = rng.random()
        if tilt_choice < 0.6:
            tilt = -0.79
        elif tilt_choice < 0.75:
            tilt = -1.57
        elif tilt_choice < 0.9:
            tilt = -0.52
        else:
            tilt = 0.0

        if tilt == 0.0:
            # point on the roof/top, camera looking straight down from above
            ip      = np.array([cx + rng.uniform(-0.3, 0.3) * foot, cy + rng.uniform(-0.3, 0.3) * foot, h])
            heading = rng.uniform(-np.pi, np.pi)
        else:
            # point on a side of the structure, camera looking horizontally into it
            a       = rng.uniform(-np.pi, np.pi)             # outward direction from the structure axis
            ip      = np.array([cx + foot * np.cos(a), cy + foot * np.sin(a), rng.uniform(1.0, max(1.5, h - 0.5))])
            heading = np.arctan2(-np.sin(a), -np.cos(a))     # look back towards the structure

        vp = vp_of(ip, heading, tilt)

        # viewpoint feasibility
        clearance = float(tree.query(vp, k=1)[0])
        if not (clearance_lo <= clearance):
            continue
        if clearance > clearance_hi and rng.random() < 0.35:
            continue  # keep a share of tight viewpoints
        if not (1.2 <= vp[2] <= max_z - 1.0):
            continue
        if abs(vp[0]) > half_x - 1.0 or abs(vp[1]) > half_y - 1.0:
            continue
        if any(np.linalg.norm(vp - np.array(s)) < 3.4 for s in starts):
            continue
        if any(np.linalg.norm(vp - v) < 1.2 for v in vps):
            continue
        if any(np.linalg.norm(ip - i[0]) < 2.5 for i in ips):
            continue

        # inspectability: mostly shared, some exclusive
        r = rng.random()
        if r < purple_frac:
            insp = [1, 2]
        elif r < purple_frac + (1 - purple_frac) / 2:
            insp = [1]
        else:
            insp = [2]

        ips.append((ip, heading, tilt, insp))
        vps.append(vp)

    return ips


# | --------------------------- main ------------------------- |

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--name', required=True)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--n-ips', type=int, default=34)
    ap.add_argument('--half-x', type=float, default=33.0)
    ap.add_argument('--half-y', type=float, default=43.0)
    ap.add_argument('--max-z', type=float, default=40.0)
    ap.add_argument('--structures', type=int, default=26)
    ap.add_argument('--min-gap', type=float, default=5.2, help='min horizontal gap between structure footprints (m)')
    ap.add_argument('--clearance-lo', type=float, default=2.15, help='min viewpoint clearance from obstacles (m)')
    ap.add_argument('--clearance-hi', type=float, default=3.5, help='clearance above which a share of viewpoints is rejected (keeps tight ones)')
    ap.add_argument('--purple-frac', type=float, default=0.85)
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)

    starts = [(0.0, 4.5, 11.5), (0.0, -4.5, 15.5)]

    obstacles, structures = build_world(rng, args.half_x, args.half_y, args.structures, args.min_gap, starts)
    tree = cKDTree(np.array(obstacles))

    # starts must be collision-free with margin
    for s in starts:
        assert tree.query(np.array(s), k=1)[0] > 2.5, 'start position too close to obstacles'

    ips = gen_ips(rng, tree, structures, starts, args.n_ips, args.half_x, args.half_y, args.max_z,
                  args.clearance_lo, args.clearance_hi, args.purple_frac)

    if len(ips) < args.n_ips:
        print('[WARN] only generated {:d}/{:d} inspection points'.format(len(ips), args.n_ips))

    # write the obstacle cloud
    asc_path = os.path.join(RES_DIR, 'obstacles', args.name + '.asc')
    with open(asc_path, 'w') as f:
        for p in obstacles:
            f.write('{:.6f} {:.6f} {:.6f} \n'.format(p[0], p[1], p[2]))

    # write the world file
    world_path = os.path.join(RES_DIR, 'worlds', args.name + '.yaml')
    with open(world_path, 'w') as f:
        f.write('world_origin:\n\n')
        f.write('  units: "LATLON"\n\n')
        f.write('  origin_x: 47.397743\n')
        f.write('  origin_y: 8.545594\n\n')
        f.write('safety_area:\n\n')
        f.write('  enabled: true\n\n')
        f.write('  horizontal:\n\n')
        f.write('    frame_name: "world_origin"\n\n')
        f.write('    points: [\n')
        f.write('      {:.1f}, {:.1f},\n'.format(args.half_x, args.half_y))
        f.write('      {:.1f}, {:.1f},\n'.format(args.half_x, -args.half_y))
        f.write('      {:.1f}, {:.1f},\n'.format(-args.half_x, -args.half_y))
        f.write('      {:.1f}, {:.1f}\n'.format(-args.half_x, args.half_y))
        f.write('    ]\n\n')
        f.write('  vertical:\n\n')
        f.write('    frame_name: "world_origin"\n\n')
        f.write('    max_z: {:.1f}\n'.format(args.max_z))
        f.write('    min_z: -2\n')

    # write the problem file
    problem_path = os.path.join(RES_DIR, 'problems', args.name + '.problem')
    with open(problem_path, 'w') as f:
        f.write('NAME: Synthetic unseen problem ({})\n'.format(args.name))
        f.write('COMMENT: generated by local_eval/generate_problem.py, seed {}\n\n'.format(args.seed))
        f.write('# ID X Y Z HEADING\n')
        f.write('ROBOTS_START\n')
        for k, s in enumerate(starts):
            f.write('{:d} {:.1f} {:.1f} {:.1f} -1.57\n'.format(k + 1, s[0], s[1], s[2]))
        f.write('ROBOTS_END\n\n')
        f.write('# ID X Y Z HEADING TILT INSPECTABILITY\n')
        f.write('INSPECTION_POINTS_START\n')
        for k, (ip, heading, tilt, insp) in enumerate(ips):
            f.write('{:d} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:s}\n'.format(
                k + 1, ip[0], ip[1], ip[2], heading, tilt, ' '.join(str(i) for i in insp)))
        f.write('INSPECTION_POINTS_END\n\n')
        f.write('OBSTACLE_POINTS: {}.asc\n'.format(args.name))
        f.write('WORLD: {}.yaml\n\n'.format(args.name))
        f.write('EOF\n')

    n_purple = sum(1 for i in ips if len(i[3]) == 2)
    print('generated {}: {} obstacle points, {} structures, {} IPs ({} shared, {} exclusive)'.format(
        args.name, len(obstacles), len(structures), len(ips), n_purple, len(ips) - n_purple))


if __name__ == '__main__':
    main()
