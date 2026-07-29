#!/usr/bin/env python3
"""
Local test harness for the mrim_planner: runs the actual planner code with ROS
stubbed out and evaluates the produced trajectories exactly like mrim_manager
does (score, dynamic constraints, obstacle/mutual distances, final positions).

Usage (inside the local_eval venv):
    python harness/run_planner.py --problem apocalypse_small.problem
    python harness/run_planner.py --problem apocalypse_large.problem --config virtual
"""

import argparse
import os
import sys
import time

HARNESS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.abspath(os.path.join(HARNESS_DIR, '..', '..'))

sys.path.insert(0, os.path.join(HARNESS_DIR, 'stubs'))
sys.path.insert(0, os.path.join(REPO_ROOT, 'mrim_task', 'mrim_planner', 'scripts'))

import numpy as np
import yaml

import rospy  # the stub


# | -------------------- helpers ------------------- |

def flatten(d, prefix=''):
    out = {}
    for k, v in d.items():
        key = (prefix + '/' + str(k)) if prefix else str(k)
        if isinstance(v, dict):
            out.update(flatten(v, key))
        else:
            out[key] = v
    return out


def wrap_angle(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# | ------------------ evaluation ------------------- |

def evaluate(problem, trajs, params, solution_time):
    '''
    Mirror of the mrim_manager offline evaluation.
    trajs: list (per robot) of numpy arrays [n, 4] (x, y, z, heading)
    '''
    from scipy.spatial import cKDTree

    dt         = params.get('trajectories/dt', 0.2)
    tol        = params.get('dynamic_constraints/tolerance', 0.01)
    v_lim      = [params['dynamic_constraints/max_velocity/x'],
                  params['dynamic_constraints/max_velocity/y'],
                  params['dynamic_constraints/max_velocity/z'],
                  params['dynamic_constraints/max_heading_rate']]
    a_lim      = [params['dynamic_constraints/max_acceleration/x'],
                  params['dynamic_constraints/max_acceleration/y'],
                  params['dynamic_constraints/max_acceleration/z'],
                  params['dynamic_constraints/max_heading_rate_acceleration']]
    obst_lim   = params.get('trajectories/check/obstacles', 1.5)
    mutual_lim = params.get('trajectories/check/mutual', 2.5)
    vp_dist    = params.get('viewpoints/distance', 4.0)
    insp_dist  = params.get('viewpoints/inspection_limits/distance', 0.3)
    insp_hdg   = params.get('viewpoints/inspection_limits/heading', 0.2)
    mission_to = params.get('mission/timeout', 200.0)
    sol_soft   = params.get('solution_time_constraint/soft', 120.0)
    sol_hard   = params.get('solution_time_constraint/hard', 240.0)

    report    = []
    zero_score = False

    def check(ok, msg):
        nonlocal zero_score
        report.append('[{}] {}'.format('OK  ' if ok else 'FAIL', msg))
        if not ok:
            zero_score = True

    # --- dynamic constraints (manager: v = first diffs with v[0]=0; a = diffs of v) ---
    for r, T in enumerate(trajs):
        vel = np.zeros((len(T), 4))
        vel[1:, 0:3] = np.diff(T[:, 0:3], axis=0) / dt
        vel[1:, 3]   = np.array([wrap_angle(T[k, 3] - T[k - 1, 3]) for k in range(1, len(T))]) / dt
        acc          = np.zeros_like(vel)
        acc[1:]      = np.diff(vel, axis=0) / dt

        names = ['vel_x', 'vel_y', 'vel_z', 'vel_hdg']
        for ax in range(4):
            m = np.max(np.abs(vel[:, ax]))
            check(m < v_lim[ax] + tol, 'UAV{} {:8s} max {:6.3f} (limit {:.2f})'.format(r + 1, names[ax], m, v_lim[ax]))
        names = ['acc_x', 'acc_y', 'acc_z', 'acc_hdg']
        for ax in range(4):
            m = np.max(np.abs(acc[:, ax]))
            check(m < a_lim[ax] + tol, 'UAV{} {:8s} max {:6.3f} (limit {:.2f})'.format(r + 1, names[ax], m, a_lim[ax]))

    # --- obstacle distances ---
    obst = np.array([[o.x, o.y, o.z] for o in problem.obstacle_points])
    tree = cKDTree(obst)
    for r, T in enumerate(trajs):
        d_min = float(np.min(tree.query(T[:, 0:3], k=1)[0]))
        check(d_min > obst_lim, 'UAV{} min obstacle distance {:.3f} m (limit {:.2f})'.format(r + 1, d_min, obst_lim))

    # --- mutual distances (padded with the last pose) ---
    n_max = max(len(t) for t in trajs)
    idx   = np.arange(n_max)
    pos   = [t[np.minimum(idx, len(t) - 1), 0:3] for t in trajs]
    d     = np.linalg.norm(pos[0] - pos[1], axis=1)
    check(float(np.min(d)) > mutual_lim, 'min mutual distance {:.3f} m (limit {:.2f})'.format(float(np.min(d)), mutual_lim))

    # --- final positions ---
    for r, T in enumerate(trajs):
        sp = problem.start_poses[r]
        d_end = float(np.linalg.norm(T[-1, 0:3] - np.array([sp.position.x, sp.position.y, sp.position.z])))
        check(d_end <= 1.0, 'UAV{} final position distance {:.3f} m (limit 1.00)'.format(r + 1, d_end))

    # --- solution time ---
    hard_ok = solution_time <= sol_hard
    check(hard_ok, 'solution time {:.1f} s (soft {:.0f} s, hard {:.0f} s)'.format(solution_time, sol_soft, sol_hard))
    t_penalty = max(0.0, solution_time - sol_soft)

    # --- mission time ---
    t_mission = (n_max - 1) * dt
    ok = t_mission < mission_to
    report.append('[{}] mission time {:.1f} s (timeout {:.0f} s)'.format('OK  ' if ok else 'WARN', t_mission, mission_to))

    # --- inspections (playback mirror: latched, padded poses, per-robot inspectability) ---
    inspected = []
    padded = [t[np.minimum(idx, len(t) - 1)] for t in trajs]
    for ip in problem.inspection_points:
        ip_pos = np.array([ip.position.x, ip.position.y, ip.position.z])
        ok = False
        for r in range(problem.number_of_robots):
            if problem.robot_ids[r] not in ip.inspectability:
                continue
            dist_dev = np.abs(np.linalg.norm(padded[r][:, 0:3] - ip_pos, axis=1) - vp_dist)
            hdg_dev  = np.abs(wrap_angle(padded[r][:, 3] - ip.inspect_heading))
            if bool(np.any((dist_dev <= insp_dist) & (hdg_dev <= insp_hdg))):
                ok = True
                break
        inspected.append((ip.idx, ok))

    n_ok  = sum(1 for _, ok in inspected if ok)
    score = 0 if zero_score else n_ok

    return {
        'report': report,
        'zero_score': zero_score,
        'inspected': inspected,
        'n_inspected': n_ok,
        'n_points': len(problem.inspection_points),
        'score': score,
        'mission_time': t_mission,
        'solution_time': solution_time,
        'final_time': t_mission + t_penalty,
        'per_uav_times': [(len(t) - 1) * dt for t in trajs],
    }


# | --------------------- main ---------------------- |

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--problem', default=None, help='problem file name, e.g. apocalypse_small.problem (default: from config)')
    ap.add_argument('--config', default='virtual', choices=['virtual', 'real_world'], help='which planner config to use')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VALUE', help='override a parameter, e.g. --set path_planner/astar/grid_resolution=0.5')
    args = ap.parse_args()

    manager_cfg = os.path.join(REPO_ROOT, 'mrim_task', 'mrim_manager', 'config',
                               'default_offline.yaml' if args.config == 'virtual' else 'default_realworld.yaml')
    planner_cfg = os.path.join(REPO_ROOT, 'mrim_task', 'mrim_planner', 'config', args.config + '.yaml')

    params = {}
    with open(manager_cfg) as f:
        params.update(flatten(yaml.safe_load(f)))
    with open(planner_cfg) as f:
        params.update(flatten(yaml.safe_load(f)))

    params['session_problem'] = args.problem if args.problem else ''

    for kv in args.set:
        key, val = kv.split('=', 1)
        params[key] = yaml.safe_load(val)

    rospy.set_params(params)

    # point the LKH invoker to the locally built solver
    from solvers import LKHInvoker
    lkh_dir = os.environ.get('LKH_DIR', os.path.join(REPO_ROOT, 'local_eval', 'LKH-2.0.10'))
    LKHInvoker.LKHInvoker.LKH_DIR = lkh_dir + os.sep

    import planner as planner_module

    t0 = time.time()
    planner_module.MrimPlanner()
    solution_time = time.time() - t0

    # grab what the planner published
    problem_msg = rospy.get_published('~problem_out')[-1]
    traj_msgs   = [rospy.get_published('~trajectory_1_out')[-1], rospy.get_published('~trajectory_2_out')[-1]]
    trajs       = [np.array([[p.position.x, p.position.y, p.position.z, p.heading] for p in m.points]) for m in traj_msgs]

    result = evaluate(problem_msg, trajs, params, solution_time)

    print()
    print('==================== EVALUATION (mrim_manager mirror) ====================')
    for line in result['report']:
        print('  ' + line)
    missed = [idx for idx, ok in result['inspected'] if not ok]
    if missed:
        print('  [MISS] inspection points not inspected: {}'.format(missed))
    print('---------------------------------------------------------------------------')
    print('  UAV trajectory times: ' + ', '.join('{:.1f} s'.format(t) for t in result['per_uav_times']))
    print('  solution time: {:.1f} s | mission time: {:.1f} s | final time (T_I+T_P): {:.1f} s'.format(
        result['solution_time'], result['mission_time'], result['final_time']))
    print('  INSPECTED: {}/{}   SCORE: {}{}'.format(
        result['n_inspected'], result['n_points'], result['score'],
        '  (ZERO SCORE: constraint violation!)' if result['zero_score'] else ''))
    print('===========================================================================')

    return 0 if (not result['zero_score'] and result['n_inspected'] == result['n_points']) else 1


if __name__ == '__main__':
    sys.exit(main())
