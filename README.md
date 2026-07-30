# MRS Summer School 2026 — Multi-Robot Inspection (Team Solution)

Two UAVs (🟥 red, 🟦 blue) must inspect a set of inspection points (IPs) in a 3D post-disaster environment as fast as possible: plan collision-free, dynamics-feasible trajectories that visit the viewpoint of every IP and return to the start. Score = number of inspected IPs; ties are broken by mission time + computation-time penalty. Any constraint violation (dynamics, 1.5 m obstacle distance, 2.5 m mutual distance, final position, hard compute limit) zeroes the score.

This repository is our team's solution built on the official [ctu-mrs/summer-school-2026](https://github.com/ctu-mrs/summer-school-2026) task (kept as the `upstream` remote — pull organizer fixes with `git pull upstream master`).

## Our solution

All changes live in [mrim_task/mrim_planner](mrim_task/mrim_planner) (the only folder used in the competition evaluation):

1. **Heading interpolation** — linear in traveled distance along each leg ([trajectory.py](mrim_task/mrim_planner/scripts/trajectory.py)).
2. **TSP over flight-time estimates** — per-axis trapezoidal time + heading-rotation time instead of Euclidean distance, obstructed straight lines penalized by a detour factor ([tsp_solvers.py](mrim_task/mrim_planner/scripts/solvers/tsp_solvers.py)).
3. **Viewpoint assignment** — k-means clustering of shared IPs + makespan balancing between the UAVs driven by a fast NN+2-opt tour estimator.
4. **Fast A\*** — heapq + numpy occupancy grid, endpoint free-cell snapping, exact metric endpoints, recursive straightening + greedy shortcut ([astar.py](mrim_task/mrim_planner/scripts/path_planners/grid_based/astar.py)); finished RRT/RRT\* as automatic fallback.
5. **Continuous trajectories** — viewpoint-exact pure-pursuit smoothing, heading held constant around viewpoints, TOPPRA time parametrization, no stops at waypoints.
6. **UAV↔UAV collision avoidance** — minimal start-delay search (either UAV, parked-UAV padding, mission-time optimal) + keepout spheres around the other UAV's start.
7. **Safety margins** — dynamics scaled to 97 % (virtual) / 90 % (real world), obstacle planning margin, mutual-distance margin.
8. **Self-check** — after planning, every zero-score condition is verified and printed as an OK/FAIL audit.
9. **Robustness fallbacks** — LKH failure → built-in 2-opt; TOPPRA failure → stop-at-waypoints sampling; A\* failure → RRT. No single failure can zero the score.
10. **Unseen-world validation** — [local_eval/generate_problem.py](local_eval/generate_problem.py) generates worlds similar to (and harder than) `apocalypse_large`; the solution is validated on 6 such worlds besides the 3 official ones.

11. **Shell-pose optimization (DP)** — for the fixed tour order, the actual inspection pose of every IP is chosen on its tolerance sphere (exact radius + exact heading kept as margin, position varies within a cone around the nominal viewpoint direction) by dynamic programming over flight-time estimates — each UAV approaches every IP from the direction it is already flying (`tsp/shell_dp`). Validated with an automatic safety net: if the planner's self-check finds any violation or a missed inspection, it replans once with the aggressive features disabled.

### Results (evaluation logic identical to mrim_manager, all checks green)

| Problem | Score | Mission time | Planning time |
|---|:---:|:---:|:---:|
| apocalypse_small | 8/8 | 25.4 s | ~6 s |
| apocalypse_moderate | 16/16 | 44.8 s | ~8 s |
| apocalypse_large | 29/29 | 67.4 s | ~13 s |
| unseen_comp_a/b/c | 34+33+35 (all) | 75.4–79.6 s | ~15 s |
| unseen_dense | 34/34 | 86.0 s | ~16 s |
| unseen_tight | 32/32 | 84.0 s | ~15 s |
| unseen_wide40 | 40/40 | 87.0 s | ~16 s |

(Official `mrim_manager` in the Apptainer container reproduces these numbers exactly.)

## Setup

Requirements: Linux (Ubuntu recommended; WSL2 works), ~6 GB disk, [Apptainer](https://apptainer.org/) (installed automatically on Ubuntu).

```bash
mkdir -p ${HOME}/git
cd ${HOME}/git && git clone https://github.com/thanhnguyencanh/summer-school-2026.git
cd summer-school-2026 && ./install.sh   # installs Apptainer, downloads MRS image, compiles the workspace
```

`install.sh` runs the three steps below — they can also be run individually from `simulation/` (useful for partial re-runs):

| Script | What it does | When to re-run |
|---|---|---|
| `simulation/01_install.sh` | installs Apptainer via apt (**needs sudo**, Ubuntu only) | once per machine |
| `simulation/02_download.sh` | downloads the ~3.5 GB MRS image to `simulation/images/` (resumable; **deletes the old image first**) | on image updates |
| `simulation/03_compile.sh` | `catkin build` of the workspace inside the container | after changing C++ code (`mrim_state_machine`); Python/config changes need no rebuild |

## Testing & evaluation

### 1. Official offline evaluation (recommended before any submission)

```bash
./simulation/run_offline.sh      # plans + evaluates + RViz playback (press the 'Send Topic' start button in RViz)
```

The planner prints a `SELF-CHECK OF THE SOLUTION` block (every line must be `[OK]`); the manager prints `FINAL SCORE: X/Y` after the playback. Switch problems in [mrim_task/mrim_planner/config/virtual.yaml](mrim_task/mrim_planner/config/virtual.yaml) (`problem/name`), options: `apocalypse_{small,moderate,large}.problem` and our generated `unseen_*.problem` worlds.

### 2. Full simulation (real-time dynamics, two simulated UAVs)

```bash
./simulation/run_simulation.sh   # tmux session; switch panes with Shift+←/→ (planner, state_machine, control)
./simulation/kill_simulation.sh  # stop it
```

### 3. Quick headless testing (no ROS/Apptainer — any machine, ~1 min setup)

```bash
./local_eval/setup.sh            # one-time: creates venv + builds the LKH solver (needs python3-venv, gcc, wget)
./local_eval/venv/bin/python local_eval/harness/run_planner.py --problem apocalypse_large.problem
./local_eval/venv/bin/python local_eval/harness/run_planner.py --problem unseen_dense.problem --config real_world
./local_eval/venv/bin/python local_eval/harness/run_planner.py --problem apocalypse_large.problem --set path_smoothing/lookahead_dist=1.2
```

Runs the real planner code with ROS stubbed out and evaluates it exactly like `mrim_manager` (score, dynamics, distances, final positions). The run must end with all `OK` and `INSPECTED` equal to the number of points. Generate more test worlds with `local_eval/generate_problem.py --name my_world --seed 7 --n-ips 34`.

## Competition submission

Deadline: **Monday, August 3, 11:59 p.m.** via the [Google form](https://docs.google.com/forms/d/1jcoQr2TPbzro8bFdiUZ3yzLbKQDW0foHmKn6kjHJXjQ) (team name, members, zip archive).

```bash
cd mrim_task && zip -r team_name.zip mrim_planner
```

Checklist before zipping:
- [ ] `run_offline.sh` passes with full score on `apocalypse_large` and several `unseen_*` worlds
- [ ] `config/virtual.yaml` and `config/real_world.yaml` are both tuned (the same zip is used for both challenges)
- [ ] planner self-check prints all `[OK]`

## Task reference

### Constraints

| Constraint | Virtual | Real-world |
|---|:---:|:---:|
| Max solution time (soft / hard) | 120 s / 200 s | 80 s / 150 s |
| Max mission time | 200 s | 240 s |
| Max velocity x,y / z | 5 / 2 m/s | 3 / 1 m/s |
| Max acceleration x,y / z | 5 / 2 m/s² | 3 / 1 m/s² |
| Max heading rate / acceleration | 1 rad/s / 2 rad/s² | 1 rad/s / 2 rad/s² |
| Min obstacle / mutual distance | 1.5 / 2.5 m | 2.0 / 4.0 m |
| Final point within start tolerance | 1.0 m | 1.0 m |

An IP is inspected when the UAV is within 0.3 m of the viewpoint distance sphere with heading within 0.2 rad (evaluated per 0.2 s sample; real-world uses the actual UAV pose, not the reference).

### Where the code lives

- `mrim_task/mrim_planner/scripts/planner.py` — entry point, parameters, self-check
- `scripts/trajectory.py` — smoothing, heading, TOPPRA sampling, collision avoidance
- `scripts/solvers/tsp_solvers.py` — clustering, balancing, TSP, path-planning fallback chain
- `scripts/path_planners/` — A\* (grid) and RRT/RRT\* (sampling)
- `config/virtual.yaml`, `config/real_world.yaml` — all tunables
- `mrim_task/mrim_manager/` — the evaluator (**do not modify**; its config defines dt, limits, viewpoint distance)

### Troubleshooting

- `open terminal failed: missing or unsuitable terminal` → run `./simulation/kill_simulation.sh`
- Apptainer on non-Ubuntu: install manually, see [official docs](https://apptainer.org/docs/user/main/quick_start.html#installation)
- Organizer contacts: Václav Riss `rissvacl@fel.cvut.cz`, Jindřich Třaskoš `traskjin@fel.cvut.cz`, Martin Zoula `zoulamar@fel.cvut.cz`

### References

- [1] Baca, T. et al., [The MRS UAV System](https://arxiv.org/pdf/2008.08050), *JINT 102(26), 2021* — https://github.com/ctu-mrs/mrs_uav_system
- [2] Pham, H., Pham, Q. C., [TOPPRA: time-optimal path parameterization](https://hungpham2511.github.io/toppra/)
